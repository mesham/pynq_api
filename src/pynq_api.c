/*
* BSD 3-Clause License
*
* Copyright (c) 2019, Nick Brown
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <byteswap.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/utsname.h>
#include "pynq_api.h"
#include "libxlnk_cma.h"

// These three definitions are used for writing the bitstream payload to the PL and launching it
#define BS_FPGA_MAN_FLAGS "/sys/class/fpga_manager/fpga0/flags"
#define BS_FPGA_MAN "/sys/class/fpga_manager/fpga0/firmware"
#define FIRMWARE_PATH_PREFIX "/lib/firmware/"
// Used for creating memory windows
#define MEMORY_DEV_PATH "/dev/mem"
// Location of GPIO interfaces
#define GPIO_PATH "/sys/class/gpio"
// Used for handling interrupts raised by the PLS via the UIO interface
#define INTERRUPTS_FILE "/proc/interrupts"
#define UIO_DESCRIPTION_PATH "/sys/class/uio"
#define UIO_PATH "/dev"
// Number of bytes to read at a time when we load in the raw bitstream file
#define READ_CHUNK_SIZE 1024
// Definitions below here are specific to the Zynq 7-Series PS and PL clocks
#define ZYNQ_SERIES_7_IDENTIFIER "armv7l"
#define SLCR_BASE_ADDRESS 0xF8000000
#define ARM_CLK_CTRL_OFFSET 0x120
#define ARM_PLL_CTRL_OFFSET 0x100
#define PL_FCLK0_CTRL_OFFSET 0x170
#define PL_FCLK0_CTRL_SPACING 0x10
#define DEFAULT_SRC_CLK_MHZ 50.0

void _xlnk_reset();
static int checkMachineType();
static int getSrcClkMHZ(PYNQ_REGISTER*, size_t, float*);
static int enable_uio(PYNQ_UIO*);
static void pullRaisedIRQs(PYNQ_AXI_INTERRUPT_CONTROLLER*);
static void appendOrInsertRaisedInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER*, int);
static struct raised_interrupt_struct * findRaisedInterruptStructure(PYNQ_AXI_INTERRUPT_CONTROLLER*, int);
static char * get_device_path(char*);
static char * get_uio_irq_devname(int);
static int issueDMATransfer(PYNQ_AXI_DMA*, PYNQ_SHARED_MEMORY*, size_t, size_t, AXI_DMA_DIRECTION);
static int startDMAChannel(PYNQ_AXI_DMA*, AXI_DMA_DIRECTION);
static int stopDMAChannel(PYNQ_AXI_DMA*, AXI_DMA_DIRECTION);
static int isDMAChannelRunning(PYNQ_AXI_DMA*, AXI_DMA_DIRECTION);
static int isDMAChannelIdle(PYNQ_AXI_DMA*, AXI_DMA_DIRECTION);
static char * getBaseNameFromFile(char*);
static char * getBinNameFromBit(char*);
static char * extractBitstreamPayload(char*, int*);
static void extractBitstreamInfo(char*, PYNQ_BITSTREAM_INFO*);
static char * readRawBitfile(char*);
static void freeHeader(PYNQ_BITSTREAM_INFO*);
static int PYNQ_setupUIO(PYNQ_UIO * state);

/**
* Returns the time in microseconds since the epoch
*/
unsigned long long PYNQ_Wtime() {
  struct timeval time_val;
  gettimeofday(&time_val, NULL);
  return time_val.tv_sec * 1000000 + time_val.tv_usec;
}

/**
* Opens an HLS block to then interact with this. This is a thin layer of abstraction over the HLS memory mapped
* interface and assumes the ap_ctrl_hs protocol. The width is the size of memory window to open on the HLS block
* as many implementations also pass arguments via the control bus
*/
int PYNQ_openHLS(PYNQ_HLS * state, size_t address, size_t width) {
  return PYNQ_createMMIOWindow(&(state->mmio_window), address, width);
}

/**
* Starts execution of an HLS block
*/
int PYNQ_startHLS(PYNQ_HLS * state) {
  char byte_command=1;
  return PYNQ_writeMMIO(&(state->mmio_window), &byte_command, 0x0, sizeof(char));
}

int PYNQ_stopHLS(PYNQ_HLS * state) {
  char byte_command=0;
  return PYNQ_writeMMIO(&(state->mmio_window), &byte_command, 0x0, sizeof(char));
}

/**
* Tests whether an HLS block has completed execution or not. More specifically whether an HLS
* block is idle or not (as we also assume that 0x00 status marks completion.) This completion
* is written out into the flag argument provided
*/
int PYNQ_testHLSCompleted(PYNQ_HLS * state, int * flag) {
  char byte_status;
  if (!PYNQ_readMMIO(&(state->mmio_window), &byte_status, 0x0, sizeof(char))) return PYNQ_ERROR;
  *flag = byte_status == 4 || byte_status == 6 || byte_status == 0;
  return PYNQ_SUCCESS;
}

/**
* Waits for an HLS kernel to complete (or more accurately for an HLS kernel to be idle)
*/
int PYNQ_waitForHLS(PYNQ_HLS * state) {
  int isFinished=0;
  while (!isFinished) {
    if(!PYNQ_testHLSCompleted(state, &isFinished)) return PYNQ_ERROR;
  }
  return PYNQ_SUCCESS;
}

/**
* Writes some data of a specified size to some offset in the HLS block. This is just a (very) thin
* layer of abstraction over the memory mapped interface
*/
int PYNQ_writeToHLS(PYNQ_HLS * state, void * data, size_t offset, size_t size) {
  return PYNQ_writeMMIO(&(state->mmio_window), data, offset, size);
}

/**
* Reads some data of a specified size to some offset from the HLS block. This is just a (very) thin
* layer of abstraction over the memory mapped interface
*/
int PYNQ_readFromHLS(PYNQ_HLS * state, void * data, size_t offset, size_t size) {
  return PYNQ_readMMIO(&(state->mmio_window), data, offset, size);
}

/**
* Closes the interaction with an HLS block (note that this has no impact on the block itself, just the
* host connection to the block.)
*/
int PYNQ_closeHLS(PYNQ_HLS * state) {
  return PYNQ_closeMMIOWindow(&(state->mmio_window));
}

/**
* Retrieves the CPU clock frequency in Mhz and returns this via the freq flag argument
*/
int PYNQ_getCPUClockFreq(float * freq) {
  if (!checkMachineType()) return PYNQ_ERROR;
  PYNQ_REGISTER reg;
  if (!PYNQ_openRegister(&reg, SLCR_BASE_ADDRESS, 65536)) return PYNQ_ERROR;
  unsigned int arm_clk_odiv, pll_fbdiv;
  if (!PYNQ_readRegisterRange(&reg, ARM_CLK_CTRL_OFFSET, 8, 13, &arm_clk_odiv)) return PYNQ_ERROR;
  if (!PYNQ_readRegisterRange(&reg, ARM_PLL_CTRL_OFFSET, 12, 18, &pll_fbdiv)) return PYNQ_ERROR;
  *freq=(DEFAULT_SRC_CLK_MHZ * pll_fbdiv) / arm_clk_odiv;
  return PYNQ_closeRegister(&reg);
}

/**
* Retrieves the clock frequency of one of the four PL clocks as specified by the clock index
* argument and writes this into the freq argument
*/
int PYNQ_getPLClockFreq(unsigned int clock_index, float * freq) {
  if (!checkMachineType()) return PYNQ_ERROR;
  if (clock_index >= 4) {
    fprintf(stderr, "Clock index is 0 to 3 on the Zynq, not %d\n", clock_index);
    return PYNQ_ERROR;
  }
  PYNQ_REGISTER reg;
  if (!PYNQ_openRegister(&reg, SLCR_BASE_ADDRESS, 65536)) return PYNQ_ERROR;
  size_t offset=PL_FCLK0_CTRL_OFFSET + (PL_FCLK0_CTRL_SPACING * clock_index);
  float src_clk_mhz;
  if (!getSrcClkMHZ(&reg, offset, &src_clk_mhz)) return PYNQ_ERROR;

  unsigned int pl_clk_odiv0, pl_clk_odiv1;
  if (!PYNQ_readRegisterRange(&reg, offset, 20, 25, &pl_clk_odiv0)) return PYNQ_ERROR;
  if (!PYNQ_readRegisterRange(&reg, offset, 8, 13, &pl_clk_odiv1)) return PYNQ_ERROR;

  *freq=src_clk_mhz / (pl_clk_odiv0 * pl_clk_odiv1);
  return PYNQ_closeRegister(&reg);
}

/**
* Sets the clock frequency of one of the four PL clocks as specified by the clock index. The
* caller must provide the frequency along with either div0 or div1.
*/
int PYNQ_setPLClockFreq(unsigned int clock_index, float * freq, int * div0, int *div1) {
  if (!checkMachineType()) return PYNQ_ERROR;
  if (clock_index >= 4) {
    fprintf(stderr, "Clock index is 0 to 3 on the Zynq, not %d\n", clock_index);
    return PYNQ_ERROR;
  }

  if (div0 == NULL && div1 == NULL) {
    fprintf(stderr, "Must provide either div0 or div1 when setting the clock\n");
    return PYNQ_ERROR;
  }

  PYNQ_REGISTER reg;
  if (!PYNQ_openRegister(&reg, SLCR_BASE_ADDRESS, 65536)) return PYNQ_ERROR;
  size_t offset=PL_FCLK0_CTRL_OFFSET + (PL_FCLK0_CTRL_SPACING * clock_index);
  float src_clk_mhz;
  if (!getSrcClkMHZ(&reg, offset, &src_clk_mhz)) return PYNQ_ERROR;
  int my_d0, my_d1;

  if (div0 != NULL && div1 == NULL) {
    my_d0=*div0;
    my_d1 = src_clk_mhz / *freq / my_d0;
  } else if (div0 == NULL && div1 != NULL) {
    my_d1=*div1;
    my_d0 = src_clk_mhz / *freq / my_d1;
  } else {
    my_d0=*div0;
    my_d1=*div1;
  }

  unsigned int pl_clk_odiv0, pl_clk_odiv1;
  if (!PYNQ_readRegisterRange(&reg, offset, 20, 25, &pl_clk_odiv0)) return PYNQ_ERROR;
  if (!PYNQ_readRegisterRange(&reg, offset, 8, 13, &pl_clk_odiv1)) return PYNQ_ERROR;

  if (my_d0 <= 0 || my_d0 > ((1 << pl_clk_odiv0) - 1)) {
    fprintf(stderr, "Frequency divider 0 value of '%d' out of range\n", my_d0);
    return PYNQ_ERROR;
  }

  if (my_d1 <= 0 || my_d1 > ((1 << pl_clk_odiv1) - 1)) {
    fprintf(stderr, "Frequency divider 1 value of '%d' out of range\n", my_d1);
    return PYNQ_ERROR;
  }

  if (!PYNQ_setRegisterRange(&reg, offset, 20, 25, &my_d0)) return PYNQ_ERROR;
  if (!PYNQ_setRegisterRange(&reg, offset, 8, 13, &my_d1)) return PYNQ_ERROR;

  return PYNQ_closeRegister(&reg);
}

/**
* Opens a register for interaction at a specific address and width
*/
int PYNQ_openRegister(PYNQ_REGISTER* state, size_t address, size_t width) {
  return PYNQ_createMMIOWindow(&(state->mmio_window), address, width);
}

/**
* Reads the register at a specific offset (address) and bit index. This bit is then returned in
* the data argument
*/
int PYNQ_readRegister(PYNQ_REGISTER* state, size_t address, int index, unsigned int * data) {
  unsigned int read_data;
  if (PYNQ_readMMIO(&(state->mmio_window), &read_data, address, sizeof(unsigned int)) == PYNQ_ERROR) {
    return PYNQ_ERROR;
  }
  unsigned int mask = 1 << index;
  *data=(read_data & mask) >> index;
  return PYNQ_SUCCESS;
}

/**
* Reads a range of bits from a 32 bit window on the register at offset address. These read bits must be contiguous between
* the start_index and stop_index, and are written into the data argument
*/
int PYNQ_readRegisterRange(PYNQ_REGISTER* state, size_t address, int start_index, int stop_index, unsigned int * data) {
  unsigned int read_data;
  if (PYNQ_readMMIO(&(state->mmio_window), &read_data, address, sizeof(unsigned int)) == PYNQ_ERROR) {
    return PYNQ_ERROR;
  }
  if (start_index >= stop_index) {
    unsigned int mask = ((1 << (start_index - stop_index + 1)) - 1) << stop_index;
    *data=(read_data & mask) >> stop_index;
  } else {
    unsigned int width = stop_index - start_index + 1;
    unsigned int mask = ((1 << width) - 1) << start_index;
    *data=(read_data & mask) >> start_index;
  }
  return PYNQ_SUCCESS;
}

/**
* Sets the bit value of a register at offset address. The bit index to write to is provided as is the data itself to write at that bit
*/
int PYNQ_setRegister(PYNQ_REGISTER* state, size_t address, int index, unsigned int * data) {
  unsigned int read_data, write_data;
  if (PYNQ_readMMIO(&(state->mmio_window), &read_data, address, sizeof(unsigned int)) == PYNQ_ERROR) {
    return PYNQ_ERROR;
  }

  unsigned int mask = 1 << index;
  write_data=(read_data & ~mask) | (*data << index);
  return PYNQ_writeMMIO(&(state->mmio_window), &write_data, 0x00, sizeof(unsigned int));
}

/**
* Sets the bit values of a register between a range of a 32 bit window at offset address. The start_index to stop_index are
* assumed to be contiguous and the data provided in the data argument
*/
int PYNQ_setRegisterRange(PYNQ_REGISTER* state, size_t address, int start_index, int stop_index, unsigned int * data) {
  unsigned int read_data, write_data;
  if (PYNQ_readMMIO(&(state->mmio_window), &read_data, address, sizeof(unsigned int)) == PYNQ_ERROR) {
    return PYNQ_ERROR;
  }

  unsigned int count = start_index >= stop_index ? start_index-stop_index+1 : stop_index-start_index+1;
  unsigned int shift = start_index >= stop_index ? stop_index : start_index;
  unsigned int mask = ((1 << count) - 1) << shift;

  write_data=(read_data & ~mask) | (*data << shift);
  return PYNQ_writeMMIO(&(state->mmio_window), &write_data, 0x00, sizeof(unsigned int));
}

/**
* Closes a register
*/
int PYNQ_closeRegister(PYNQ_REGISTER* state) {
  return PYNQ_closeMMIOWindow(&(state->mmio_window));
}

/**
* Opens an AXI interrupt controller for interaction at a specific base address. Whilst this isn't
* strictly speaking part of the core ZYNQ eco-system, this is a very common IP block and hence we support it
*/
int PYNQ_openInterruptController(PYNQ_AXI_INTERRUPT_CONTROLLER * state, size_t base_address) {
  PYNQ_createMMIOWindow(&(state->mmio_window), base_address, 128);
  state->raised_interrupt_head=NULL;
  state->enabled_hw_interrupts=0;

  unsigned int command=0x00; //disable all interrupt lines by default (enabled on an interrupt by interrupt basis)
  PYNQ_writeMMIO(&(state->mmio_window), &command, 0x8, sizeof(unsigned int));

  return PYNQ_SUCCESS;
}

/**
* Registers the tracking of an interrupt by the AXI interrupt controller, note that the IRQ number is the number as the
* controller sees it and bares no relationship to the IRQ number at the ZYNQ processing system. The caller can specify whether
* the raising of this interrupt will generate a hardware interrupt on the irq pin of the controller.
*/
int PYNQ_registerInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER * state, int irq_number, int generate_hw_interrupt) {
  struct raised_interrupt_struct * interrupt = findRaisedInterruptStructure(state, irq_number);
  if (interrupt == NULL) {
    interrupt=(struct raised_interrupt_struct *) malloc(sizeof(struct raised_interrupt_struct));
    interrupt->irq=irq_number;
    interrupt->number=0;
    interrupt->active_waiting=0;
    interrupt->next=state->raised_interrupt_head;
    state->raised_interrupt_head=interrupt;
  }

  if (!interrupt->active_waiting) {
    if (generate_hw_interrupt) {
      int new_hw_interrupts=state->enabled_hw_interrupts | (1 << irq_number);
      if (new_hw_interrupts != state->enabled_hw_interrupts) {
        state->enabled_hw_interrupts=new_hw_interrupts;
        PYNQ_writeMMIO(&(state->mmio_window), &new_hw_interrupts, 0x8, sizeof(unsigned int));
      }
    }
    interrupt->active_waiting=1;
    unsigned int irq_bit=1 << irq_number;
    PYNQ_writeMMIO(&(state->mmio_window), &irq_bit, 0x10, sizeof(unsigned int));
    unsigned int enable_global_interupts=3;
    PYNQ_writeMMIO(&(state->mmio_window), &enable_global_interupts, 0x1C, sizeof(unsigned int));
  }
}

/**
* Tests whether an interrupt has been raised at the AXI interrupt controller, and this is written into the
* flag argument, with the IRQ number being dictated by the controller and not the ZYNQ PS. Note that the
* interrupt must have been registered before it is tested for. Also in a single test a number of interrupts might
* be detected and if so they will be stored for future testing by other callers.
*/
int PYNQ_testForInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER * state, int irq_number, int * flag) {
  struct raised_interrupt_struct * interrupt = findRaisedInterruptStructure(state, irq_number);
  if (interrupt != NULL) {
    if (interrupt->number > 0) {
      interrupt->number--;
      *flag=1;
      return PYNQ_SUCCESS;
    }
  } else {
    fprintf(stderr, "You need to register an interrupt with IRQ %d before testing or waiting for it\n", irq_number);
    return PYNQ_ERROR;
  }

  pullRaisedIRQs(state);
  if (interrupt->number > 0) {
    interrupt->number--;
    *flag=1;
  } else {
    *flag=0;
  }
  return PYNQ_SUCCESS;
}

/**
* Waits for an interrupt to be raised at the AXI interrupt controller based on the IRQ number which is
* dictated by the controller and not the Zynq PS.
*/
int PYNQ_waitForInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER * state, int irq_number) {
  int flag=0;
  while (flag == 0) {
    PYNQ_testForInterrupt(state, irq_number, &flag);
  }
  return PYNQ_SUCCESS;
}

/**
* Closes the connection to the interrupt controller (although this will still function uneffected)
* and frees up associated memory.
*/
int PYNQ_closeInterruptController(PYNQ_AXI_INTERRUPT_CONTROLLER * state) {
  PYNQ_closeMMIOWindow(&(state->mmio_window));
  struct raised_interrupt_struct * head=state->raised_interrupt_head;
  while (head != NULL) {
    struct raised_interrupt_struct * p=head;
    head=head->next;
    free(p);
  }
  return PYNQ_SUCCESS;
}

/**
* Opens a Userspace I/O (UIO) interface based on the provided IRQ number which is the number as seen by the
* Zynq PS.
*/
int PYNQ_openUIO(PYNQ_UIO * state, int irq) {
  char * dev_name=get_uio_irq_devname(irq + 61);
  if (dev_name == NULL) return PYNQ_ERROR;
  state->filename=get_device_path(dev_name);
  free(dev_name);
  state->active=0;
  state->irq=irq;
  return PYNQ_SUCCESS;
}

int PYNQ_setupUIO(PYNQ_UIO * state) {
  if (!state->active) {
    if (enable_uio(state) == PYNQ_ERROR) return PYNQ_ERROR;
  }
  state->file_descriptor = open(state->filename, O_RDWR);
  if (state->file_descriptor == -1) {
    fprintf(stderr, "Error opening UIO file '%s'\n", state->filename);
    return PYNQ_ERROR;
  }

  // uint32_t info = 1; // unmask first interrupt
  // ssize_t nb = write(state->file_descriptor, &info, sizeof(info));
  // printf("Enabled interrupt\n");
  // if (nb != (ssize_t)sizeof(info)) {
  //   perror("ERROR: Write to UIO descriptor failed");
  //   close(state->file_descriptor);
  //   exit(EXIT_FAILURE);
  // }
}

/**
* Waits for the UIO to detect the raised interrupt and returns the interrupt count (i.e. the sequence number
* of the raised interrupt which can be useful to ensure the code has not missed interrupts)
*/
int PYNQ_waitForUIO(PYNQ_UIO * state, int * flag) {
  static int first_call = 1;

  if (first_call) {
    PYNQ_setupUIO(state);
    first_call = 0;
  }

  uint32_t info = 1; // unmask first interrupt
  ssize_t nb = write(state->file_descriptor, &info, sizeof(info));
  if (nb != (ssize_t)sizeof(info)) {
    perror("ERROR: Write to UIO descriptor failed");
    close(state->file_descriptor);
    exit(EXIT_FAILURE);
  }

  int data;
  int code=read(state->file_descriptor, &data, sizeof(int));
  if (code != sizeof(int)) {
    fprintf(stderr, "Expected to read 32 bit interrupt from '%s' but got %d bytes\n", state->filename, code);
    return PYNQ_ERROR;
  }
  //close(f);
  state->active=0;
  *flag=data;
  return PYNQ_SUCCESS;
}

/**
* Checks the UIO interface for whether an interrupt has been raised or not. If not then -1 is returned
* in the flag, otherwise the flag contains the interrupt count (i.e. the sequence number
* of the raised interrupt which can be useful to ensure the code has not missed interrupts)
*/
int PYNQ_checkForUIO(PYNQ_UIO * state, int * flag) {
  if (!state->active) {
    if (enable_uio(state) == PYNQ_ERROR) return PYNQ_ERROR;
  }
  int f = open(state->filename, O_RDONLY | O_NONBLOCK);
  if (f == -1) {
    fprintf(stderr, "Error opening UIO file '%s'\n", state->filename);
    return PYNQ_ERROR;
  }
  int data;
  int code=read(f, &data, sizeof(int));
  close(f);
  if (code == sizeof(int)) {
    state->active=0;
    *flag=data;
  } else if (code == -1) {
    *flag=-1;
  } else {
    fprintf(stderr, "Expected to read 32 bit interrupt from '%s' or none, but got %d bytes\n", state->filename, code);
    return PYNQ_ERROR;
  }
  return PYNQ_SUCCESS;
}

/**
* Closes the UIO interface, note that this is just the connection to that interface in this user code and
* does not effect the raising of interrupts on the IRQ by the PL or other codes listening for interrupts via
* that same UIO
*/
int PYNQ_closeUIO(PYNQ_UIO * state) {
  free(state->filename);
  close(state->file_descriptor);
  return PYNQ_SUCCESS;
}

/**
* This opens the DMA based on the AXI DMA IP block at the specified base address. Whilst this is not a core
* part of the Zynq eco-system it is a very popular IP block and-so abstracting it is useful. The block works by containing
* two separate channels, one for writing and one for reading. This call starts the channels.
*/
int PYNQ_openDMA(PYNQ_AXI_DMA * state, size_t base_address) {
  PYNQ_createMMIOWindow(&(state->mmio_window), base_address, 128);
  state->write_channel_offset=0x00;
  state->read_channel_offset=0x30;
  state->first_transfer[AXI_DMA_WRITE]=1;
  state->first_transfer[AXI_DMA_READ]=1;
  state->interrupt_mode=0;
  if (startDMAChannel(state, AXI_DMA_WRITE) == PYNQ_ERROR) return PYNQ_ERROR;
  if (startDMAChannel(state, AXI_DMA_READ) == PYNQ_ERROR) return PYNQ_ERROR;
  return PYNQ_SUCCESS;
}

/**
* Sets the interrupt mode of the DMA transfer, by default no interrupt will be raised when transfers complete but this can be
* changed by calling into this function. Note that this call will stop and restart the channels in the new mode, so you should
* not do this whilst a transfer is in progress
*/
int PYNQ_setDMATransferInterruptMode(PYNQ_AXI_DMA * state, int interrupt_mode) {
  if (interrupt_mode != state->interrupt_mode) {
    state->interrupt_mode=interrupt_mode;
    if (stopDMAChannel(state, AXI_DMA_WRITE) == PYNQ_ERROR) return PYNQ_ERROR;
    if (stopDMAChannel(state, AXI_DMA_READ) == PYNQ_ERROR) return PYNQ_ERROR;
    if (startDMAChannel(state, AXI_DMA_WRITE) == PYNQ_ERROR) return PYNQ_ERROR;
    if (startDMAChannel(state, AXI_DMA_READ) == PYNQ_ERROR) return PYNQ_ERROR;
  }
  return PYNQ_SUCCESS;
}

/**
* Starts a DMA transfer from some location in shared (DRAM) memory at the data_offset, in a specific direction (either writing
* or reading)
*/
int PYNQ_issueDMATransfer(PYNQ_AXI_DMA * state, PYNQ_SHARED_MEMORY * sm_state, size_t data_offset,
    size_t length, AXI_DMA_DIRECTION direction) {

  if (direction != AXI_DMA_WRITE && direction != AXI_DMA_READ) {
    fprintf(stderr, "You must supply either AXI_DMA_WRITE or AXI_DMA_READ as the transfer DMA direction\n");
    return PYNQ_ERROR;
  }
  return issueDMATransfer(state, sm_state, data_offset, length, direction);
}

/**
* Writes some data to the PL via the DMA engine. The data is located in the provided shared memory at the specified offset of
* length provided.
*/
int PYNQ_writeDMA(PYNQ_AXI_DMA * state, PYNQ_SHARED_MEMORY * sm_state, size_t offset, size_t length) {
  return issueDMATransfer(state, sm_state, offset, length, AXI_DMA_WRITE);
}

/**
* Reads some data from the PL via the DMA engine. The data is copied into the provided shared memory at the specified offset of
* length provided.
*/
int PYNQ_readDMA(PYNQ_AXI_DMA * state, PYNQ_SHARED_MEMORY * sm_state, size_t offset, size_t length) {
  return issueDMATransfer(state, sm_state, offset, length, AXI_DMA_READ);
}

/**
* Tests whether DMA has completed and writes this into the provided flag. Note that this follows the interrupt less approach, even
* if the user enabled interrupts
*/
int PYNQ_testForDMAComplete(PYNQ_AXI_DMA * state, AXI_DMA_DIRECTION direction, int * flag) {
  if (direction != AXI_DMA_WRITE && direction != AXI_DMA_READ) {
    fprintf(stderr, "You must supply either AXI_DMA_WRITE or AXI_DMA_READ as the transfer DMA direction\n");
    return PYNQ_ERROR;
  }
  if (!isDMAChannelRunning(state, AXI_DMA_WRITE)) {
    fprintf(stderr, "Error testing DMA completion status as engine is not running\n");
    return PYNQ_ERROR;
  }
  *flag=!isDMAChannelIdle(state, direction);
  return PYNQ_SUCCESS;
}

/**
* Waits fo DMA to complete in a specified direction
*/
int PYNQ_waitForDMAComplete(PYNQ_AXI_DMA * state, AXI_DMA_DIRECTION direction) {
  if (direction != AXI_DMA_WRITE && direction != AXI_DMA_READ) {
    fprintf(stderr, "You must supply either AXI_DMA_WRITE or AXI_DMA_READ as the transfer DMA direction\n");
    return PYNQ_ERROR;
  }
  if (!isDMAChannelRunning(state, AXI_DMA_WRITE)) {
    fprintf(stderr, "Error waiting for DMA completion as engine is not running");
    return PYNQ_ERROR;
  }
  while (!isDMAChannelIdle(state, direction));
  return PYNQ_SUCCESS;
}

/**
* Closes the DMA transfer connection and stops the channels
*/
int PYNQ_closeDMA(PYNQ_AXI_DMA * state) {
  if (stopDMAChannel(state, AXI_DMA_WRITE) == PYNQ_ERROR) return PYNQ_ERROR;
  if (stopDMAChannel(state, AXI_DMA_READ) == PYNQ_ERROR) return PYNQ_ERROR;
  return PYNQ_closeMMIOWindow(&(state->mmio_window));
}

/**
* Allocates a portion of DRAM to be shared memory that the PL can also interact with. Whilst the PL can see
* the DRAM memory, one must be careful as virtual memory can not be used as normal - instead the memory must be
* physically pinned. This calls into the cma library which maintains its own memory allocation state. Upon code
* termination the cma library does not clean up memory, therefore if there is not enough memory then the link can
* reset to free all memory and retry, although you might not want this and hence it is an option.
*
* Once allocated we assume the caller will then use the pointer and physical_address members of the PYNQ_SHARED_MEMORY
* type directly in their code
*/
int PYNQ_allocatedSharedMemory(PYNQ_SHARED_MEMORY * state, size_t length, int allow_reset) {
  unsigned int page_size=sysconf(_SC_PAGESIZE);
  if (length >= page_size*cma_pages_available()) {
    // If we haven't got enough shared memory then first reset the link which should free up any previously allocated but not freed memory
    if (allow_reset) _xlnk_reset();
    // Now recheck how much shared memory there is and error if we don't have enough available
    if (length >= page_size*cma_pages_available()) {
      fprintf(stderr, "Unable to allocate shared memory as %d bytes is requested but only %d bytes available\n", length, page_size*cma_pages_available());
      return PYNQ_ERROR;
    }
  }
  state->pointer=cma_alloc(length, 0);
  if (state->pointer == NULL) {
    fprintf(stderr, "Unable to allocate shared memory of %d bytes, allocator returned error\n", length);
    return PYNQ_ERROR;
  }
  state->physical_address=cma_get_phy_addr(state->pointer);
  return PYNQ_SUCCESS;
}

/**
* Frees the allocated shared memory
*/
int PYNQ_freeSharedMemory(PYNQ_SHARED_MEMORY * state) {
  cma_free(state->pointer);
  return PYNQ_SUCCESS;
}

/**
* Creates an MMIO window at a specific base address of a provided size
*/
int PYNQ_createMMIOWindow(PYNQ_MMIO_WINDOW * state, size_t address_base, size_t length) {
  // Align the base address with the pages
  state->virt_base = address_base & ~(sysconf(_SC_PAGESIZE) - 1);
  state->virt_offset = address_base - state->virt_base;
  state->length=length;
  state->address_base=address_base;

  state->file_handle=open(MEMORY_DEV_PATH, O_RDWR | O_SYNC);
  if (state->file_handle == -1) {
    fprintf(stderr, "Unable to open '%s' to create memory window", MEMORY_DEV_PATH);
    return PYNQ_ERROR;
  }
  state->buffer=mmap(NULL, length+state->virt_offset, PROT_READ | PROT_WRITE, MAP_SHARED, state->file_handle, state->virt_base);
  if (state->buffer == MAP_FAILED) {
    fprintf(stderr, "Mapping memory to MMIO region failed");
    return PYNQ_ERROR;
  }
  return PYNQ_SUCCESS;
}

/**
* Closes an MMIO window that we have previously created
*/
int PYNQ_closeMMIOWindow(PYNQ_MMIO_WINDOW * state) {
  close(state->file_handle);
  return PYNQ_SUCCESS;
}

/**
* Writes some data, of provided size to the specified offset in the memory window
*/
int PYNQ_writeMMIO(PYNQ_MMIO_WINDOW * state, void * data, size_t offset, size_t size_data) {
  memcpy(&(state->buffer[offset]), data, size_data);
  return PYNQ_SUCCESS;
}

/**
* Reads some data, of provided size to the specified offset from the memory window
*/
int PYNQ_readMMIO(PYNQ_MMIO_WINDOW * state, void * data, size_t offset, size_t size_data) {
  memcpy(data, &(state->buffer[offset]), size_data);
  return PYNQ_SUCCESS;
}

/**
* Opens a General Purpose I/O (GPIO) port at a specified index with direction provided
*/
int PYNQ_openGPIO(PYNQ_GPIO * state, int index, GPIO_DIRECTION direction) {
  state->index=index;
  state->direction=direction;

  char path[150], export_path[150];
  sprintf(path, "%s/gpio%d", GPIO_PATH, index);
  if (access(path, F_OK) == -1 ) {
    sprintf(export_path, "%s/export", GPIO_PATH);
    FILE * create_handle=fopen(export_path, "w");
    if (create_handle == NULL) {
      fprintf(stderr, "Can not open '%s' to create GPIO\n", export_path);
      return PYNQ_ERROR;
    }
    fprintf(create_handle, "%d", index);
    fclose(create_handle);
  }
  char f_name[150];
  sprintf(f_name,"%s/direction", path);
  FILE * handle=fopen(f_name, "w");
  if (handle == NULL) {
    fprintf(stderr, "Can not open '%s' to create GPIO\n", f_name);
    return PYNQ_ERROR;
  }
  if (direction == GPIO_OUT) {
    fprintf(handle, "out");
  } else if (direction == GPIO_IN) {
    fprintf(handle, "in");
  } else {
    fclose(handle);
    fprintf(stderr, "Must either supply GPIO_OUT or GPIO_IN as GPIO direction in open call\n");
    return PYNQ_ERROR;
  }
  fclose(handle);

  state->filename=(char*) malloc(strlen(path) + 7);
  sprintf(state->filename,"%s/value", path);
  return PYNQ_SUCCESS;
}

/**
* Closes an opened GPIO port
*/
int PYNQ_closeGPIO(PYNQ_GPIO * state) {
  char unexport_path[150];
  sprintf(unexport_path, "%s/unexport", GPIO_PATH);

  free(state->filename);
  FILE * handle=fopen(unexport_path, "w");
  if (handle == NULL) {
    fprintf(stderr, "Can not open '%s' to close GPIO\n", unexport_path);
    return PYNQ_ERROR;
  }
  fprintf(handle, "%d", state->index);
  fclose(handle);
  return PYNQ_SUCCESS;
}

/**
* Writes some data, val, to the GPIO that we have previously opened
*/
int PYNQ_writeGPIO(PYNQ_GPIO * state, int * val) {
  FILE * file_handle=fopen(state->filename, state->direction==GPIO_OUT ? "w" : "r");
  if (file_handle == NULL) {
    fprintf(stderr, "Can not open '%s' to write to GPIO\n", state->filename);
    return PYNQ_ERROR;
  }
  fprintf(file_handle, "%d", *val);
  fclose(file_handle);
  return PYNQ_SUCCESS;
}

/**
* Reads some data, val, from the GPIO that we have previously opened
*/
int PYNQ_readGPIO(PYNQ_GPIO * state, int * val) {
  FILE * file_handle=fopen(state->filename, state->direction==GPIO_OUT ? "w" : "r");
  if (file_handle == NULL) {
    fprintf(stderr, "Can not open '%s' to read from GPIO\n", state->filename);
    return PYNQ_ERROR;
  }
  *val=fgetc(file_handle);
  fclose(file_handle);
  return PYNQ_SUCCESS;
}

/**
* Loads the bitstream with name and location, bitstream_name, onto the PL and activates it. 
* Pass partial==0 for full bitstream configuration and partial==1 for partial bitstream.
*/
int PYNQ_loadBitstream(char * bitstream_name, int partial) {
  if (partial != 0 && partial != 1)
    fprintf(stderr, "Unrecognized loadBitstream flag\n");

  int bitstream_payload_length;
  char * bitstream_payload=extractBitstreamPayload(bitstream_name, &bitstream_payload_length);

  char * base_name=getBaseNameFromFile(bitstream_name);
  char * binfile_name=getBinNameFromBit(base_name);
  char firmware_path[strlen(binfile_name) + strlen(FIRMWARE_PATH_PREFIX)+1];
  sprintf(firmware_path, "%s%s", FIRMWARE_PATH_PREFIX, binfile_name);
  FILE * f =fopen(firmware_path, "wb");
  if (f == NULL) {
    fprintf(stderr, "Can not open firmware file '%s'\n", firmware_path);
    free(bitstream_payload);
    free(binfile_name);
    return PYNQ_ERROR;
  }
  fwrite(bitstream_payload, sizeof(char), bitstream_payload_length, f);
  fclose(f);
  free(bitstream_payload);

  FILE *fptr = fopen(BS_FPGA_MAN_FLAGS, "w");
  if (fptr == NULL) {
    fprintf(stderr, "Can not open man flags file '%s'\n", BS_FPGA_MAN_FLAGS);
    free(binfile_name);
    return PYNQ_ERROR;
  }
  fprintf(fptr, "%d", partial);
  fclose(fptr);

  fptr = fopen(BS_FPGA_MAN, "w");
  if (fptr == NULL) {
    fprintf(stderr, "Can not open man file '%s'\n", BS_FPGA_MAN);
    return PYNQ_ERROR;
  }
  fprintf(fptr, "%s", binfile_name);
  fclose(fptr);
  free(binfile_name);
  return PYNQ_SUCCESS;
}

/**
* Extracs information from the bit stream, bitstream_name, into the header type. This contains both the header information
* and payload data.
*/
int PYNQ_extractBitstreamInfo(PYNQ_BITSTREAM_INFO * bitstream_header, char * bitstream_name) {
  extractBitstreamInfo(bitstream_name, bitstream_header);
  return PYNQ_SUCCESS;
}

/**
* Frees the memory associated with bitstream information type that was previously used to extract the information from
* the bit stream into.
*/
int PYNQ_freeBitstreamInfo(PYNQ_BITSTREAM_INFO * header) {
  free(header->data);
  free(header->design);
  free(header->date);
  free(header->part);
  free(header->time);
  free(header->version);
  return PYNQ_SUCCESS;
}

/**
* Checks that the PS/PL is a 7-Series Zynq, or else the register info for the clock is entirely wrong. To support
* other technologies, such as the ultrascale it is simply a case of modifying the defines and clock functions as appropriate
*/
static int checkMachineType() {
  struct utsname uname_info;
  if (uname(&uname_info) == -1) {
    fprintf(stderr, "Can not gather uname information to ensure that the PS/PL is a 7-Series Zynq\n");
    return PYNQ_ERROR;
  }
  if (strcmp(ZYNQ_SERIES_7_IDENTIFIER, uname_info.machine) != 0) {
    fprintf(stderr, "For clock work the PS/PL must be a 7-Series Zynq, whereas uname returned '%s'\n", uname_info.machine);
    return PYNQ_ERROR;
  }
  return PYNQ_SUCCESS;
}

/**
* Retreives the source clock MHZ frequency (without any divisors) from the appropriate registers
*/
static int getSrcClkMHZ(PYNQ_REGISTER* reg, size_t offset, float * src_clk_mhz) {
  unsigned int src_clk_idx;
  if (!PYNQ_readRegisterRange(reg, offset, 4, 5, &src_clk_idx)) return PYNQ_ERROR;
  unsigned int pll_reg=src_clk_idx < 2 ? 0x108 : src_clk_idx == 2 ? 0x100 : 0x104;
  unsigned int pll_fbdiv;
  if (!PYNQ_readRegisterRange(reg, pll_reg, 12, 18, &pll_fbdiv)) return PYNQ_ERROR;
  *src_clk_mhz=(DEFAULT_SRC_CLK_MHZ * pll_fbdiv);
  return PYNQ_SUCCESS;
}

/**
* Will enable the UIO interface, ready to then capture interrupts raised on this
*/
static int enable_uio(PYNQ_UIO * state) {
  if (!state->active) {
    state->active=1;
    FILE * f = fopen(state->filename, "r+b");
    if (f == NULL) {
      fprintf(stderr, "Error opening UIO file '%s'\n", state->filename);
      return PYNQ_ERROR;
    }
    char data[4]={0x00, 0x00, 0x00, 0x01};
    size_t num_written=fwrite(&data, sizeof(char), 4, f);
    if (num_written != 4) {
      fprintf(stderr, "Expected to write 32 bit value to enable interrupt at '%s', but only wrote %d bytes\n", state->filename, num_written);
      return PYNQ_ERROR;
    }
    fclose(f);
  }
  return PYNQ_SUCCESS;
}

/**
* Will pull down raised IRQs from the AXI interrupt controller, insert these into the state
* and then acknowledge these on the AXI interrupt controller
*/
static void pullRaisedIRQs(PYNQ_AXI_INTERRUPT_CONTROLLER * state) {
  unsigned int raised_irqs, work, current_irq=0;
  // Read the interrupts
  PYNQ_readMMIO(&(state->mmio_window), &raised_irqs, 0x04, sizeof(unsigned int));
  work=raised_irqs;

  while (work != 0) {
    if (work % 2 == 1) {
      unsigned int disable_irq=1 << current_irq;
      PYNQ_writeMMIO(&(state->mmio_window), &disable_irq, 0x14, sizeof(unsigned int));
      appendOrInsertRaisedInterrupt(state, current_irq);
    }
    work = work >> 1;
    current_irq++;
  }
  // Acknowledge the interrupts
  PYNQ_writeMMIO(&(state->mmio_window), &raised_irqs, 0x0C, sizeof(unsigned int));
}

/**
* Will append the raising of an interrupt based on its IRQ to the state (by incrementing the counter) or if no such
* state exists for this specific IRQ then that will be allocated and is stored single linked list fashion
*/
static void appendOrInsertRaisedInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER * state, int irq) {
  struct raised_interrupt_struct * interrupt = findRaisedInterruptStructure(state, irq);
  if (interrupt == NULL) {
    interrupt=(struct raised_interrupt_struct *) malloc(sizeof(struct raised_interrupt_struct));
    interrupt->irq=irq;
    interrupt->number=0;
    interrupt->next=state->raised_interrupt_head;
    state->raised_interrupt_head=interrupt;
  }
  interrupt->active_waiting=0;
  interrupt->number++;
}

/**
* Retrieves the interrupt information associated with a single interrupt by its IRQ for this controller, or NULL
* if no information currently exists
*/
static struct raised_interrupt_struct * findRaisedInterruptStructure(PYNQ_AXI_INTERRUPT_CONTROLLER * state, int irq) {
  struct raised_interrupt_struct * head=state->raised_interrupt_head;
  while (head != NULL) {
    if (head->irq == irq) return head;
    head=head->next;
  }
  return NULL;
}

/**
* Given the device name will search the proc filesystem to find the appropriate UIO path that corresponds to this,
* which can then be used to directly read for UIO work
*/
static char * get_device_path(char * dev_name) {
  char full_name[512], line[512];
  DIR *dp;
  struct dirent *ep;
  dp=opendir(UIO_DESCRIPTION_PATH);
  if (dp != NULL) {
    while (ep = readdir(dp)) {
      if (ep->d_type == DT_DIR || ep->d_type == DT_LNK) {
        if (strcmp(ep->d_name, ".") == 0 || strcmp(ep->d_name, "..") == 0) continue;
        sprintf(full_name, "%s/%s/name", UIO_DESCRIPTION_PATH, ep->d_name);
        FILE * f = fopen(full_name, "r");
        if (f != NULL) {
          if (fgets(line, sizeof(line), f)) {
            char * new_line=strrchr(line,'\n');
            if (new_line != NULL) new_line[0]='\0';
            if (strcmp(line, dev_name) == 0) {
              fclose(f);
              char * device_path=(char*) malloc(strlen(ep->d_name)+6);
              sprintf(device_path, "%s/%s", UIO_PATH, ep->d_name);
              return device_path;
            }
          }
          fclose(f);
        }
      }
    }
  }
  fprintf(stderr, "Can not find UIO location with corresponding name of '%s'", dev_name);
  return NULL;
}

/**
* Given an IRQ number (at the Zynq PS) will look through the interrupts file in proc to locate the
* corresponding dev name associated with this IRQ. Null is returned if no IRQ is located
*/
static char * get_uio_irq_devname(int irq) {
  char line[256], irq_str[20];
  sprintf(irq_str, "%d", irq);
  FILE * f = fopen(INTERRUPTS_FILE, "r");
  if (f == NULL) {
    fprintf(stderr, "Can not open '%s'\n", INTERRUPTS_FILE);
    return NULL;
  }

  while (fgets(line, sizeof(line), f)) {
    char * tokenised = strtok(line, " ");
    int c=0;
    char back_strings[256][3];
    while (tokenised != NULL) {
      if (c > 2) strcpy(back_strings[0], back_strings[1]);
      if (c > 1) strcpy(back_strings[1], back_strings[2]);
      strcpy(back_strings[2], tokenised);

      tokenised = strtok(NULL, " ");
      c+=1;
    }
    if (c > 3 && strcmp(irq_str, back_strings[0]) == 0) {
      char * new_line=strrchr(back_strings[2],'\n');
      if (new_line != NULL) new_line[0]='\0';
      char * dev_name=(char*) malloc(strlen(back_strings[2]) + 1);
      strcpy(dev_name, back_strings[2]);
      fclose(f);
      return dev_name;
    }
  }
  fclose(f);
  fprintf(stderr, "Can not locate IRQ %d in interrupts file '%s'\n", irq, INTERRUPTS_FILE);
  return NULL;
}

/**
* Issues a DMA transfer on the AXI DMA block to or from the shared memory at a specific offset with length and the
* direction (write or read). Will error if the engine is not running or another transfer is in progress (but there are
* two channels, one for write and one for read.)
*/
static int issueDMATransfer(PYNQ_AXI_DMA * state, PYNQ_SHARED_MEMORY * sm_state, size_t data_offset,
    size_t length, AXI_DMA_DIRECTION direction) {
  int mmio_offset = direction == AXI_DMA_WRITE ? state->write_channel_offset : state->read_channel_offset;

  unsigned int physical_address=((unsigned int) sm_state->physical_address) + (unsigned int) data_offset;
  unsigned int xfer_length=(unsigned int) length;

  if (!isDMAChannelRunning(state, direction)) {
    fprintf(stderr, "Error writing to DMA as engine is not running\n");
    return PYNQ_ERROR;
  }

  if (!state->first_transfer[direction] && !isDMAChannelIdle(state, direction)) {
    fprintf(stderr, "Error writing to DMA as another transfer is in progress\n");
    return PYNQ_ERROR;
  }

  PYNQ_writeMMIO(&(state->mmio_window), &physical_address, mmio_offset+0x18, sizeof(unsigned int));
  PYNQ_writeMMIO(&(state->mmio_window), &xfer_length, mmio_offset+0x28, sizeof(unsigned int));
  state->first_transfer[direction]=0;
  return PYNQ_SUCCESS;
}

/**
* Starts a DMA channel in the write or read direction, either with or without interrupt raising enabled
* as configured in the state
*/
static int startDMAChannel(PYNQ_AXI_DMA * state, AXI_DMA_DIRECTION direction) {
  int offset = direction == AXI_DMA_WRITE ? state->write_channel_offset : state->read_channel_offset;
  unsigned int command= state->interrupt_mode == 0 ? 0x0001 : 0x1001;
  return PYNQ_writeMMIO(&(state->mmio_window), &command, offset, sizeof(unsigned int));
}

/**
* Stops a DMA channel in a specific direction
*/
static int stopDMAChannel(PYNQ_AXI_DMA * state, AXI_DMA_DIRECTION direction) {
  int offset = direction == AXI_DMA_WRITE ? state->write_channel_offset : state->read_channel_offset;
  unsigned int command= 0x0000;
  return PYNQ_writeMMIO(&(state->mmio_window), &command, offset, sizeof(unsigned int));
}

/**
* Returns whether a DMA channel is running (i.e. started, but it can be running and idle) or not
*/
static int isDMAChannelRunning(PYNQ_AXI_DMA * state, AXI_DMA_DIRECTION direction) {
  int offset = direction == AXI_DMA_WRITE ? state->write_channel_offset : state->read_channel_offset;
  unsigned int read_byte;
  PYNQ_readMMIO(&(state->mmio_window), &read_byte, offset+4, sizeof(unsigned int));
  return (read_byte & 0x01) == 0x00;
}

/**
* Returns whether a DMA channel is running and idle (i.e. no DMA transfer is currently in progress.)
*/
static int isDMAChannelIdle(PYNQ_AXI_DMA * state, AXI_DMA_DIRECTION direction) {
  int offset = direction == AXI_DMA_WRITE ? state->write_channel_offset : state->read_channel_offset;
  unsigned int read_byte;
  PYNQ_readMMIO(&(state->mmio_window), &read_byte, offset+4, sizeof(unsigned int));
  return (read_byte & 0x02) == 0x02;
}

/**
* Given a full path filename this will return the filename alone, with the proceeding path stripped off
*/
static char * getBaseNameFromFile(char * filename) {
  char * slash_loc=strrchr(filename, '/');
  if (slash_loc == NULL) return filename;
  return slash_loc+1;
}

/**
* Given the filename of a bit stream will return the corresponding binary filename (this is quite simple,
* but required for launching the bitstream payload on the PL.)
*/
static char * getBinNameFromBit(char * filename) {
  char * dot_point=strrchr(filename, '.');
  char * bin_name=(char*) malloc((dot_point-filename)+5);
  memcpy(bin_name, filename, dot_point-filename);
  bin_name[dot_point-filename]='\0';
  sprintf(bin_name, "%s.bin", bin_name);
  return bin_name;
}

/**
* Given the filename and path of a bitstream this will extract the payload data (returned) and the length in
* bytes (returned via the data_len argument). It completely ignores the header information and skips passed it.
* Note that the bitstream is in big endian whereas the Zynq works in little endian, hence we need to do byte swaps
* on the bitstream contents, both to find the payload part and also on the payload data itself (in 32 bit chunks)
*/
static char * extractBitstreamPayload(char * filename, int * data_len) {
  char * data_buffer=NULL;
  char * rawBitData=readRawBitfile(filename);
  short length;
  int payload_length;
  memcpy(&length, rawBitData, sizeof(short));
  length=__bswap_16(length);
  // Strip the (2+n)-byte first field (2-bit length, n-bit data) and a two-byte unknown field (usually 1)
  size_t offset=4+length;
  while (1==1) {
    char desc=rawBitData[offset];
    offset++;
    if (desc == 0x65) {
      memcpy(&(payload_length), &rawBitData[offset], sizeof(int));
      payload_length=__bswap_32(payload_length);
      *data_len=payload_length;
      offset+=4;
      data_buffer=(char*) malloc(payload_length);
      memcpy(data_buffer, &rawBitData[offset], payload_length);
      // Now iterate through and swap from big endian to little endian
      for (int i=0;i<payload_length/4;i++) {
        int data_item;
        memcpy(&data_item, &(data_buffer[i*4]), sizeof(int));
        data_item=__bswap_32(data_item);
        memcpy(&(data_buffer[i*4]), &data_item, sizeof(int));
      }
      if (payload_length%4 != 0) {
        // Handle last element if not divisible by four
        int remainder_size=payload_length - ((payload_length/4) * 4);
        int data_item=0;
        memcpy(&data_item, &(data_buffer[(payload_length/4) * 4]), remainder_size);
        data_item=__bswap_32(data_item);
        memcpy(&(data_buffer[(payload_length/4) * 4]), &data_item, sizeof(int));
      }
      break;
    } else {
      memcpy(&length, &rawBitData[offset], sizeof(short));
      length=__bswap_16(length);
      offset+=2+length;
    }
  }
  return data_buffer;
}

/**
* Given the filename and path of a bitstream this will extract the full header and payload data and return this
* via the header argument.
* Note that the bitstream is in big endian whereas the Zynq works in little endian, hence we need to do byte swaps
* on the bitstream contents, both to find the payload part and also on the payload data itself (in 32 bit chunks)
*/
static void extractBitstreamInfo(char * filename, PYNQ_BITSTREAM_INFO * header) {
  char * data_buffer=NULL;
  char * rawBitData=readRawBitfile(filename);
  short length;
  memcpy(&length, rawBitData, sizeof(short));
  length=__bswap_16(length);
  // Strip the (2+n)-byte first field (2-bit length, n-bit data) and a two-byte unknown field (usually 1)
  size_t offset=4+length;
  while (1==1) {
    char desc=rawBitData[offset];
    offset++;
    if (desc != 0x65) {
      memcpy(&length, &rawBitData[offset], sizeof(short));
      length=__bswap_16(length);
      offset+=2;
      data_buffer=(char*) malloc(length+1);
      memcpy(data_buffer, &rawBitData[offset], length);
      offset+=length;

      if (desc == 0x61) {
        data_buffer[length]='\0';
        char * semi_point=strrchr(data_buffer, ';');
        size_t str_len=semi_point-data_buffer;
        header->design=(char*) malloc(str_len+1);
        memcpy(header->design, data_buffer, str_len);
        header->design[str_len]='\0';
        str_len=strlen(data_buffer)-str_len;
        header->version=(char*) malloc(str_len+1);
        memcpy(header->version, &semi_point[1], str_len);
        header->version[str_len]='\0';
      } else if (desc == 0x62) {
        header->part=(char*) malloc(length+1);
        memcpy(header->part, data_buffer, length);
        header->part[length]='\0';
      } else if (desc == 0x63) {
        header->date=(char*) malloc(length+1);
        memcpy(header->date, data_buffer, length);
        header->date[length]='\0';
      } else if (desc == 0x64) {
        header->time=(char*) malloc(length+1);
        memcpy(header->time, data_buffer, length);
        header->time[length]='\0';
      }
      free(data_buffer);
    }
    if (desc == 0x65) {
      memcpy(&(header->data_length), &rawBitData[offset], sizeof(int));
      header->data_length=__bswap_32(header->data_length);
      offset+=4;
      header->data=(char*) malloc(header->data_length);
      memcpy(header->data, &rawBitData[offset], header->data_length);
      // Now iterate through and swap from big endian to little endian
      for (int i=0;i<header->data_length/4;i++) {
        int data_item;
        memcpy(&data_item, &(header->data[i*4]), sizeof(int));
        data_item=__bswap_32(data_item);
        memcpy(&(header->data[i*4]), &data_item, sizeof(int));
      }
      if (header->data_length%4 != 0) {
        // Handle last element if not divisible by four
        int remainder_size=header->data_length - ((header->data_length/4) * 4);
        int data_item=0;
        memcpy(&data_item, &(header->data[(header->data_length/4) * 4]), remainder_size);
        data_item=__bswap_32(data_item);
        memcpy(&(header->data[(header->data_length/4) * 4]), &data_item, sizeof(int));
      }
      break;
    }
  }
  free(rawBitData);
}

/**
* Given the filename and path of the bitstream file this will read the byte contents of that file and return it.
* We don't know how large the file contents are, so it reads the information in chunks and then resizes a
* buffer to store this into piece by piece.
*/
static char * readRawBitfile(char * filename) {
  FILE * f = fopen(filename, "rb");

  char * buffer = (char *) malloc(READ_CHUNK_SIZE);
  char * data_byte_buffer = NULL;
  size_t data_byte_buffer_size=0;

  size_t read_bytes=READ_CHUNK_SIZE;
  while (read_bytes == READ_CHUNK_SIZE) {
    read_bytes=fread(buffer, sizeof(char), READ_CHUNK_SIZE, f);
    data_byte_buffer=realloc(data_byte_buffer, data_byte_buffer_size+read_bytes);
    memcpy(&data_byte_buffer[data_byte_buffer_size], buffer, read_bytes);
    data_byte_buffer_size+=read_bytes;
  }
  fclose(f);
  free(buffer);
  return data_byte_buffer;
}
