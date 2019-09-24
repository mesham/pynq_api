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

#ifndef PYNQ_API_H_INCLUDED
#define PYNQ_API_H_INCLUDED

#include <stdlib.h>

#define PYNQ_SUCCESS 1
#define PYNQ_ERROR 0

typedef struct mmio_state_struct {
  char * buffer;
  int file_handle;
  unsigned int length, address_base, virt_base, virt_offset;
} PYNQ_MMIO_WINDOW;

typedef enum gpio_direction_emum { GPIO_OUT=0, GPIO_IN=1 } GPIO_DIRECTION;

typedef struct gpio_state_struct {
  int index;
  GPIO_DIRECTION direction;
  char * filename;
} PYNQ_GPIO;

typedef struct bit_file_header_struct {
  char *design, *version, *part, *date, *time;
  int data_length;
  char * data;
} PYNQ_BITSTREAM_INFO;

typedef struct shared_memory_state_struct {
  void * pointer;
  unsigned long physical_address;
} PYNQ_SHARED_MEMORY;

typedef enum axi_dma_direction_enum { AXI_DMA_WRITE=0, AXI_DMA_READ=1 } AXI_DMA_DIRECTION;

typedef struct axi_dma_transfer_struct {
  PYNQ_MMIO_WINDOW mmio_window;
  unsigned int write_channel_offset, read_channel_offset;
  unsigned char first_transfer[2];
  int interrupt_mode;
} PYNQ_AXI_DMA;

typedef struct uio_struct {
  char * filename;
  unsigned char active;
  int irq;
} PYNQ_UIO;

struct raised_interrupt_struct {
  int irq, number;
  unsigned char active_waiting;
  struct raised_interrupt_struct * next;
};

typedef struct interrupt_controller_struct {
  PYNQ_MMIO_WINDOW mmio_window;
  struct raised_interrupt_struct * raised_interrupt_head;
  unsigned int enabled_hw_interrupts;
} PYNQ_AXI_INTERRUPT_CONTROLLER;

typedef struct register_struct {
  PYNQ_MMIO_WINDOW mmio_window;
} PYNQ_REGISTER;

typedef struct hls_struct {
  PYNQ_MMIO_WINDOW mmio_window;
} PYNQ_HLS;

// Core API
unsigned long long PYNQ_Wtime();
int PYNQ_getCPUClockFreq(float* frequency);
int PYNQ_getPLClockFreq(unsigned int clock_index, float* frequency);
int PYNQ_setPLClockFreq(unsigned int clock_index, float * freq, int * div0, int *div1);
int PYNQ_openRegister(PYNQ_REGISTER* reg_state, size_t address, size_t width);
int PYNQ_readRegister(PYNQ_REGISTER* reg_state, size_t address, int index, unsigned int* data);
int PYNQ_readRegisterRange(PYNQ_REGISTER* reg_state, size_t address, int start_index, int end_index, unsigned int* data);
int PYNQ_setRegister(PYNQ_REGISTER* reg_state, size_t address, int index, unsigned int* data);
int PYNQ_setRegisterRange(PYNQ_REGISTER* reg_state, size_t address, int start_index, int end_index, unsigned int* data);
int PYNQ_closeRegister(PYNQ_REGISTER* reg_state);
int PYNQ_openUIO(PYNQ_UIO* uio_state, int irq);
int PYNQ_closeUIO(PYNQ_UIO* uio_state);
int PYNQ_waitForUIO(PYNQ_UIO* uio_state, int* flag);
int PYNQ_checkForUIO(PYNQ_UIO* uio_state, int* flag);
int PYNQ_allocatedSharedMemory(PYNQ_SHARED_MEMORY* sm_state, size_t length, int allow_reset);
int PYNQ_freeSharedMemory(PYNQ_SHARED_MEMORY* sm_state);
int PYNQ_createMMIOWindow(PYNQ_MMIO_WINDOW* mmio_state, size_t address, size_t length);
int PYNQ_closeMMIOWindow(PYNQ_MMIO_WINDOW* mmio_state);
int PYNQ_writeMMIO(PYNQ_MMIO_WINDOW* mmio_state, void* data, size_t offset, size_t length);
int PYNQ_readMMIO(PYNQ_MMIO_WINDOW* mmio_state, void* data, size_t offset, size_t length);
int PYNQ_openGPIO(PYNQ_GPIO* gpio_state, int index, GPIO_DIRECTION direction);
int PYNQ_closeGPIO(PYNQ_GPIO* gpio_state);
int PYNQ_writeGPIO(PYNQ_GPIO* gpio_state, int* data);
int PYNQ_readGPIO(PYNQ_GPIO* gpio_state, int* data);
int PYNQ_loadBitstream(char* filename);
int PYNQ_extractBitstreamInfo(PYNQ_BITSTREAM_INFO* info, char* filename);
int PYNQ_freeBitstreamInfo(PYNQ_BITSTREAM_INFO* info);

// HLS API
int PYNQ_openHLS(PYNQ_HLS* hls_state, size_t address, size_t width);
int PYNQ_startHLS(PYNQ_HLS* hls_state);
int PYNQ_testHLSCompleted(PYNQ_HLS* hls_state, int* flag);
int PYNQ_waitForHLS(PYNQ_HLS* hls_state);
int PYNQ_writeToHLS(PYNQ_HLS* hls_state, void* data, size_t offset, size_t length);
int PYNQ_readFromHLS(PYNQ_HLS* hls_state, void* data, size_t offset, size_t length);
int PYNQ_closeHLS(PYNQ_HLS* hls_state);

// AXI DMA API
int PYNQ_openDMA(PYNQ_AXI_DMA* dma_state, size_t base_address);
int PYNQ_setDMATransferInterruptMode(PYNQ_AXI_DMA* dma_state, int interrupt_mode);
int PYNQ_issueDMATransfer(PYNQ_AXI_DMA* dma_state, PYNQ_SHARED_MEMORY* shared_memory, size_t offset, size_t length, AXI_DMA_DIRECTION direction);
int PYNQ_writeDMA(PYNQ_AXI_DMA* dma_state, PYNQ_SHARED_MEMORY* shared_memory, size_t offset, size_t length);
int PYNQ_readDMA(PYNQ_AXI_DMA* dma_state, PYNQ_SHARED_MEMORY* shared_memory, size_t offset, size_t length);
int PYNQ_testForDMAComplete(PYNQ_AXI_DMA* dma_state, AXI_DMA_DIRECTION direction, int* flag);
int PYNQ_waitForDMAComplete(PYNQ_AXI_DMA* dma_state, AXI_DMA_DIRECTION direction);
int PYNQ_closeDMA(PYNQ_AXI_DMA* dma_state);

// AXI Interrupt controller
int PYNQ_openInterruptController(PYNQ_AXI_INTERRUPT_CONTROLLER* interrupt_ctrl_state, size_t base_address);
int PYNQ_registerInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER* interrupt_ctrl_state, int irq, int generate_hw_interrupt);
int PYNQ_testForInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER* interrupt_ctrl_state, int irq, int* flag);
int PYNQ_waitForInterrupt(PYNQ_AXI_INTERRUPT_CONTROLLER* interrupt_ctrl_state, int irq);
int PYNQ_closeInterruptController(PYNQ_AXI_INTERRUPT_CONTROLLER* interrupt_ctrl_state);

#endif // PYNQ_API_H_INCLUDED
