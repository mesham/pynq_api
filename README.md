# PYNQ API: C API for PYNQ FPGA board

This project is a C based API for interacting with the 7-Series Zynq on the Pynq board, providing drivers that can be used to easily work with IP blocks running on the PL. In addition to the core parts of the API, there is also support provided for abstracting the interaction with some of the common AXI based IP blocks.

The idea of this work is to provide an alternative to the Python drivers that ship with the Pynq board, so that people can experiment with intergrating their existing codes onto the FPGA and interacting with these. The design of this API is very simple and as such should be trivial to wrap in bindings for other languages such as Fortran and Java.

See the <a href="https://github.com/mesham/pynq_api/tree/master/docs">documentation</a> for a detailed description of the API calls and functionality.

## Installation

Clone or download the repository onto your Pynq board and then issue ``make``. Once the code has build then issues ``sudo make install`` to install the library and header file to a common location on the board

## Usage

You should include the ``pynq_api.h`` header file in your codes, in order to link the code you will need to link against the ``pynq``, ``cma`` and ``pthread`` library. I.e. ``gcc -o my_code my_code.c -lpynq -lcma -lpthread``

## Example usage

This simple example assumes you have created an AXI DMA memory IP block, set it at address ``0x40400000`` and the streaming interfaces of this IP block are connected to some data storage such as an AXI streaming FIFO queue.

```c
#include <stdio.h>
#include <pynq_api.h>

int main() {
  PYNQ_loadBitstream("/home/xilinx/my_bitstream.bit");
  
  PYNQ_SHARED_MEMORY shared_memory_1, shared_memory_2;
  PYNQ_allocatedSharedMemory(&shared_memory_1, sizeof(int)*10, 1);
  PYNQ_allocatedSharedMemory(&shared_memory_2, sizeof(int)*10, 1);
  
  int * d1=(int*)shared_memory_1.pointer;
  int * d2=(int*)shared_memory_2.pointer;
  for (int i=0;i<10;i++) {
    d1[i]=i;
    d2[i]=0;
  }
  
  PYNQ_AXI_DMA dma;
  PYNQ_openDMA(&dma, 0x40400000);
  
  PYNQ_writeDMA(&dma, &shared_memory_1, 0, sizeof(int) * 10);
  PYNQ_readDMA(&dma, &shared_memory_2, 0, sizeof(int) * 10);

  PYNQ_waitForDMAComplete(&dma, AXI_DMA_WRITE);
  PYNQ_waitForDMAComplete(&dma, AXI_DMA_READ);

  for (int i=0;i<10;i++) {
    printf("%d %d %d\n", i, d1[i], d2[i]);
  }

  PYNQ_closeDMA(&dma);
  PYNQ_freeSharedMemory(&shared_memory_1);
  PYNQ_freeSharedMemory(&shared_memory_2);
  return 0;
}

```
