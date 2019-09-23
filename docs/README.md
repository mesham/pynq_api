# PYNQ API Documentation

In this documents folder we have a number of separate file which discuss different aspects of the API.

* <a href="https://github.com/mesham/pynq_api/blob/master/docs/core.md">Core API</a> which describes the core API interacting with the Zynq IP block
* <a href="https://github.com/mesham/pynq_api/blob/master/docs/ps.md">Programmable System API</a> which describes the management of the PS and PS/PL interface
* <a href="https://github.com/mesham/pynq_api/blob/master/docs/extended.md">Extended API</a> which describes the extended PYNQ API that interacts with common IP block

## Miscellaneous

### Error codes

All PYNQ API functions return an integer error code which should be tested in user code. The _PYNQ_SUCCESS_ preprocessor define indicates that the call was successful, whereas _PYNQ_ERROR_ indicates that an error occured whilst executing the call. In the case of errors the API will also often print an associated error message to stderr providing further explanation.

### Timing

The `unsigned long long PYNQ_Wtime()` API call will return the time since the Epoch in microseconds and is useful for timing.

## Terminology

Throughout this documentation we use a number of acronyms, the common ones are described below:

Acronym | Description
--------- | ----------- 
PS | Programming System, the host ARM cores running Linux
PL | Programmable Logic, the FPGA logic configured by the bitstream
MMIO | Memory Mapped Input Output, memory connection to the IP blocks as configured by Vivado address editor
GPIO | General Purpose Input Output, physical pins on the Pynq board
UIO | User-space Input Output, used to track the raising of interrupts from the PL to the PS
AXI | Point to point interconnect protocol designed for high performance
HLS | High Level Synthesis, used to program IP blocks in C, C++ or System C
DMA | Direct Memory Access, where the PL directly accesses host DRAM rather than through the PS
