# Extended API
This documentation describes parts of the PYNQ API that provide extended functionality. This is functionality not specific or core to the Zynq or Pynq, but instead covers interaction with very common IP blocks one often uses in their block designs. As such a layer of abstraction over these is thought to be generally useful.

* [AXI Direct Memory Access (DMA)](#axi-direct-memory-access-dma)

## AXI Direct Memory Access (DMA)

The AXI DMA block provides the ability to perform high performance burst transfers between PS DRAM and the PL. In-fact the DMA IP block provides two channels, a read and write channel, along with a control bus (via AXI-Lite.) However the specifics of this can be ignored by the user when using the PYNQ API, apart from understanding that only one transfer per channel can take place at any point in time, but as the read and write channels are separate then transfers on these can occur concurrently. Note that DMA can only interact with data held in shared memory portion of DRAM, as per the <a href="https://github.com/mesham/pynq_api/blob/master/docs/core.md#shared-memory">shared memory</a> functionality in the core API.

![DMA illustration](https://pynq.readthedocs.io/en/v2.3/_images/dma.png)

Code containing calls to the DMA functionality must be run as sudo.

### Opening a DMA block

`int PYNQ_openDMA(PYNQ_AXI_DMA* dma_state, size_t base_address)`

This API call will open the DMA channel whose _base_address_ corresponds to the control bus address configured in the Vivado address editor. The user should define a variabe of type _PYNQ_AXI_DMA_ and pass a pointer to this to the open function, which is then used as the context for interaction with the the DMA IP block. This call will start both read and write DMA engines in non-interrupt mode (i.e. they will not generate an interrupt when transfers complete). An integer status code is returned indicating success or failure. 

### Setting the DMA interrupt mode

`int PYNQ_setDMATransferInterruptMode(PYNQ_AXI_DMA* dma_state, int interrupt_mode)`

This call will set the interrupt mode for both the read and write DMA engines of the _dma_state_ block, raising an interrupt when transfers complete. The _interrupt_mode_ flag denoted whether the mode should be used (integer value greater than zero) or not (zero value.) If the mode changes from its current setting then the DMA engines will be reset, this should not be done whilst a transfer is in progress. An integer status code is returned indicating success or failure. 

### Writing to DMA

`int PYNQ_writeDMA(PYNQ_AXI_DMA* dma_state, PYNQ_SHARED_MEMORY* shared_memory, size_t offset, size_t length)`

This call will write _length_ bytes of data from the _shared_memory_ location in DRAM, starting at _offset_ which is relative to the start of this block of shared memory, to the DMA block of _dma_state_. This is non-blocking, i.e. will return before the data transfer has completed, and issuing a write if there is already a write DMA transfer active will result in an error. An integer status code is returned indicating success or failure. 

The example here will allocate some shared memory and then fill this up with integers ranging from 0 to 49. The DMA block whose control port is at memory location _0x4D00000_ is then opened and a write issued to the engine to copy the 50 integers from host DDR starting at location _0x0_ of the _shared_memory_ block to the PL. We then wait for the DMA to complete and then close the engine and free the shared memory.

```c
PYNQ_SHARED_MEMORY shared_memory;
PYNQ_allocatedSharedMemory(&shared_memory, 2048, 1);
int * data=(int*) shared_memory.pointer;
for (int i=0;i<50;i++) data[i]=i;

PYNQ_AXI_DMA dma;
PYNQ_openDMA(&dma, 0x4D00000);
PYNQ_writeDMA(&dma, &shared_memory, 0x0, sizeof(int)*50);
PYNQ_waitForDMAComplete(&shared_memory, AXI_DMA_WRITE);
PYNQ_closeDMA(&dma);

PYNQ_freeSharedMemory(&shared_memory);
```
### Reading from DMA

`int PYNQ_readDMA(PYNQ_AXI_DMA* dma_state, PYNQ_SHARED_MEMORY* shared_memory, size_t offset, size_t length)`

This call will read _length_ bytes of data from DMA engine and write to to the _shared_memory_ location in DRAM, starting at _offset_ which is relative to the start of this block of shared memory. This is non-blocking, i.e. will return before the data transfer has completed, and issuing a read if there is already a read DMA transfer active will result in an error. An integer status code is returned indicating success or failure. 

### Issuing DMA transfers

`int PYNQ_issueDMATransfer(PYNQ_AXI_DMA* dma_state, PYNQ_SHARED_MEMORY* shared_memory, size_t offset, size_t length, AXI_DMA_DIRECTION direction)`

This call is very similar to the write and read calls, but instead the _direction_ of either _AXI_DMA_WRITE_ or _AXI_DMA_READ_ is provided which determines the direction of the DMA. This is non-blocking, i.e. will return before the data transfer has completed, and issuing a transfer on a channel already active with a DMA transfer will result in an error. An integer status code is returned indicating success or failure. 

### Testing for DMA completion

`int PYNQ_testForDMAComplete(PYNQ_AXI_DMA* dma_state, AXI_DMA_DIRECTION direction, int* flag)` 

This API call will test a DMA transfer has completed, or more specifically whether a DMA engine, either the read or write channel, is idle or not. The _direction_ can be either _AXI_DMA_WRITE_ or _AXI_DMA_READ_ and the completion status is written into _flag_, zero representing not completed (e.g. busy, non-idle) and greater than zero indicating completed (e.g. idle). An integer status code is returned indicating success or failure. 

### Waiting for DMA completion

`int PYNQ_waitForDMAComplete(PYNQ_AXI_DMA* dma_state, AXI_DMA_DIRECTION direction)`

This call will block and wait for a DMA transfer to complete. The _direction_ can be either _AXI_DMA_WRITE_ or _AXI_DMA_READ_ and an integer status code is returned indicating success or failure.  

### Closing the DMA block

`int PYNQ_closeDMA(PYNQ_AXI_DMA* dma_state)`

This call will close the connection to the DMA block, _dma_state_, and stop the DMA engines. An integer status code is returned indicating success or failure. 

## High Level Synthesis (HLS)

The PYNQ API provides abstraction over interaction with HLS blocks via the AXI-Lite control port. All these HLS calls should be issued from code running on the PS as sudo.

### Opening an HLS block

`int PYNQ_openHLS(PYNQ_HLS* hls_state, size_t address, size_t width)`

Opens an HLS block at _address_, which is defined in the Vivado address editor, of size _width_ in bytes. The user should declare a variable of type _PYNQ_HLS_ in their code and pass a pointer to this, this variable is then the context of the opened HLS block. An integer status code is returned indicating success or failure. 

### Starting the HLS kernel

`int PYNQ_startHLS(PYNQ_HLS* hls_state)`

This starts the HLS kernel of the block opened as _hls_state_, returning an integer status code is returned indicating success or failure. 

### Testing for HLS kernel completion

`int PYNQ_testHLSCompleted(PYNQ_HLS* hls_state, int* flag)`

Tests whether the HLS kernel at block _hls_state_ has finished execution (completed) or not. This status is written into _flag_, zero indicating it has not completed (e.g. currently running) and above zero indicating it has completed. Internally this also accepts 0x00 as completion, so really represents whether the kernel is idle or not. An integer status code is returned indicating success or failure. 

### Waiting for HLS kernel completion

`int PYNQ_waitForHLS(PYNQ_HLS* hls_state)`

Waits for the HLS kernel at _hls_state_ to finish execution. An integer status code is returned indicating success or failure. 

### Writing to the HLS kernel control port

`int PYNQ_writeToHLS(PYNQ_HLS* hls_state, void* data, size_t offset, size_t length)`

It is common for variables to be passed to the HLS kernel via the AXI-Lite control port (and also other aspects, such as setting the address of AXI slave ports.) This API call provides this, writing _length_ bytes from the _data_ pointer to the HLS kernel of _hls_state_ starting at memory location _offset_ which is relative to the start of the HLS kernel's memory region (i.e. writing to _0x00_ will write to the location commonly used by HLS as the status register). An integer status code is returned indicating success or failure. 

### Reading from the HLS kernel control port

`int PYNQ_readFromHLS(PYNQ_HLS* hls_state, void* data, size_t offset, size_t length)`

It is common to read from the HLS kernel via the AXI-Lite control port (for instance returning data and sending data back via pointers in HLS). This API call provides this, reading _length_ bytes from the HLS kernel of _hls_state_ starting at memory location _offset_ which is relative to the start of the HLS kernel's memory region and putting this in the _data_ pointer. An integer status code is returned indicating success or failure. 

### Closing the HLS block

`int PYNQ_closeHLS(PYNQ_HLS* hls_state)`

Will close the connection to the HLS block represented by _hls_state_. Note that this has no impact on the block itself and will not impact anything that is currently executing by it.


