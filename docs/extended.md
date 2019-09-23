# Extended API
This documentation describes parts of the PYNQ API that provide extended functionality. This is functionality not specific or core to the Zynq or Pynq, but instead covers interaction with very common IP blocks one often uses in their block designs. As such a layer of abstraction over these is thought to be generally useful.

## AXI Direct Memory Access (DMA)

The AXI DMA block provides the ability to perform high performance burst transfers between PS DRAM and the PL. 


(https://pynq.readthedocs.io/en/v2.3/_images/dma.png)
