# Programmable System PYNQ API

This part of the PYNQ API facilitates management of the Processing System (PS) i.e. ARM CPUs running Linux, and PS/PL interface.

* [Working with the ARM architecture registers](#working-with-the-arm-architecture-registers)
* [Clocks](#clocks)

## Working with the ARM architecture registers

The ARM CPU provides a number of registers which one might want to interact with for setting specific configuration conditions. This part of the API provides functionality to write to registers at the bit level, and all the register interaction calls must be issued from code running as sudo.

### Opening a register

`int PYNQ_openRegister(PYNQ_REGISTER* reg_state, size_t address, size_t width)`

This call will open a register on the ARM at memory address _address_ of size _width_ in bytes. The user should pass the pointer to a variable of type _PYNQ_REGISTER_ defined in their code, and this is used as a descriptor of that register during interaction. This call returns an integer status flag indicating success or failure.

### Setting a single bit in the register

`int PYNQ_setRegister(PYNQ_REGISTER* reg_state, size_t offset, int index, unsigned int* data)`

This call will set a single bit at bitwise location _index_ in the _data_ unsigned integer provided as a pointer. This is set at the location _offset_ which is relative to the start of the register as defined by the _address_ in the open call (i.e. _0x0_ will write to the start of the register). This call returns an integer status flag indicating success or failure.

### Setting a range of bits in the register

`int PYNQ_setRegisterRange(PYNQ_REGISTER* reg_state, size_t offset, int start_index, int end_index, unsigned int* data)`

This call will set a continuous range of bits between bitwise location _start_index_ and _end_index_ in the _data_ unsigned integer provided as a pointer. It makes no difference whether _start_index_ is smaller or larger than _end_index_, in either situation the appropriate bits will be set. This set is performed at the location _offset_ which is relative to the start of the register as defined by the _address_ in the open call (i.e. _0x0_ will write to the start of the register). This call returns an integer status flag indicating success or failure.

### Reading a single bit from the register

`int PYNQ_readRegister(PYNQ_REGISTER* reg_state, size_t offset, int index, unsigned int* data)`

This API call will read a single bit at bitwise location _index_ from the register at the location _offset_ which is relative to the start of the register as defined by _address_ in the open call (i.e. _0x0_ will read from the start of the register). This bit is then written to the _index_ location in the _data_ unsigned integer provided as a pointer. This call returns an integer status flag indicating success or failure.

### Reading a range of bits from the register

`int PYNQ_readRegisterRange(PYNQ_REGISTER* reg_state, size_t address, int start_index, int end_index, unsigned int* data)`

This API call will read a continuous range of bits between bitwise location _start_index_ and _end_index_ from the register at the location _offset_ which is relative to the start of the register as defined by _address_ in the open call (i.e. _0x0_ will read from the start of the register). It makes no difference whether _start_index_ is smaller or larger than _end_index_, in either situation the appropriate bits will be read. This bit is then written between the _start_index_ and _end_index_ bitwise locations in the _data_ unsigned integer provided as a pointer. This call returns an integer status flag indicating success or failure.

### Closing a register

`int PYNQ_closeRegister(PYNQ_REGISTER* reg_state)`

Closes an opened register denoted by the _reg_state_ pointer.

## Clocks

Using this functionality one can obtain the frequency of the CPU clock and all the PL clocks. All frequencies are in megahertz (MHz) Users can also set the frequency of PL clocks to other values. All these calls must be issued from code running as sudo.

### Getting CPU clock frequency

`int PYNQ_getCPUClockFreq(float* cpu_clock_frequency)`

This call retrieves the frequency of the ARM CPU (PS) clock in MHz and writes it to the float variable passed via the _cpu_clock_frequency_ pointer. This call returns an integer status flag indicating success or failure.

### Getting PL clock frequency

`int PYNQ_getPLClockFreq(unsigned int clock_index, float* pl_clock_frequency)`

This API call retrieves the frequency of one of the four PL clock, as determined by _clock_index_ in MHz and writes it to the float variable passed via the _pl_clock_frequency_ pointer. This call returns an integer status flag indicating success or failure.

### Setting PL clock frequency

`int PYNQ_setPLClockFreq(unsigned int clock_index, float * freq, int * div0, int *div1)`

This call sets the clock frequency of one of the four PL clock using the _freq_ float provided (in Mhz) along with the two divisors _div0_ and _div1_. Note that one of _div0_ or _div1_ must be provided, if both are provided then values are set directly from these, otherwise the missing divisor is automatically calculated based on the provided _freq_ and divisor. This call returns an integer status flag indicating success or failure.
