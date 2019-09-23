# PS side of PYNQ API

This part of the PYNQ API facilitates management of the Processing System (PS) i.e. ARM CPUs running Linux, and PS/PL interface.

## Working with the ARM architecture registers

The ARM CPU provides a number of registers which one might want to interact with for setting specific configuration conditions. This part of the API provides functionality to write to registers at the bit level, and all the reigster interaction calls must be issued from code running as sudo.

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
