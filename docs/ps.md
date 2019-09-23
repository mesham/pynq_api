# PS side of PYNQ API

This part of the PYNQ API facilitates management of the Processing System (PS) i.e. ARM CPUs running Linux, and PS/PL interface.

## Working with the ARM architecture registers

The ARM CPU provides a number of registers which one might want to interact with for setting specific configuration conditions. All the reigster interaction calls must be issued from code running as sudo.
