# PYNQ API Documentation

In this documents folder we have a number of separate file which discuss different aspects of the API.

* <a href="https://github.com/mesham/pynq_api/blob/master/docs/core.md">Core API</a> which describes the core C PYNQ API

## Error codes

All PYNQ API functions return an integer error code which should be tested in user code. The _PYNQ_SUCCESS_ preprocessor define indicates that the call was successful, whereas _PYNQ_ERROR_ indicates that an error occured whilst executing the call. In the case of errors the API will also often print an associated error message to stderr providing further explanation.
