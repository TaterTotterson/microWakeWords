
**********************
Modifying the Software
**********************

Implementing the ASR API
========================

Begin your ASR port by creating a new folder under ``modules/asr/``.  The ``asr.h`` and ``device_memory.h`` files include comments detailing the public API methods and parameters.  ASR ports that implement the public API defined can easily be added to current and future XCORE-VOICE example designs that support speech recognition.

Pay close attention to the functions:
- ``asr_printf``
- ``devmem_malloc``
- ``devmem_free``
- ``devmem_read_ext``
- ``devmem_read_ext_async``
- ``devmem_read_ext_wait``

ASR libraries should call ``asr_printf`` instead of ``printf`` or xcore's ``debug_printf``.

ASR libraries must not call ``malloc`` directly to allocate dynamic memory. Instead call the ``devmem_malloc`` and ``devmem_free`` functions.  This allows the application to provide alternative implementations of these functions - like ``pvPortMalloc`` and ``vPortFree`` in a FreeRTOS application.  

The ``devmem_read_ext`` function is provided to load data directly from external memory (QSPI flash or LPDDR) into SRAM. This is the recommended 
way to load coefficients or blocks of data from a model.  It is far more efficient to load the data into SRAM and perform any math on the 
data while it is in SRAM.  The ``devmem_read_ext`` function a signature similar to ``memcpy``.  The caller is responsible for 
allocating the destination buffer.

Like ``devmem_read_ext``, the ``devmem_read_ext_async`` function is provided to load data directly from external memory (QSPI flash or LPDDR) into SRAM. ``devmem_read_ext_async`` differs in that it does not block the caller's thread.  Instead it loads the data in another thread.  One must have a free core when calling ``devmem_read_ext_async`` or an exception will be raised.  ``devmem_read_ext_async`` returns a handle that can later be used to wait for the load to complete.  Call ``devmem_read_ext_wait`` to block the callers thread until the load is complete.  Currently, each call to ``devmem_read_ext_async`` must be followed by a call to ``devmem_read_ext_wait``.  You can not have more than one read in flight at a time.  

.. note::

  XMOS provides an arithmetic and DSP library which leverages the XS3 Vector Processing Unit (VPU) to accelerate costly operations on vectors of 16- or 32-bit data. Included are functions for block floating-point arithmetic, fast Fourier transforms, discrete cosine transforms, linear filtering and more.  See the XMath Programming Guide for more information.

.. note::

  To minimize SRAM scratch space usage, some ASR ports load coefficients into SRAM in chunks.  This is useful when performing a routine  such as a vector matrix multiply as this operation can be performed on a portion of the matrix at a time.

When the port of the new ASR is complete, you can use the example in ``examples/speech_recognition`` to test it.

.. note::

  You may also need to modify ``BRICK_SIZE_SAMPLES`` in ``app_conf.h`` to match the number of audio samples expected per process for your ASR port.  In other example designs, this is defined by ``appconfINTENT_SAMPLE_BLOCK_LENGTH``.  This is set to 240 in the existing example designs.  

In the current source code, the model data (and optional grammar data) are set in ``examples/speech_recognition/src/process_file.c``.  Modify these variables to reflect your data.  The remainder of the API should be familiar to ASR developers.  The API can be extended if necessary.


Flashing Models
===============

To flash your model, modify the ``--data`` argument passed to ``xflash`` command in the :ref:`sln_voice_asr_programming_guide_flash_model` section.

See ``examples/speech_recognition/asr_example/asr_example_model.h`` to see how the model's flash address is defined.

Placing Models in SRAM
======================

Small models (near or under 100kB in size) may be placed in SRAM.  See ``examples/speech_recognition/asr_example/asr_example_model.c`` for more information on placing your model in SRAM.

*******
ASR API
*******

.. doxygengroup:: asr_api
   :content-only:

*****************
Device Memory API
*****************

.. doxygengroup:: devmem_api
   :content-only:

|newpage|
