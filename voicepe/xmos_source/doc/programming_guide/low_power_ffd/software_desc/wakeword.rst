.. _sln_voice_low_power_ffd_wakeword:

#############
src/wakeword
#############

This folder contains the wake word recognition functionality for the Low Power FFD application.

.. list-table:: Low Power FFD wakeword
   :widths: 45 55
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - wakeword.c
     - The wake word engine source file. Responsible for the transfer of audio samples into the ASR and handling of wake word detection events.
   * - wakeword.h
     - The wake word engine header file.


Major Components
================

The wakeword module provides the application with two API functions:

.. code-block:: c
    :caption: Wake Word API (wakeword.h)

    void wakeword_init(void);
    wakeword_result_t wakeword_handler(asr_sample_t *buf, size_t num_frames);


wakeword_init
^^^^^^^^^^^^^

This function performs the required initialization for the wakeword_handler() function to
operate. This involves initializing an instance of devmem_manager_t for use by the ASR abstraction
layer and initialization of the ASR unit itself. It is to be called once during startup before any
call to wakeword_handler() occurs.


wakeword_handler
^^^^^^^^^^^^^^^^

This function performs wake word detection logic and reports back to the caller a result, indicating
whether a wake word was recognized. Note: this routine is called by audio_pipeline_output(), meaning
this routine's logic should be kept to a minimum to ensure timing requirements are met.

In this implementation a single wake word ID of 1 is defined. Minimal adaptation is needed to support
other models supporting other IDs or more than one valid wake word.
