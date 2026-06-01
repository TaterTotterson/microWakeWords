.. _sln_voice_low_power_ffd_src:

###
src
###

This folder contains the core application source.

.. list-table:: FFD src
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - gpio_ctrl directory
     - contains general purpose input handling and LED handling tasks
   * - intent_engine directory
     - contains intent engine code
   * - intent_handler directory
     - contains intent handling code
   * - power directory
     - contains low power control logic and related audio buffer
   * - rtos_conf directory
     - contains default FreeRTOS configuration headers
   * - wakeword directory
     - contains wake word detection code
   * - app_conf_check.h
     - header to validate app_conf.h
   * - app_conf.h
     - header to describe app configuration
   * - config.xscope
     - xscope configuration file
   * - ff_appconf.h
     - default fatfs configuration header
   * - main.c
     - main application source file
   * - device_memory_impl.c
     - contains XCORE device memory functions for supporting ASR functionality
   * - device_memory_impl.h
     - header for the device memory implementation


Audio Pipeline
==============

The audio pipeline module provides the application with three API functions:

.. code-block:: c
    :caption: Audio Pipeline API (audio_pipeline.h)

    void audio_pipeline_init(
            void *input_app_data,
            void *output_app_data);

    void audio_pipeline_input(
            void *input_app_data,
            int32_t **input_audio_frames,
            size_t ch_count,
            size_t frame_count);

    int audio_pipeline_output(
            void *output_app_data,
            int32_t **output_audio_frames,
            size_t ch_count,
            size_t frame_count);


audio_pipeline_init
^^^^^^^^^^^^^^^^^^^

This function has the role of creating the audio pipeline, with two optional application pointers
which are provided to the application in the audio_pipeline_input() and audio_pipeline_output() callbacks.

In Low Power FFD, the audio pipeline is initialized with no additional arguments, and instantiates a
3 stage pipeline on tile 1, as described in:
:ref:`sln_voice_low_power_ffd_ap`


audio_pipeline_input
^^^^^^^^^^^^^^^^^^^^

This function has the role of providing the audio pipeline with the input frames.

In Low Power FFD, the input is received from the rtos_mic_array driver.


audio_pipeline_output
^^^^^^^^^^^^^^^^^^^^^

This function has the role of receiving the processed audio pipeline output.

In Low Power FFD, the output is sent to both the wake word handler and the intent engine. Because
the intent engine will be suspended in low power mode and that there is a finite time that it takes
to resume full power operation, there is a ring buffer placed between the audio output received
from this routine and the intent engine's stream buffer.


Main
====

The major components of main are:

.. code-block:: c
    :caption: Main components (main.c)

    void startup_task(void *arg)
    void vApplicationMinimalIdleHook(void)
    void tile_common_init(chanend_t c)
    void main_tile0(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
    void main_tile1(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)


startup_task
^^^^^^^^^^^^

This function has the role of launching tasks on each tile. For those familiar with XCORE, it is comparable to the main par loop in an XC main.


vApplicationMinimalIdleHook
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a FreeRTOS callback. By calling "waiteu" without events configured, this has the effect of both MIPs and power savings on XCORE.

.. code-block:: c
    :caption: vApplicationMinimalIdleHook (main.c)

    asm volatile("waiteu");

tile_common_init
^^^^^^^^^^^^^^^^

This function is the common tile initialization, which initializes the bsp_config, creates the startup task, and starts the FreeRTOS kernel.


main_tile0
^^^^^^^^^^

This function is the application C entry point on tile 0, provided by the SDK.


main_tile1
^^^^^^^^^^

This function is the application C entry point on tile 1, provided by the SDK.
