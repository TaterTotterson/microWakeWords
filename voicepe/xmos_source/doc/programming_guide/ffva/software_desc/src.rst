
.. _sln_voice_ffva_src:

###
src
###

This folder contains the core application source.

.. list-table:: FFVA src
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - gpio_test directory
     - contains general purpose input handling task
   * - usb directory
     - contains intent handling code
   * - ww_model_runner directory
     - contains placeholder wakeword model runner task
   * - app_conf_check.h
     - header to validate app_conf.h
   * - app_conf.h
     - header to describe app configuration
   * - config.xscope
     - xscope configuration file
   * - ff_appconf.h
     - default fatfs configuration header
   * - FreeRTOSConfig.h
     - header to describe FreeRTOS configuration
   * - main.c
     - main application source file


Main
====

The major components of main are:

.. code-block:: c
    :caption: Main components (main.c)

    void startup_task(void *arg)
    void tile_common_init(chanend_t c)
    void main_tile0(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
    void main_tile1(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
    void i2s_rate_conversion_enable(void)
    size_t i2s_send_upsample_cb(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *send_buf, size_t samples_available)

    size_t i2s_send_downsample_cb(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *receive_buf, size_t sample_spaces_free)

startup_task
^^^^^^^^^^^^

This function has the role of launching tasks on each tile.  For those familiar with XCORE, it is comparable to the main par loop in an XC main.


tile_common_init
^^^^^^^^^^^^^^^^

This function is the common tile initialization, which initializes the bsp_config, creates the startup task, and starts the FreeRTOS kernel.


main_tile0
^^^^^^^^^^

This function is the application C entry point on tile 0, provided by the SDK.


main_tile1
^^^^^^^^^^

This function is the application C entry point on tile 1, provided by the SDK.


i2s_rate_conversion_enable
^^^^^^^^^^^^^^^^^^^^^^^^^^

This application features 16kHz and 48kHz audio input and output. The XMOS DPS blocks operate on 16kHz audio. Input streams are downsampled when needed. Output streams are upsampled when needed. When in |I2S| modes This function is called by the bsp_config to enable the |I2S| sample rate conversion.


i2s_send_upsample_cb
^^^^^^^^^^^^^^^^^^^^

This function is the |I2S| upsampling callback.

i2s_send_downsample_cb
^^^^^^^^^^^^^^^^^^^^^^

This function is the |I2S| downsampling callback.

