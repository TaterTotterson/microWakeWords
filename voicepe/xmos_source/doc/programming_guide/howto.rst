
#######
How-Tos
#######

This section includes instructions on anticipated or common software modifications.

*****************************************
Changing the input and output sample rate
*****************************************

In the example design ``app_conf.h`` file, change ``appconfAUDIO_PIPELINE_SAMPLE_RATE`` to either 16000 or 48000.

************************************************************
|I2S| AEC reference input audio & USB processed audio output
************************************************************

The FFVA example design includes 2 basic configurations; INT and UA.  The INT configuration is setup with |I2S| for input and output audio.  The UA configuration is setup with USB for input and output audio.  This HOWTO explains how to modify the FFVA example design for |I2S| input audio and USB output audio.  

In the ``ffva_ua.cmake`` file, changing the ``appconfAEC_REF_DEFAULT`` to ``appconfAEC_REF_I2S`` will result in the expected input frames.

.. code-block:: cmake

    set(FFVA_UA_COMPILE_DEFINITIONS
        ${APP_COMPILE_DEFINITIONS}
        appconfI2S_ENABLED=1
        appconfUSB_ENABLED=1
        appconfAEC_REF_DEFAULT=appconfAEC_REF_I2S

        appconfI2S_MODE=appconfI2S_MODE_MASTER
        MIC_ARRAY_CONFIG_MCLK_FREQ=24576000
    )

For integrating with |I2S| there are a few other differences from the default UA configuration. FFVA was designed to be integrated with an Raspberry Pi for an AVS demo.  And, due to that the INT config uses an externally generated ``MCLK``. When integrating with an external Raspberry Pi ``MCLK``, you will want the following ``FFVA_UA_COMPILE_DEFINITIONS``:

.. code-block:: cmake


    set(FFVA_UA_COMPILE_DEFINITIONS
        ${APP_COMPILE_DEFINITIONS}
        appconfI2S_ENABLED=1
        appconfUSB_ENABLED=1
        appconfAEC_REF_DEFAULT=appconfAEC_REF_I2S

        appconfI2S_MODE=appconfI2S_MODE_SLAVE
        appconfEXTERNAL_MCLK=1
        appconfI2S_AUDIO_SAMPLE_RATE=48000
        MIC_ARRAY_CONFIG_MCLK_FREQ=12288000
    )

``appconfI2S_AUDIO_SAMPLE_RATE`` can also be 16000. Only 48k and 16k conversions is supported in FFVA.

If you enable ``appconfEXTERNAL_MCLK``, the FFVA example application will sit at initialization until we can lock on to that clock source, so it MUST be active during boot.

Since the FFVA example application is not receiving reference audio through USB in this configuration, USB adaptive mode will not adapt to the input.  By default, ffva will output the configured nominal rate.

If you enable ``appconfAEC_REF_DEFAULT=appconfAEC_REF_I2S`` and ``appconfI2S_MODE=appconfI2S_MODE_MASTER``. You need to invert ``I2S_DATA_IN`` and ``I2S_MIC_DATA`` in the ``bsp_config/XK_VOICE_L71/XK_VOICE_L71.xn`` file to have the reference audio play properly.

Lastly, with |I2S| enabled the DAC is always initialized by the FFVA example application. If FFVA cannot be the |I2C| host then it is up to the host to initialize the DAC, like in the AVS demo.
