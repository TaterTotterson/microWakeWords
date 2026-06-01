.. _sln_voice_ffd_intent_engine:

#################
src/intent_engine
#################

This folder contains the intent engine module for the FFD application.

.. list-table:: FFD Intent Engine
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - intent_engine_io.c
     - contains additional io intent engine code
   * - intent_engine_support.c
     - contains general intent engine support code
   * - intent_engine.c
     - contains the implementation of default intent engine code
   * - intent_engine.h
     - header for intent engine code


Major Components
================

The intent engine module provides the application with two API functions:

.. code-block:: c
    :caption: Intent Engine API (intent_engine.h)

    int32_t intent_engine_create(uint32_t priority, void *args);
    void intent_engine_ready_sync(void);
    int32_t intent_engine_sample_push(asr_sample_t *buf, size_t frames);

If replacing the existing model, these are the only two functions that are required to be populated.


intent_engine_create
^^^^^^^^^^^^^^^^^^^^^^^

This function has the role of creating the model running task and providing a pointer, which can be used by the application to handle the output intent result.  In the case of the default configuration, the application provides a FreeRTOS Queue object.

In FFD, the audio pipeline output is on tile 1 and the ASR engine on tile 0.

.. code-block:: c
    :caption: intent_engine_create snippet (intent_engine_io.c)

    #if ASR_TILE_NO == AUDIO_PIPELINE_TILE_NO
        intent_engine_task_create(priority);
    #else
        intent_engine_intertile_task_create(priority);
    #endif

The call to intent_engine_intertile_task_create() will create two threads on tile 0.  One thread is the ASR engine thread.  The other thread is an intertile rx thread, which will interface with the audio pipeline output.

intent_engine_ready_sync
^^^^^^^^^^^^^^^^^^^^^^^^^

This function is called by both tiles and serves to ensure that tile 0 is ready to receive
audio samples before starting the audio pipeline. This is a preventative measure to avoid dropping
samples at startup.

.. code-block:: c
    :caption: intent_engine_create snippet (intent_engine_io.c)

        int sync = 0;
    #if ON_TILE(AUDIO_PIPELINE_TILE_NO)
        size_t len = rtos_intertile_rx_len(intertile_ctx, appconfINTENT_ENGINE_READY_SYNC_PORT, RTOS_OSAL_WAIT_FOREVER);
        xassert(len == sizeof(sync));
        rtos_intertile_rx_data(intertile_ctx, &sync, sizeof(sync));
    #else
        rtos_intertile_tx(intertile_ctx, appconfINTENT_ENGINE_READY_SYNC_PORT, &sync, sizeof(sync));
    #endif

intent_engine_sample_push
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This function has the role of sending the ASR output channel from the audio pipeline to the intent engine.

In FFD, the audio pipeline output is on tile 1 and the ASR engine on tile 0.

.. code-block:: c
    :caption: intent_engine_create snippet (intent_engine_io.c)

    #if appconfINTENT_ENABLED && ON_TILE(AUDIO_PIPELINE_TILE_NO)
    #if ASR_TILE_NO == AUDIO_PIPELINE_TILE_NO
        intent_engine_samples_send_local(
                frames,
                buf);
    #else
        intent_engine_samples_send_remote(
                intertile_ap_ctx,
                frames,
                buf);
    #endif
    #endif

The call to intent_engine_samples_send_remote() will send the audio samples to the previously configured intertile rx thread.


intent_engine_process_asr_result
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This function can be replaced by the application to handle the intent in a completely different manner.


Miscellaneous Functions
^^^^^^^^^^^^^^^^^^^^^^^

The following helper functions are provided for supporting the command processing features that are unique to the default FFD application:

  - intent_engine_keyword_queue_count
  - intent_engine_keyword_queue_complete
  - intent_engine_stream_buf_reset
  - intent_engine_play_response
