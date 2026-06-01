.. _sln_voice_low_power_ffd_intent_engine:

#################
src/intent_engine
#################

This folder contains the intent engine module for the low power FFD application.

.. list-table:: Low Power FFD Intent Engine
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

The intent engine module provides the application with the following primary API functions:

.. code-block:: c
    :caption: Intent Engine API (intent_engine.h)

    int32_t intent_engine_create(uint32_t priority, void *args);
    void intent_engine_ready_sync(void);
    int32_t intent_engine_sample_push(asr_sample_t *buf, size_t frames);

These APIs provide the functionality needed to feed audio pipeline samples into the ASR engine.


intent_engine_create
^^^^^^^^^^^^^^^^^^^^

This function has the role of creating the model running task and providing a pointer, which can be
used by the application to handle the output intent result. In the case of the default configuration,
the application provides a FreeRTOS Queue object.

In Low Power FFD, the audio pipeline output is on tile 1 and the ASR engine on tile 0.

.. code-block:: c
    :caption: intent_engine_create snippet (intent_engine_io.c)

    intent_engine_intertile_task_create(priority);

The call to intent_engine_intertile_task_create() will create two threads on tile 0. One thread is
the ASR engine thread. The other thread is an intertile RX thread, which will interface with the
audio pipeline output.


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
^^^^^^^^^^^^^^^^^^^^^^^^^

This function has the role of sending the ASR output channel from the audio pipeline to the intent engine.

In Low Power FFD, the audio pipeline output is on tile 1 and the ASR engine on tile 0.

.. code-block:: c
    :caption: intent_engine_create snippet (intent_engine_io.c)

        intent_engine_samples_send_remote(
                intertile_ap_ctx,
                frames,
                buf);

The call to intent_engine_samples_send_remote() will send the audio samples to the previously
configured intertile RX thread.


intent_engine_process_asr_result
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This function can be replaced by the application to handle the intent in a completely different manner.


Low Power Components
====================

The following APIs are the intent engine mechanisms needed by the power control task.

.. code-block:: c
    :caption: Low Power APIs (intent_engine.h)

    void intent_engine_full_power_request(void);
    void intent_engine_low_power_accept(void);

In this implementation, it is the responsibility of tile 0 (intent engine tile) to determine when
to request a transition into low power mode; however, tile 1 may reject the request. When tile 1
accepts the request (via `LOW_POWER_ACK`), the power control task calls `intent_engine_low_power_accept`.
When tile 1 rejects the request (via `LOW_POWER_NAK`), the power control task calls
`intent_engine_full_power_request`.

.. note::
    There is an additional `LOW_POWER_HALT` response where the power control task calls
    `intent_engine_halt`. This is primarily for end-of-evaluation handling logic for the underlying
    ASR engine and is not needed for a normal application.

After tile 1 accepts the low power request, tile 0 begins preparations for entering low power by
locking various resources and waiting for any enqueued commands to finish up. The helper functions
below are provided for this purpose.

.. code-block:: c
    :caption: Low Power Helper Functions (intent_engine.h)

    int32_t intent_engine_keyword_queue_count(void);
    void intent_engine_keyword_queue_complete(void);
    uint8_t intent_engine_low_power_ready(void);

Before tile 1 sends `LOW_POWER_ACK` it also stops pushing audio samples via `intent_engine_sample_push`.
After receiving the low power response, the application may clear the stream buffer and keyword
queue to avoid processing stale samples/commands when returning to full power mode. The functions
below provide this functionality.

.. code-block:: c
    :caption: Low Power Helper Functions (intent_engine.h)

    void intent_engine_keyword_queue_reset(void);
    void intent_engine_stream_buf_reset(void);

.. note::
    Since it is possible that a command is spoken/recognized between the time when tile 0 requests
    low power and when tile 1 responds to the request, the application should not reset these
    buffer entities until it has received `LOW_POWER_ACK`; otherwise, recognized commands may be lost.


Evaluation Specific Components
==============================

The following functions are provided for the primary purpose of facilitating the evaluation of the
ASR model. The provided ASR models have evaluation periods which will end due to various factors.
When the evaluation period ends, the application logic halts the intent engine via `intent_engine_halt`.
This is primarily to ensure the device remains in full-power mode to allow functionality that may be
exclusive to tile 0 to function.

.. code-block:: c
    :caption: Evaluation-specific Helper Functions (intent_engine.h)

    void intent_engine_halt(void);
