
|newpage|

**********************
Software Modifications
**********************

The FFD example design consists of three major software blocks, the audio pipeline, keyword spotter, and keyword handler.  This section will go into detail on how to replace each/all of these subsystems.

.. figure:: diagrams/ffd_diagram.drawio.png
   :align: center
   :scale: 80 %
   :alt: ffd diagram


It is highly recommended to be familiar with the application as a whole before attempting replacing these functional units.  This information can be found here:
:ref:`sln_voice_ffd_software_description`

See :ref:`sln_voice_ffd_software_description` for more details on the memory footprint and CPU usage of the major software components.

Replacing XCORE-VOICE DSP Block
-------------------------------

The audio pipeline can be replaced by making changes to the `audio_pipeline.c` file.

It is up to the user to ensure that the input and output frames of the audio pipeline remain the same, or the remainder of the application will not function properly.

This section will walk through an example of replacing the XMOS NS stage, with a custom stage foo.

Declaration and Definition of DSP Context
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline.c)

    typedef struct ns_stage_ctx {
        ns_state_t state;
    } ns_stage_ctx_t;

    static ns_stage_ctx_t ns_stage_state = {};

With:

.. code-block:: c
    :caption: Foo (audio_pipeline.c)

    typedef struct foo_stage_ctx {
        /* Your required state context here */
    } foo_stage_ctx_t;

    static foo_stage_ctx_t foo_stage_state = {};


DSP Function
^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline.c)

    static void stage_ns(frame_data_t *frame_data)
    {
    #if appconfAUDIO_PIPELINE_SKIP_NS
        (void) frame_data;
    #else
        int32_t ns_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
        configASSERT(NS_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        ns_process_frame(
                    &ns_stage_state.state,
                    ns_output,
                    frame_data->samples[0]);
        memcpy(frame_data->samples, ns_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
    #endif
    }

With:

.. code-block:: c
    :caption: Foo (audio_pipeline.c)

    static void stage_foo(frame_data_t *frame_data)
    {
        int32_t foo_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
        foo_process_frame(
                    &foo_stage_state.state,
                    foo_output,
                    frame_data->samples[0]);
        memcpy(frame_data->samples, foo_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
    }

Runtime Initialization
^^^^^^^^^^^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline.c)

    ns_init(&ns_stage_state.state);

With:

.. code-block:: c
    :caption: Foo (audio_pipeline.c)

    foo_init(&foo_stage_state.state);


Audio Pipeline Setup
^^^^^^^^^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline.c)

    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_vnr_and_ic,
        (pipeline_stage_t)stage_ns,
        (pipeline_stage_t)stage_agc,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_vnr_and_ic) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_ns),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_agc) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

With:

.. code-block:: c
    :caption: Foo (audio_pipeline.c)

    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_vnr_and_ic,
        (pipeline_stage_t)stage_foo,
        (pipeline_stage_t)stage_agc,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_vnr_and_ic) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_foo),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_agc) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

It is also possible to add or remove stages.  Refer to the RTOS Framework documentation on the generic pipeline sw_service.

Replacing Example Design Interfaces
-----------------------------------

It may be desired to have a different output interface to talk to a host, or not have a host at all and handle the intent local to the XCORE device.

Different Peripheral IO
^^^^^^^^^^^^^^^^^^^^^^^

To add or remove a peripheral IO, modify the bsp_config accordingly.  Refer to documentation inside the RTOS Framework on how to instantiate different RTOS peripheral drivers.

|newpage|

Direct Control
^^^^^^^^^^^^^^

In a single controller system, the XCORE can be used to control peripherals directly.

The proc_keyword_res task can be modified as follows:

.. code-block:: c
    :caption: Intent Handler (intent_handler.c)

    static void proc_keyword_res(void *args) {
        QueueHandle_t q_intent = (QueueHandle_t) args;
        int32_t id = 0;

        while(1) {
            xQueueReceive(q_intent, &id, portMAX_DELAY);

            /* User logic here */
        }
    }

This code example will receive the ID of each intent, and can be populated by any user application logic.  User logic can use other RTOS drivers to control various peripherals, such as screens, motors, lights, etc, based on the intent engine outputs.

.. figure:: diagrams/ffd_direct_control_diagram.drawio.png
   :align: center
   :scale: 80 %
   :alt: ffd host direct control diagram

|newpage|
