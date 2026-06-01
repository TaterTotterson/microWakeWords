
|newpage|

**********************
Software Modifications
**********************

The FFVA example design consists of three major software blocks, the audio interface, audio pipeline, and placeholder for a keyword handler. This section will go into detail on how to modify each/all of these subsystems.

.. figure:: diagrams/ffva_diagram.drawio.png
   :align: center
   :scale: 80 %
   :alt: ffva diagram


It is highly recommended to be familiar with the application as a whole before attempting replacing these functional units.

See :ref:`sln_voice_memory_cpu` for more details on the memory footprint and CPU usage of the major software components.

Replacing XCORE-VOICE DSP Block
-------------------------------

The audio pipeline can be replaced by making changes to the ``audio_pipeline.c`` file.

It is up to the user to ensure that the input and output frames of the audio pipeline remain the same, or the remainder of the application will not function properly.

This section will walk through an example of replacing the XMOS NS stage, with a custom stage foo.

Declaration and Definition of DSP Context
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline_t0.c)

    static ns_stage_ctx_t DWORD_ALIGNED ns_stage_state = {};


With:

.. code-block:: c
    :caption: Foo (audio_pipeline_t0.c)

    typedef struct foo_stage_ctx {
        /* Your required state context here */
    } foo_stage_ctx_t;

    static foo_stage_ctx_t foo_stage_state = {};


DSP Function
^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline_t0.c)

    static void stage_ns(frame_data_t *frame_data)
    {
    #if appconfAUDIO_PIPELINE_SKIP_NS
    #else
        int32_t DWORD_ALIGNED ns_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
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
    :caption: Foo (audio_pipeline_t0.c)

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
    :caption: XMOS NS (audio_pipeline_t0.c)

    ns_init(&ns_stage_state.state);

With:

.. code-block:: c
    :caption: Foo (audio_pipeline_t0.c)

    foo_init(&foo_stage_state.state);


Audio Pipeline Setup
^^^^^^^^^^^^^^^^^^^^

Replace:

.. code-block:: c
    :caption: XMOS NS (audio_pipeline_t0.c)

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
    :caption: Foo (audio_pipeline_t0.c)

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


Populating a Keyword Engine Block
-------------------------------------

To add a keyword engine block, a user may populate the existing ``model_runner_manager()`` function with their model:

.. code-block:: c
    :caption: Model Runner (model_runner.c)

    configSTACK_DEPTH_TYPE model_runner_manager_stack_size = 287;

    void model_runner_manager(void *args)
    {
        StreamBufferHandle_t input_queue = (StreamBufferHandle_t)args;

        int16_t buf[appconfWW_FRAMES_PER_INFERENCE];

        /* Perform any initialization here */

        while (1)
        {
            /* Receive audio frames */
            uint8_t *buf_ptr = (uint8_t*)buf;
            size_t buf_len = appconfWW_FRAMES_PER_INFERENCE * sizeof(int16_t);
            do {
                size_t bytes_rxed = xStreamBufferReceive(input_queue,
                                                        buf_ptr,
                                                        buf_len,
                                                        portMAX_DELAY);
                buf_len -= bytes_rxed;
                buf_ptr += bytes_rxed;
            } while(buf_len > 0);

            /* Perform inference here */
            // rtos_printf("inference\n");
        }
    }

Populate initialization and inference engine calls where commented. After adding user code, the stack size of the task will need to be adjusted accordingly based on the engine being used. The input streambuffer must be emptied at least at the rate of the audio pipeline otherwise frames will be lost.


Replacing Example Design Interfaces
-----------------------------------

It may be desired to have a different input or output interfaces to talk to a host.

Hybrid Audio Peripheral IO
^^^^^^^^^^^^^^^^^^^^^^^^^^

One example use case may be to create a hybrid audio solution where reference frames or output audio streams are used over an interface other than |I2S| or USB.

.. code-block:: c
    :caption: Audio Pipeline Input (main.c)

    void audio_pipeline_input(void *input_app_data,
                            int32_t **input_audio_frames,
                            size_t ch_count,
                            size_t frame_count)
    {
        (void) input_app_data;
        int32_t **mic_ptr = (int32_t **)(input_audio_frames + (2 * frame_count));

        static int flushed;
        while (!flushed) {
            size_t received;
            received = rtos_mic_array_rx(mic_array_ctx,
                                        mic_ptr,
                                        frame_count,
                                        0);
            if (received == 0) {
                rtos_mic_array_rx(mic_array_ctx,
                                mic_ptr,
                                frame_count,
                                portMAX_DELAY);
                flushed = 1;
            }
        }

        rtos_mic_array_rx(mic_array_ctx,
                        mic_ptr,
                        frame_count,
                        portMAX_DELAY);

        /* Your ref input source here */
    }

Refer to documentation inside the RTOS Framework on how to instantiate different RTOS peripheral drivers. Populate the above code snippet with your input frame source. Refer to the default application for an example of populating reference via |I2S| or USB.

.. code-block:: c
    :caption: Audio Pipeline Output (main.c)

    int audio_pipeline_output(void *output_app_data,
                            int32_t **output_audio_frames,
                            size_t ch_count,
                            size_t frame_count)
    {
        (void) output_app_data;

        /* Your output sink here */

    #if appconfWW_ENABLED
        ww_audio_send(intertile_ctx,
                    frame_count,
                    (int32_t(*)[2])output_audio_frames);
    #endif

        return AUDIO_PIPELINE_FREE_FRAME;
    }

Refer to documentation inside the RTOS Framework on how to instantiate different RTOS peripheral drivers. Populate the above code snippet with your output frame sink. Refer to the default application for an example of outputting the ASR channel via |I2S| or USB.


Different Peripheral IO
^^^^^^^^^^^^^^^^^^^^^^^

To add or remove a peripheral IO, modify the bsp_config accordingly.  Refer to documentation inside the RTOS Framework on how to instantiate different RTOS peripheral drivers.


Application Filesystem Usage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This application is equipped with a FAT filesystem in flash for general use. To add files to the filesystem, simply place them in the `filesystem_support` directory before running the filesystem setup commands in :doc:`Deploying the Firmware with Linux or macOS <deploying/linux_macos>` or :doc:`Deploying the Firmware with Native Windows <deploying/native_windows>`.

The application can access the filesystem via the `FatFS` API.

|newpage|
