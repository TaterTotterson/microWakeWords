// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* STD headers */
#include <string.h>
#include <stdint.h>
#include <xcore/hwtimer.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "stream_buffer.h"

/* Library headers */
#include "generic_pipeline.h"

/* App headers */
#include "app_conf.h"
#include "audio_pipeline.h"
#include "audio_pipeline_dsp.h"
#include "doa/doa_estimator.h"

#if appconfAUDIO_PIPELINE_FRAME_ADVANCE != 240
#error This pipeline is only configured for 240 frame advance
#endif

#if ON_TILE(1)

static void *audio_pipeline_input_i(void *input_app_data)
{
    frame_data_t *frame_data;
    frame_data = pvPortMalloc(sizeof(frame_data_t));
    memset(frame_data, 0x00, sizeof(frame_data_t));

    audio_pipeline_input(input_app_data,
                       (int32_t **)frame_data->aec_reference_audio_samples,
                       4,
                       appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    doa_estimator_process_frame(frame_data->mic_samples_passthrough[0],
                                frame_data->mic_samples_passthrough[1],
                                appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    memcpy(frame_data->samples, frame_data->mic_samples_passthrough, sizeof(frame_data->samples));

    return frame_data;
}

static int audio_pipeline_output_i(frame_data_t *frame_data,
                                   void *output_app_data)
{

    rtos_intertile_tx(intertile_ctx,
                      appconfAUDIOPIPELINE_PORT,
                      frame_data,
                      sizeof(frame_data_t));

    return AUDIO_PIPELINE_FREE_FRAME;
}

void empty_stage(void)
{
    ;
}

static void initialize_pipeline_stages(void)
{
    ;
}

void audio_pipeline_init(
    void *input_app_data,
    void *output_app_data)
{
    const int stage_count = 2;
    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)empty_stage,
        (pipeline_stage_t)empty_stage,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(empty_stage) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(empty_stage) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

    initialize_pipeline_stages();

    generic_pipeline_init((pipeline_input_t)audio_pipeline_input_i,
                        (pipeline_output_t)audio_pipeline_output_i,
                        input_app_data,
                        output_app_data,
                        stages,
                        (const size_t*) stage_stack_sizes,
                        appconfAUDIO_PIPELINE_TASK_PRIORITY,
                        stage_count);

}
#endif /* ON_TILE(1)*/
