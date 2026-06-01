// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <platform.h>
#include <xs1.h>
#include <xcore/hwtimer.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "app_conf.h"
#include "platform/driver_instances.h"
#include "ww_model_runner/ww_model_runner.h"

#define ASR_CHANNEL             (0)
#define COMMS_CHANNEL           (1)

#if appconfWW_ENABLED
extern configSTACK_DEPTH_TYPE model_runner_manager_stack_size;

static StreamBufferHandle_t audio_stream = NULL;

void ww_audio_send(rtos_intertile_t *intertile_ctx,
                    size_t frame_count,
                    int32_t (*processed_audio_frame)[2])
{
    configASSERT(frame_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    uint16_t ww_samples[appconfAUDIO_PIPELINE_FRAME_ADVANCE];

    for (int i = 0; i < frame_count; i++) {
        ww_samples[i] = (uint16_t)(processed_audio_frame[i][ASR_CHANNEL] >> 16);
    }

    if(audio_stream != NULL) {
        if (xStreamBufferSend(audio_stream, ww_samples, sizeof(ww_samples), 0) != sizeof(ww_samples)) {
            rtos_printf("lost output samples for ww\n");
        }
    }
}

void ww_task_create(unsigned priority)
{
    audio_stream = xStreamBufferCreate(2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE,
                                       appconfWW_FRAMES_PER_INFERENCE);

    xTaskCreate((TaskFunction_t)model_runner_manager,
                "model_manager",
                model_runner_manager_stack_size,
                audio_stream,
                uxTaskPriorityGet(NULL),
                NULL);
}

#endif
