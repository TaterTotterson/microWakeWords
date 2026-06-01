// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef WW_MODEL_RUNNER_H_
#define WW_MODEL_RUNNER_H_

#include "FreeRTOS.h"

void ww_task_create(unsigned priority);

void ww_audio_send(rtos_intertile_t *intertile_ctx,
                   size_t frame_count,
                   int32_t (*processed_audio_frame)[2]);

void model_runner_manager(void *args);

#endif /* WW_MODEL_RUNNER_H_ */
