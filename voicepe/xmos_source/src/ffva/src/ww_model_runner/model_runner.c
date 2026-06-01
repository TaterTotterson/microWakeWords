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
