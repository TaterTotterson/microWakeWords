// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef AUDIO_PIPELINE_H_
#define AUDIO_PIPELINE_H_

#include <stdint.h>
#include "app_conf.h"

#define AUDIO_PIPELINE_DONT_FREE_FRAME 0
#define AUDIO_PIPELINE_FREE_FRAME      1

void audio_pipeline_init(
        void *input_app_data,
        void *output_app_data);

void audio_pipeline_input(
        void *input_app_data,
        int32_t **input_audio_frames,
        size_t ch_count,
        size_t frame_count);

int audio_pipeline_output(
        void *output_app_data,
        int32_t **output_audio_frames,
        size_t ch_count,
        size_t frame_count);

#endif /* AUDIO_PIPELINE_H_ */
