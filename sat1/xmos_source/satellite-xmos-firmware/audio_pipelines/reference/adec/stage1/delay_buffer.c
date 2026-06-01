// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <stdint.h>
#include <string.h>
#include "delay_buffer.h"

void delay_buffer_init(delay_buf_state_t *state, int default_delay_samples) {
    memset(state->delay_buffer, 0, sizeof(state->delay_buffer));
    memset(&state->curr_idx[0], 0, sizeof(state->curr_idx));
    state->delay_samples = default_delay_samples;
}

void get_delayed_sample(delay_buf_state_t *delay_state, int32_t *sample, int32_t ch) {
    delay_state->delay_buffer[ch][delay_state->curr_idx[ch]] = *sample;
    int32_t abs_delay_samples = (delay_state->delay_samples < 0) ? -delay_state->delay_samples : delay_state->delay_samples;
    // Send back the samples with the correct delay
    uint32_t delay_idx = (
            (DELAY_BUF_MAX_DELAY_SAMPLES + delay_state->curr_idx[ch] - abs_delay_samples)
            % DELAY_BUF_MAX_DELAY_SAMPLES
            );
    *sample = delay_state->delay_buffer[ch][delay_idx];
    delay_state->curr_idx[ch] = (delay_state->curr_idx[ch] + 1) % DELAY_BUF_MAX_DELAY_SAMPLES;
}

void update_delay_samples(delay_buf_state_t *delay_state, int32_t num_samples) {
    delay_state->delay_samples = num_samples;
}

void reset_partial_delay_buffer(delay_buf_state_t *delay_state, int32_t ch) {
    int32_t num_samples = delay_state->delay_samples;
    if(!num_samples) {
        return;
    }
    // Reset delay_state->delay_samples before the current index

    num_samples = (num_samples < 0) ? -num_samples : num_samples;
    // Reset num_samples samples before curr_idx
    int32_t reset_start = (
            (DELAY_BUF_MAX_DELAY_SAMPLES + delay_state->curr_idx[ch] - num_samples)
            % DELAY_BUF_MAX_DELAY_SAMPLES
            );
    if(reset_start < delay_state->curr_idx[ch]) {
        //reset_start hasn't wrapped around
        memset(&delay_state->delay_buffer[ch][reset_start], 0, num_samples*sizeof(int32_t));
    }
    else {
        //reset_start has wrapped around
        memset(&delay_state->delay_buffer[ch][0], 0, delay_state->curr_idx[ch]*sizeof(int32_t));
        int remaining = num_samples - delay_state->curr_idx[ch];
        memset(&delay_state->delay_buffer[ch][DELAY_BUF_MAX_DELAY_SAMPLES - remaining], 0, remaining*sizeof(int32_t));
    }
}
