#pragma once

#include <stddef.h>
#include <stdint.h>

#define DOA_ESTIMATOR_MAX_LAG_SAMPLES (4)
#define DOA_ESTIMATOR_FLAG_VALID      (1u << 0)

typedef struct {
    /* Positive values mean mic1 best matches a later sample than mic0. */
    int16_t sample_delay;
    uint8_t confidence;
    uint8_t flags;
    uint32_t energy;
    uint32_t frame_counter;
} doa_estimator_state_t;

void doa_estimator_process_frame(const int32_t *mic0,
                                 const int32_t *mic1,
                                 size_t frame_count);

void doa_estimator_get_state(doa_estimator_state_t *state);
