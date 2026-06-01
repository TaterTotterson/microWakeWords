#pragma once

#include <stddef.h>
#include <stdint.h>

#define DOA_ESTIMATOR_MAX_LAG_SAMPLES (4)
#define DOA_ESTIMATOR_FLAG_VALID      (1u << 0)
#define DOA_ESTIMATOR_FLAG_FOUR_MIC   (1u << 1)

typedef struct {
    /* Positive values mean mic1 best matches a later sample than mic0. */
    int16_t sample_delay;
    int16_t vertical_delay;
    uint8_t angle_index;
    uint8_t confidence;
    uint8_t flags;
    uint32_t energy;
    uint32_t frame_counter;
    uint32_t mic_energy[4];
} doa_estimator_state_t;

void doa_estimator_process_frame(const int32_t *mic0,
                                 const int32_t *mic1,
                                 size_t frame_count);

void doa_estimator_process_frame_4(const int32_t *east,
                                   const int32_t *west,
                                   const int32_t *north,
                                   const int32_t *south,
                                   size_t frame_count);

void doa_estimator_get_state(doa_estimator_state_t *state);
