#include "doa_estimator.h"

#include <limits.h>
#include <string.h>

#define DOA_ESTIMATOR_SAMPLE_SHIFT (12)
#define DOA_ESTIMATOR_MIN_ENERGY   (512)

static volatile doa_estimator_state_t latest_state = {0};

static int64_t abs_i64(int64_t value)
{
    return value < 0 ? -value : value;
}

static int32_t scaled_sample(int32_t sample)
{
    return sample >> DOA_ESTIMATOR_SAMPLE_SHIFT;
}

static int64_t lag_correlation(const int32_t *mic0,
                               const int32_t *mic1,
                               size_t frame_count,
                               int lag)
{
    int64_t corr = 0;
    size_t start0 = 0;
    size_t start1 = 0;
    size_t count = frame_count;

    if (lag > 0) {
        start1 = (size_t)lag;
        count -= (size_t)lag;
    } else if (lag < 0) {
        start0 = (size_t)-lag;
        count -= (size_t)-lag;
    }

    for (size_t i = 0; i < count; i++) {
        corr += (int64_t)scaled_sample(mic0[start0 + i]) *
                (int64_t)scaled_sample(mic1[start1 + i]);
    }

    return corr;
}

static uint32_t clamp_u32(int64_t value)
{
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

static uint32_t mic_energy(const int32_t *mic, size_t frame_count)
{
    int64_t energy = 0;

    if (mic == NULL) {
        return 0;
    }

    for (size_t i = 0; i < frame_count; i++) {
        int32_t sample = scaled_sample(mic[i]);
        energy += (int64_t)sample * (int64_t)sample;
    }

    return clamp_u32(energy);
}

static uint8_t estimate_angle_index(int ew_delay, int ns_delay)
{
    static const int16_t led_x[24] = {
        0, -259, -500, -707, -866, -966, -1000, -966,
        -866, -707, -500, -259, 0, 259, 500, 707,
        866, 966, 1000, 966, 866, 707, 500, 259,
    };
    static const int16_t led_y[24] = {
        -1000, -966, -866, -707, -500, -259, 0, 259,
        500, 707, 866, 966, 1000, 966, 866, 707,
        500, 259, 0, -259, -500, -707, -866, -966,
    };
    int best_index = 12;
    int32_t best_dot = INT32_MIN;

    if (ew_delay == 0 && ns_delay == 0) {
        return 12;
    }

    for (int i = 0; i < 24; i++) {
        int32_t dot = ((int32_t)ew_delay * led_x[i]) + ((int32_t)ns_delay * led_y[i]);
        if (dot > best_dot) {
            best_dot = dot;
            best_index = i;
        }
    }

    return (uint8_t)best_index;
}

static void estimate_pair(const int32_t *mic0,
                          const int32_t *mic1,
                          size_t frame_count,
                          int *best_lag_out,
                          uint8_t *confidence_out,
                          uint8_t *valid_out)
{
    int64_t best_corr = -1;
    int64_t second_corr = -1;
    int best_lag = 0;

    *best_lag_out = 0;
    *confidence_out = 0;
    *valid_out = 0;

    if (mic0 == NULL || mic1 == NULL || frame_count <= (2 * DOA_ESTIMATOR_MAX_LAG_SAMPLES)) {
        return;
    }

    for (int lag = -DOA_ESTIMATOR_MAX_LAG_SAMPLES;
         lag <= DOA_ESTIMATOR_MAX_LAG_SAMPLES;
         lag++) {
        int64_t corr = abs_i64(lag_correlation(mic0, mic1, frame_count, lag));

        if (corr > best_corr) {
            second_corr = best_corr;
            best_corr = corr;
            best_lag = lag;
        } else if (corr > second_corr) {
            second_corr = corr;
        }
    }

    if (best_corr > 0) {
        int64_t gap = best_corr - (second_corr > 0 ? second_corr : 0);
        int64_t confidence = (gap * 255) / best_corr;

        *best_lag_out = best_lag;
        *confidence_out = confidence > 255 ? 255 : (uint8_t)confidence;
        *valid_out = 1;
    }
}

void doa_estimator_process_frame(const int32_t *mic0,
                                 const int32_t *mic1,
                                 size_t frame_count)
{
    doa_estimator_state_t next = {0};
    int delay = 0;
    uint8_t confidence = 0;
    uint8_t valid = 0;

    next.mic_energy[0] = mic_energy(mic0, frame_count);
    next.mic_energy[1] = mic_energy(mic1, frame_count);
    next.energy = clamp_u32((int64_t)next.mic_energy[0] + (int64_t)next.mic_energy[1]);
    next.frame_counter = latest_state.frame_counter + 1;
    next.angle_index = 12;

    estimate_pair(mic0, mic1, frame_count, &delay, &confidence, &valid);

    if (next.energy >= DOA_ESTIMATOR_MIN_ENERGY && valid) {
        next.sample_delay = (int16_t)delay;
        next.confidence = confidence;
        next.flags = DOA_ESTIMATOR_FLAG_VALID;
    }

    latest_state.sample_delay = next.sample_delay;
    latest_state.vertical_delay = next.vertical_delay;
    latest_state.angle_index = next.angle_index;
    latest_state.confidence = next.confidence;
    latest_state.flags = next.flags;
    latest_state.energy = next.energy;
    latest_state.frame_counter = next.frame_counter;
    memcpy((void *)latest_state.mic_energy, next.mic_energy, sizeof(latest_state.mic_energy));
}

void doa_estimator_process_frame_4(const int32_t *east,
                                   const int32_t *west,
                                   const int32_t *north,
                                   const int32_t *south,
                                   size_t frame_count)
{
    doa_estimator_state_t next = {0};
    int ew_delay = 0;
    int ns_delay = 0;
    uint8_t ew_confidence = 0;
    uint8_t ns_confidence = 0;
    uint8_t ew_valid = 0;
    uint8_t ns_valid = 0;

    next.mic_energy[0] = mic_energy(east, frame_count);
    next.mic_energy[1] = mic_energy(west, frame_count);
    next.mic_energy[2] = mic_energy(north, frame_count);
    next.mic_energy[3] = mic_energy(south, frame_count);
    next.energy = clamp_u32((int64_t)next.mic_energy[0] +
                            (int64_t)next.mic_energy[1] +
                            (int64_t)next.mic_energy[2] +
                            (int64_t)next.mic_energy[3]);
    next.frame_counter = latest_state.frame_counter + 1;
    next.angle_index = 12;

    estimate_pair(east, west, frame_count, &ew_delay, &ew_confidence, &ew_valid);
    estimate_pair(north, south, frame_count, &ns_delay, &ns_confidence, &ns_valid);

    if (next.energy >= DOA_ESTIMATOR_MIN_ENERGY && (ew_valid || ns_valid)) {
        next.sample_delay = (int16_t)ew_delay;
        next.vertical_delay = (int16_t)ns_delay;
        next.angle_index = estimate_angle_index(ew_delay, ns_delay);

        if (ew_valid && ns_valid) {
            next.confidence = ew_confidence < ns_confidence ? ew_confidence : ns_confidence;
        } else {
            next.confidence = ew_valid ? ew_confidence : ns_confidence;
        }

        next.flags = DOA_ESTIMATOR_FLAG_VALID | DOA_ESTIMATOR_FLAG_FOUR_MIC;
    }

    latest_state.sample_delay = next.sample_delay;
    latest_state.vertical_delay = next.vertical_delay;
    latest_state.angle_index = next.angle_index;
    latest_state.confidence = next.confidence;
    latest_state.flags = next.flags;
    latest_state.energy = next.energy;
    latest_state.frame_counter = next.frame_counter;
    memcpy((void *)latest_state.mic_energy, next.mic_energy, sizeof(latest_state.mic_energy));
}

void doa_estimator_get_state(doa_estimator_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->sample_delay = latest_state.sample_delay;
    state->vertical_delay = latest_state.vertical_delay;
    state->angle_index = latest_state.angle_index;
    state->confidence = latest_state.confidence;
    state->flags = latest_state.flags;
    state->energy = latest_state.energy;
    state->frame_counter = latest_state.frame_counter;
    memcpy(state->mic_energy, (const void *)latest_state.mic_energy, sizeof(state->mic_energy));
}
