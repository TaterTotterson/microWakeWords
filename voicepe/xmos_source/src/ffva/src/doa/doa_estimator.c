#include "doa_estimator.h"

#include <limits.h>

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

void doa_estimator_process_frame(const int32_t *mic0,
                                 const int32_t *mic1,
                                 size_t frame_count)
{
    doa_estimator_state_t next = {0};
    int64_t energy = 0;
    int64_t best_corr = -1;
    int64_t second_corr = -1;
    int best_lag = 0;

    if (mic0 == NULL || mic1 == NULL || frame_count <= (2 * DOA_ESTIMATOR_MAX_LAG_SAMPLES)) {
        return;
    }

    for (size_t i = 0; i < frame_count; i++) {
        int32_t s0 = scaled_sample(mic0[i]);
        int32_t s1 = scaled_sample(mic1[i]);
        energy += (int64_t)s0 * (int64_t)s0;
        energy += (int64_t)s1 * (int64_t)s1;
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

    next.frame_counter = latest_state.frame_counter + 1;
    next.energy = energy > UINT32_MAX ? UINT32_MAX : (uint32_t)energy;

    if (energy >= DOA_ESTIMATOR_MIN_ENERGY && best_corr > 0) {
        int64_t gap = best_corr - (second_corr > 0 ? second_corr : 0);
        int64_t confidence = (gap * 255) / best_corr;

        next.sample_delay = (int16_t)best_lag;
        next.confidence = confidence > 255 ? 255 : (uint8_t)confidence;
        next.flags = DOA_ESTIMATOR_FLAG_VALID;
    }

    latest_state.sample_delay = next.sample_delay;
    latest_state.confidence = next.confidence;
    latest_state.flags = next.flags;
    latest_state.energy = next.energy;
    latest_state.frame_counter = next.frame_counter;
}

void doa_estimator_get_state(doa_estimator_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->sample_delay = latest_state.sample_delay;
    state->confidence = latest_state.confidence;
    state->flags = latest_state.flags;
    state->energy = latest_state.energy;
    state->frame_counter = latest_state.frame_counter;
}
