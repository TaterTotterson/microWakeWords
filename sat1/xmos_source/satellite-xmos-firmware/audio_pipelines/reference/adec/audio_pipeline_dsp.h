// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef AUDIO_PIPELINE_DSP_H_
#define AUDIO_PIPELINE_DSP_H_

#include <stdint.h>
#include "app_conf.h"

/* Pipeline config */
#define AP_MAX_Y_CHANNELS (2)
#define AP_MAX_X_CHANNELS (2)
#define AP_FRAME_ADVANCE (240)

/* AEC config */
#define AEC_MAX_Y_CHANNELS   (AP_MAX_Y_CHANNELS)
#define AEC_MAX_X_CHANNELS   (AP_MAX_X_CHANNELS)
#define AEC_MAIN_FILTER_PHASES    (10)
#define AEC_SHADOW_FILTER_PHASES    (5)

/* Delay buffer config */
#define MAX_DELAY_BUF_CHANNELS (2)
#define DELAY_BUF_MAX_DELAY_MS                ( 150 )
#define DELAY_BUF_MAX_DELAY_SAMPLES           ( 16000*DELAY_BUF_MAX_DELAY_MS/1000 )

#include "aec_api.h"
#include "aec/aec_memory_pool.h"
#include "agc_api.h"
#include "ic_api.h"
#include "ns_api.h"
#include "vnr_features_api.h"
#include "vnr_inference_api.h"
#include "adec_api.h"

/* Note: Changing the order here will effect the channel order for
 * audio_pipeline_input() and audio_pipeline_output()
 */
typedef struct {
    int32_t samples[appconfAUDIO_PIPELINE_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    int32_t aec_reference_audio_samples[appconfAUDIO_PIPELINE_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    int32_t mic_samples_passthrough[appconfAUDIO_PIPELINE_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];

    /* Below is additional context needed by other stages on a per frame basis */
    int32_t vnr_pred_flag;
    float_s32_t max_ref_energy;
    float_s32_t aec_corr_factor;
    int32_t ref_active_flag;
} frame_data_t;

typedef struct aec_ctx {
    aec_state_t DWORD_ALIGNED aec_main_state;
    aec_state_t DWORD_ALIGNED aec_shadow_state;
    aec_shared_state_t DWORD_ALIGNED aec_shared_state;
    uint8_t DWORD_ALIGNED aec_main_memory_pool[sizeof(aec_memory_pool_t)];
    uint8_t DWORD_ALIGNED aec_shadow_memory_pool[sizeof(aec_shadow_filt_memory_pool_t)];
} aec_ctx_t;

typedef struct ic_stage_ctx {
    ic_state_t DWORD_ALIGNED state;
} ic_stage_ctx_t;

typedef struct vnr_pred_stage_ctx {
    vnr_pred_state_t vnr_pred_state; 
} vnr_pred_stage_ctx_t;

typedef struct ns_stage_ctx {
    ns_state_t DWORD_ALIGNED state;
} ns_stage_ctx_t;

typedef struct agc_stage_ctx {
    agc_meta_data_t md;
    agc_state_t DWORD_ALIGNED state;
} agc_stage_ctx_t;

#endif /* AUDIO_PIPELINE_DSP_H_ */
