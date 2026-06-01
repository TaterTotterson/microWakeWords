// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef AEC_MEMORY_POOL_H
#define AEC_MEMORY_POOL_H

#include "audio_pipeline_dsp.h"
#include "xmath/xmath.h"

//Memory pool definition
typedef struct {
    /** Memory pointed to by aec_shared_state_t::y and aec_shared_state_t::Y*/
    int32_t mic_input_frame[AEC_MAX_Y_CHANNELS][AEC_PROC_FRAME_LENGTH + AEC_FFT_PADDING];
    /** Memory pointed to by aec_shared_state_t::x and aec_shared_state_t::X. Also reused for main filter
     * aec_state_t::T*/
    int32_t ref_input_frame[AEC_MAX_X_CHANNELS][AEC_PROC_FRAME_LENGTH + AEC_FFT_PADDING];
    /** Memory pointed to by aec_shared_state_t::prev_y*/
    int32_t mic_prev_samples[AEC_MAX_Y_CHANNELS][AEC_PROC_FRAME_LENGTH - AEC_FRAME_ADVANCE];
    /** Memory pointed to by aec_shared_state_t::prev_x*/
    int32_t ref_prev_samples[AEC_MAX_X_CHANNELS][AEC_PROC_FRAME_LENGTH - AEC_FRAME_ADVANCE];
    /** Memory pointed to by main filter aec_state_t::H_hat, aec_shared_state_t::X_fifo, main filter
     * aec_state_t::X_fifo_1d and shadow filter aec_state_t::X_fifo_1d*/
    complex_s32_t phase_pool_H_hat_X_fifo[((AEC_MAX_Y_CHANNELS*AEC_MAX_X_CHANNELS*AEC_MAIN_FILTER_PHASES) + (AEC_MAX_X_CHANNELS*AEC_MAIN_FILTER_PHASES)) * AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by main filter aec_state_t::Error and aec_state_t::error*/
    complex_s32_t Error[AEC_MAX_Y_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by main filter aec_state_t::Y_hat and aec_state_t::y_hat*/
    complex_s32_t Y_hat[AEC_MAX_Y_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by main_filter aec_state_t::X_energy*/
    int32_t X_energy[AEC_MAX_X_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by aec_shared_state_t::sigma_XX*/
    int32_t sigma_XX[AEC_MAX_X_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by main filter aec_state_t::inv_X_energy*/
    int32_t inv_X_energy[AEC_MAX_X_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by main filter aec_state_t::overlap*/
    int32_t overlap[AEC_MAX_Y_CHANNELS][AEC_UNUSED_TAPS_PER_PHASE*2];
}aec_memory_pool_t;

typedef struct {
    /** Memory pointed to by shadow filter aec_state_t::H_hat*/
    complex_s32_t phase_pool_H_hat[AEC_MAX_Y_CHANNELS * AEC_MAX_X_CHANNELS * AEC_SHADOW_FILTER_PHASES * AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by shadow filter aec_state_t::Error and aec_state_t::error*/
    complex_s32_t Error[AEC_MAX_Y_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by shadow filter aec_state_t::Y_hat and aec_state_t::y_hat*/
    complex_s32_t Y_hat[AEC_MAX_Y_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by shadow filter aec_state_t::T*/
    complex_s32_t T[AEC_MAX_X_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by shadow_filter aec_state_t::X_energy*/
    int32_t X_energy[AEC_MAX_X_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by shadow_filter aec_state_t::inv_X_energy*/
    int32_t inv_X_energy[AEC_MAX_X_CHANNELS][AEC_FD_FRAME_LENGTH];
    /** Memory pointed to by shadow filter aec_state_t::overlap*/
    int32_t overlap[AEC_MAX_Y_CHANNELS][AEC_UNUSED_TAPS_PER_PHASE*2];
}aec_shadow_filt_memory_pool_t;
#endif
