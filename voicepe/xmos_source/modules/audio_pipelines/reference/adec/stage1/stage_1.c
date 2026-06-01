// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include "audio_pipeline_dsp.h"
#include "stage_1.h"

extern void aec_process_frame_1thread(
        aec_state_t *main_state,
        aec_state_t *shadow_state,
        int32_t (*output_main)[AEC_FRAME_ADVANCE],
        int32_t (*output_shadow)[AEC_FRAME_ADVANCE],
        const int32_t (*y_data)[AEC_FRAME_ADVANCE],
        const int32_t (*x_data)[AEC_FRAME_ADVANCE]);

static void aec_switch_configuration(stage_1_state_t *state, aec_conf_t *conf)
{
    aec_init(&state->aec_main_state, &state->aec_shadow_state, &state->aec_shared_state,
            &state->aec_main_memory_pool[0], &state->aec_shadow_memory_pool[0],
            conf->num_y_channels, conf->num_x_channels,
            conf->num_main_filt_phases, conf->num_shadow_filt_phases);
}

static inline void get_delayed_frame(
        int32_t (*input_y_data)[AP_FRAME_ADVANCE],
        int32_t (*input_x_data)[AP_FRAME_ADVANCE],
        delay_buf_state_t *delay_state)
{
    int num_channels = (delay_state->delay_samples) > 0 ? AP_MAX_Y_CHANNELS : AP_MAX_X_CHANNELS;
    if (delay_state->delay_samples >= 0) {/** Requested Mic delay +ve => delay mic*/
        for(int ch=0; ch<num_channels; ch++) {
            for(int i=0; i<AP_FRAME_ADVANCE; i++) {
                get_delayed_sample(delay_state, &input_y_data[ch][i], ch);
            }
        }
    }
    else if (delay_state->delay_samples < 0) {/* Requested Mic delay negative => advance mic which can't be done, so delay reference*/
        for(int ch=0; ch<num_channels; ch++) {
            for(int i=0; i<AP_FRAME_ADVANCE; i++) {
                get_delayed_sample(delay_state, &input_x_data[ch][i], ch);
            }
        }
    }
    return;
}

void stage_1_init(stage_1_state_t *state, aec_conf_t *de_conf, aec_conf_t *non_de_conf, adec_config_t *adec_config) {
    state->delay_estimator_enabled = 0;
    state->ref_active_threshold =  f64_to_float_s32(pow(10, REF_ACTIVE_THRESHOLD_dB/20.0)); //-60dB
    state->hold_aec_count = 0; //No. of consecutive frames reference has been absent for
    state->hold_aec_limit = (16000*HOLD_AEC_LIMIT_SECONDS)/AP_FRAME_ADVANCE; //bypass AEC only when reference has been absent for atleast 3 seconds (200 frames)

    delay_buffer_init(&state->delay_state, 0/*Initialise with 0 delay_samples*/);
    memcpy(&state->aec_de_mode_conf, de_conf, sizeof(aec_conf_t));
    memcpy(&state->aec_non_de_mode_conf, non_de_conf, sizeof(aec_conf_t));

    adec_init(&state->adec_state, adec_config);
    aec_switch_configuration(state, &state->aec_non_de_mode_conf);
}

/** Process a frame of data through AEC and ADEC*/
static int framenum = 0;
void stage_1_process_frame(stage_1_state_t *state, int32_t (*output_frame)[AP_FRAME_ADVANCE],
    float_s32_t *max_ref_energy, float_s32_t *aec_corr_factor, int32_t *ref_active_flag,
    int32_t (*input_y)[AP_FRAME_ADVANCE], int32_t (*input_x)[AP_FRAME_ADVANCE])
{
    //printf("frame %d\n",framenum);
    framenum++;

    delay_buf_state_t *delay_state_ptr = &state->delay_state;
    get_delayed_frame(
            input_y,
            input_x,
            delay_state_ptr
            );

    /** Detect if there's activity on the reference channels*/
    *ref_active_flag = aec_detect_input_activity(input_x, state->ref_active_threshold, state->aec_main_state.shared_state->num_x_channels);

    /** AEC*/
    aec_process_frame_1thread(&state->aec_main_state, &state->aec_shadow_state, output_frame, NULL, input_y, input_x);

    /** Update metadata*/
    *max_ref_energy = aec_calc_max_input_energy(input_x, state->aec_main_state.shared_state->num_x_channels);
    for(int ch=0; ch<state->aec_main_state.shared_state->num_y_channels; ch++) {
        aec_corr_factor[ch] = aec_calc_corr_factor(&state->aec_main_state, ch);
    }

    /** Delay Estimation*/
    adec_input_t adec_in;
    adec_estimate_delay(
            &adec_in.from_de,
            state->aec_main_state.H_hat[0],
            state->aec_main_state.num_phases
            );


    /** ADEC*/
    // Create input to ADEC from AEC
    adec_in.from_aec.y_ema_energy_ch0 = state->aec_main_state.shared_state->y_ema_energy[0];
    adec_in.from_aec.error_ema_energy_ch0 = state->aec_main_state.error_ema_energy[0];
    adec_in.from_aec.shadow_flag_ch0 = state->aec_main_state.shared_state->shadow_filter_params.shadow_flag[0];
    // Directly from app
    adec_in.far_end_active_flag = *ref_active_flag;

    adec_output_t adec_output;
    adec_process_frame(
            &state->adec_state,
            &adec_output,
            &adec_in
            );

    //** Reset AEC state if needed*/
    if(adec_output.reset_aec_flag) {
        aec_reset_state(&state->aec_main_state, &state->aec_shadow_state);
    }

    /** Update delay buffer if there's a delay change requested by ADEC*/
    if(adec_output.delay_change_request_flag == 1){
        //printf("Frame %d: Set delay to %ld\n", framenum, adec_output.requested_mic_delay_samples);
        // Update delay_buffer delay_samples with mic delay requested by adec
        update_delay_samples(&state->delay_state, adec_output.requested_mic_delay_samples);
        for(int ch=0; ch<AP_MAX_Y_CHANNELS; ch++) {
            reset_partial_delay_buffer(&state->delay_state, ch);
        }
    }

    // Overwrite output with mic input if delay estimation enabled
    if (state->delay_estimator_enabled) {
        for(int ch=0; ch<AP_MAX_Y_CHANNELS; ch++) {
            memcpy(&output_frame[ch][0], &input_y[ch][0], AP_FRAME_ADVANCE*sizeof(int32_t)); // AEC cannot process the frame in-place because of this
        }
    }

    /** Switch AEC config if needed*/
    if (adec_output.delay_estimator_enabled_flag && !state->delay_estimator_enabled) {
        /** Now that a AEC -> DE change has been requested, reset force_de_cycle_trigger in case this transition is
         * requested as a result of force_de_cycle_trigger being set*/
        state->adec_state.adec_config.force_de_cycle_trigger = 0;

        // Initialise AEC for delay estimation config
        aec_switch_configuration(state, &state->aec_de_mode_conf);
        state->aec_main_state.shared_state->config_params.coh_mu_conf.adaption_config = AEC_ADAPTION_FORCE_ON;
        state->delay_estimator_enabled = 1;
        //printf("framenum %d: switch to de mode\n", framenum);
    } else if ((!adec_output.delay_estimator_enabled_flag && state->delay_estimator_enabled)) {
        // Start AEC for normal aec config
        aec_switch_configuration(state, &state->aec_non_de_mode_conf);
        state->delay_estimator_enabled = 0;
        //printf("framenum %d: switch to aec mode\n", framenum);

    }
}
