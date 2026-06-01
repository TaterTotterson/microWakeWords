// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* STD headers */
#include <string.h>
#include <stdint.h>
#include <xcore/hwtimer.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "stream_buffer.h"

/* Library headers */
#include "generic_pipeline.h"
#include "adec_api.h"

/* App headers */
#include "app_conf.h"
#include "audio_pipeline.h"
#include "audio_pipeline_dsp.h"
#include "doa/doa_estimator.h"
#include "platform/driver_instances.h"
#include "stage_1.h"

#if appconfAUDIO_PIPELINE_FRAME_ADVANCE != 240
#error This pipeline is only configured for 240 frame advance
#endif

#if ON_TILE(1)
// Stage1 - AEC, DE, ADEC
static stage_1_state_t DWORD_ALIGNED stage_1_state;
static aec_conf_t aec_de_mode_conf;
static aec_conf_t aec_non_de_mode_conf;
static adec_config_t adec_conf;

static void *audio_pipeline_input_i(void *input_app_data)
{
    frame_data_t *frame_data;

    frame_data = pvPortMalloc(sizeof(frame_data_t));
    memset(frame_data, 0x00, sizeof(frame_data_t));

    audio_pipeline_input(input_app_data,
                       (int32_t **)frame_data->aec_reference_audio_samples,
                       4,
                       appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    frame_data->vnr_pred_flag = 0;

    doa_estimator_process_frame(frame_data->mic_samples_passthrough[0],
                                frame_data->mic_samples_passthrough[1],
                                appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    memcpy(frame_data->samples, frame_data->mic_samples_passthrough, sizeof(frame_data->samples));

    return frame_data;
}

static int audio_pipeline_output_i(frame_data_t *frame_data,
                                   void *output_app_data)
{
    rtos_intertile_tx(intertile_ctx,
                      appconfAUDIOPIPELINE_PORT,
                      frame_data,
                      sizeof(frame_data_t));
    return AUDIO_PIPELINE_FREE_FRAME;
}

static void stage_aec(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_AEC
#else
    int32_t DWORD_ALIGNED stage_1_out[AEC_MAX_Y_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];

    stage_1_process_frame(&stage_1_state,
                          &stage_1_out[0],
                          &frame_data->max_ref_energy,
                          &frame_data->aec_corr_factor,
                          &frame_data->ref_active_flag,
                          frame_data->samples,
                          frame_data->aec_reference_audio_samples);

    memcpy(frame_data->samples, stage_1_out, AEC_MAX_Y_CHANNELS * appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#endif
}

static void initialize_pipeline_stages(void)
{
    aec_non_de_mode_conf.num_y_channels = 2;
    aec_non_de_mode_conf.num_x_channels = 2;
    aec_non_de_mode_conf.num_main_filt_phases = AEC_MAIN_FILTER_PHASES;
    aec_non_de_mode_conf.num_shadow_filt_phases = AEC_SHADOW_FILTER_PHASES;

    aec_de_mode_conf.num_y_channels = 1;
    aec_de_mode_conf.num_x_channels = 1;
    aec_de_mode_conf.num_main_filt_phases = 30;
    aec_de_mode_conf.num_shadow_filt_phases = 0;

    // Disable ADEC's automatic mode. We only want to estimate and correct for the delay at startup
    adec_conf.bypass = 1; // Bypass automatic DE correction
    adec_conf.force_de_cycle_trigger = 1; // Force a delay correction cycle, so that delay correction happens once after initialisation. Make sure this is set back to 0 after adec has requested a transition into DE mode once, to stop any further delay correction (automatic or forced) by ADEC
    stage_1_init(&stage_1_state, &aec_de_mode_conf, &aec_non_de_mode_conf, &adec_conf);
}

void audio_pipeline_init(
    void *input_app_data,
    void *output_app_data)
{
    const int stage_count = 1;

    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_aec,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_aec) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),

    };

    initialize_pipeline_stages();

    generic_pipeline_init((pipeline_input_t)audio_pipeline_input_i,
                        (pipeline_output_t)audio_pipeline_output_i,
                        input_app_data,
                        output_app_data,
                        stages,
                        (const size_t*) stage_stack_sizes,
                        appconfAUDIO_PIPELINE_TASK_PRIORITY,
                        stage_count);
}
#endif /* ON_TILE(1) */
