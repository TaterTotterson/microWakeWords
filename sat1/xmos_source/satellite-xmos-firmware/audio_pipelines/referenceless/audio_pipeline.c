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

/* Library headers */
#include "generic_pipeline.h"
#include "agc_api.h"
#include "ic_api.h"
#include "ns_api.h"
#include "vnr_features_api.h"
#include "vnr_inference_api.h"

/* App headers */
#include "app_conf.h"
#include "audio_pipeline.h"

#define VNR_AGC_THRESHOLD              (0.5)
#define EMA_ENERGY_ALPHA               (0.25)
#define IC_INPUT_VNR_THRESHOLD         (0.5)
#define IC_INPUT_VNR_THRESHOLD_HIGH    (0.75)

/* Note: Changing the order here will effect the channel order for
 * audio_pipeline_input() and audio_pipeline_output()
 */
typedef struct {
    int32_t samples[appconfAUDIO_PIPELINE_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    float_s32_t input_vnr_pred;
    float_s32_t output_vnr_pred;
    control_flag_e control_flag;
} frame_data_t;

#if appconfAUDIO_PIPELINE_FRAME_ADVANCE != 240
#error This pipeline is only configured for 240 frame advance
#endif

typedef struct ic_stage_ctx {
    ic_state_t state;
} ic_stage_ctx_t;

typedef struct vnr_pred_stage_ctx {
    vnr_pred_state_t vnr_pred_state;
} vnr_pred_stage_ctx_t;

typedef struct ns_stage_ctx {
    ns_state_t state;
} ns_stage_ctx_t;

typedef struct agc_stage_ctx {
    agc_meta_data_t md;
    agc_state_t state;
} agc_stage_ctx_t;

static ic_stage_ctx_t DWORD_ALIGNED ic_stage_state = {};
static vnr_pred_stage_ctx_t DWORD_ALIGNED vnr_pred_stage_state = {};
static ns_stage_ctx_t DWORD_ALIGNED ns_stage_state = {};
static agc_stage_ctx_t DWORD_ALIGNED agc_stage_state = {};

static trace_data_t* trace_data = 0;

static void *audio_pipeline_input_i(void *input_app_data)
{
    frame_data_t *frame_data;

    frame_data = pvPortMalloc(sizeof(frame_data_t));
    memset(frame_data, 0x00, sizeof(frame_data_t));

    audio_pipeline_input(input_app_data,
                       (int32_t **)frame_data->samples,
                       2,
                       appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    frame_data->input_vnr_pred = f32_to_float_s32(0.0);
    frame_data->output_vnr_pred = f32_to_float_s32(0.0);
    frame_data->control_flag = ADAPT;

    return frame_data;
}

static int audio_pipeline_output_i(frame_data_t *frame_data,
                                   void *output_app_data)
{
    if (trace_data) {
        assert(trace_data == output_app_data);
        trace_data->input_vnr_pred = float_s32_to_float(frame_data->input_vnr_pred);
        trace_data->control_flag = (int)frame_data->control_flag;
    }

    return audio_pipeline_output(output_app_data,
                               (int32_t **)frame_data->samples,
                               4,
                               appconfAUDIO_PIPELINE_FRAME_ADVANCE);
}

static void stage_vnr_and_ic(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR
    (void) frame_data;
#else

    int32_t DWORD_ALIGNED ic_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];

    ic_filter(&ic_stage_state.state,
              frame_data->samples[0],
              frame_data->samples[1],
              ic_output);

    // VNR
    vnr_pred_state_t *vnr_pred_state = &vnr_pred_stage_state.vnr_pred_state;
    ic_calc_vnr_pred(&ic_stage_state.state, &vnr_pred_state->input_vnr_pred, &vnr_pred_state->output_vnr_pred);

    ic_adapt(&ic_stage_state.state, vnr_pred_stage_state.vnr_pred_state.input_vnr_pred);

    frame_data->input_vnr_pred = vnr_pred_stage_state.vnr_pred_state.input_vnr_pred;
    frame_data->output_vnr_pred = vnr_pred_stage_state.vnr_pred_state.output_vnr_pred;
    frame_data->control_flag = ic_stage_state.state.ic_adaption_controller_state.control_flag;

    memcpy(frame_data->samples, ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#endif
}

static void stage_ns(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_NS
    (void) frame_data;
#else
    int32_t DWORD_ALIGNED ns_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    configASSERT(NS_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    ns_process_frame(
                &ns_stage_state.state,
                ns_output,
                frame_data->samples[0]);
    memcpy(frame_data->samples, ns_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#endif
}

static void stage_agc(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_AGC
    (void) frame_data;
#else
    int32_t DWORD_ALIGNED agc_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    configASSERT(AGC_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    agc_stage_state.md.vnr_flag = float_s32_gt(frame_data->output_vnr_pred, f32_to_float_s32(VNR_AGC_THRESHOLD));

    agc_process_frame(
            &agc_stage_state.state,
            agc_output,
            frame_data->samples[0],
            &agc_stage_state.md);
    memcpy(frame_data->samples, agc_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#endif
}

static void initialize_pipeline_stages(void) {
    ic_init(&ic_stage_state.state);

    // Set some VNR parameters
    ic_stage_state.state.ic_adaption_controller_state.adaption_controller_config.input_vnr_threshold = 
        f64_to_float_s32(IC_INPUT_VNR_THRESHOLD);
    ic_stage_state.state.ic_adaption_controller_state.adaption_controller_config.input_vnr_threshold_high = 
        f64_to_float_s32(IC_INPUT_VNR_THRESHOLD_HIGH);

    ns_init(&ns_stage_state.state);

    agc_init(&agc_stage_state.state, &AGC_PROFILE_ASR);
    agc_stage_state.md.aec_ref_power = AGC_META_DATA_NO_AEC;
    agc_stage_state.md.aec_corr_factor = AGC_META_DATA_NO_AEC;
}

void audio_pipeline_init(
    void *input_app_data,
    void *output_app_data)
{
    const int stage_count = 3;

    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_vnr_and_ic,
        (pipeline_stage_t)stage_ns,
        (pipeline_stage_t)stage_agc,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_vnr_and_ic) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_ns),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_agc) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

    initialize_pipeline_stages();

    trace_data = (trace_data_t *) output_app_data;
    generic_pipeline_init((pipeline_input_t)audio_pipeline_input_i,
                        (pipeline_output_t)audio_pipeline_output_i,
                        input_app_data,
                        output_app_data,
                        stages,
                        (const size_t*) stage_stack_sizes,
                        appconfAUDIO_PIPELINE_TASK_PRIORITY,
                        stage_count);
}
