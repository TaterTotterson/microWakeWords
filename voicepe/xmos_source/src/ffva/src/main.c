// Copyright 2020-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <platform.h>
#include <xs1.h>
#include <xcore/channel.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "queue.h"

/* Library headers */
#include "rtos_printf.h"
#include "src.h"

/* App headers */
#include "app_conf.h"
#include "platform/platform_init.h"
#include "platform/driver_instances.h"
#include "platform/platform_conf.h"
#include "usb_support.h"
// #include "usb_audio.h"
#include "audio_pipeline.h"
// #include "ww_model_runner/ww_model_runner.h"
// #include "fs_support.h"
#include "dfu_servicer.h"
#include "configuration_servicer.h"
#include "doa/doa_servicer.h"

#include "gpio_test/gpio_test.h"

volatile int mic_from_usb = appconfMIC_SRC_DEFAULT;
volatile int aec_ref_source = appconfAEC_REF_DEFAULT;

#if appconfI2S_ENABLED && (appconfI2S_MODE == appconfI2S_MODE_SLAVE)
void i2s_slave_intertile(void *args) {
    (void) args;
    int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];

    while(1) {
        memset(tmp, 0x00, sizeof(tmp));

        size_t bytes_received = 0;
        bytes_received = rtos_intertile_rx_len(
                intertile_ctx,
                appconfI2S_OUTPUT_SLAVE_PORT,
                portMAX_DELAY);

        xassert(bytes_received == sizeof(tmp));

        rtos_intertile_rx_data(
                intertile_ctx,
                tmp,
                bytes_received);

        rtos_i2s_tx(i2s1_ctx,
                    (int32_t*) tmp,
                    appconfAUDIO_PIPELINE_FRAME_ADVANCE,
                    portMAX_DELAY);
    }
}
#endif

void audio_pipeline_input(void *input_app_data,
                        int32_t **input_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
    (void) input_app_data;
    int32_t **mic_ptr = (int32_t **)(input_audio_frames + (2 * frame_count));

    static int flushed;
    while (!flushed) {
        size_t received;
        received = rtos_mic_array_rx(mic_array_ctx,
                                     mic_ptr,
                                     frame_count,
                                     0);
        if (received == 0) {
            rtos_mic_array_rx(mic_array_ctx,
                              mic_ptr,
                              frame_count,
                              portMAX_DELAY);
            flushed = 1;
        }
    }

    /*
     * NOTE: ALWAYS receive the next frame from the PDM mics,
     * even if USB is the current mic source. The controls the
     * timing since usb_audio_recv() does not block and will
     * receive all zeros if no frame is available yet.
     */
    rtos_mic_array_rx(mic_array_ctx,
                      mic_ptr,
                      frame_count,
                      portMAX_DELAY);

#if appconfUSB_ENABLED
    int32_t **usb_mic_audio_frame = NULL;
    size_t ch_cnt = 2;  /* ref frames */

    if (aec_ref_source == appconfAEC_REF_USB) {
        usb_mic_audio_frame = input_audio_frames;
    }

    if (mic_from_usb) {
        ch_cnt += 2;  /* mic frames */
    }

    /*
     * As noted above, this does not block.
     * and expects ref L, ref R, mic 0, mic 1
     */
    usb_audio_recv(intertile_usb_audio_ctx,
                   frame_count,
                   usb_mic_audio_frame,
                   ch_cnt);
#endif

#if appconfI2S_ENABLED
    if (!appconfUSB_ENABLED || aec_ref_source == appconfAEC_REF_I2S) {
        /* This shouldn't need to block given it shares a clock with the PDM mics */

        xassert(frame_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        /* I2S provides sample channel format */
        int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];
        int32_t *tmpptr = (int32_t *)input_audio_frames;

        size_t rx_count =
        rtos_i2s_rx(i2s1_ctx,
                    (int32_t*) tmp,
                    frame_count,
                    portMAX_DELAY);
        xassert(rx_count == frame_count);

        for (int i=0; i<frame_count; i++) {
            /* ref is first */
            *(tmpptr + i) = tmp[i][0];
            *(tmpptr + i + frame_count) = tmp[i][1];
        }
    }
#endif
}

int audio_pipeline_output(void *output_app_data,
                        int32_t **output_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
    (void) output_app_data;

#if appconfI2S_ENABLED
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
#if !appconfI2S_TDM_ENABLED
    xassert(frame_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    /* I2S expects sample channel format */
    int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int j=0; j<frame_count; j++) {
        /* ASR output is first */
        // tmp[j][0] = *(tmpptr+j);                    // proc 0, AGC audio
        // tmp[j][1] = *(tmpptr+j+(1*frame_count))     // proc 1, mic 1 audio with AEC applied
        // tmp[j][0] = *(tmpptr+j+(2*frame_count));    // ref 0, overwritten by IC audio
        // tmp[j][1] = *(tmpptr+j+(3*frame_count));    // ref 1, overwritten by NS audio
        // tmp[j][0] = *(tmpptr+j+(4*frame_count));    // mic 0
        // tmp[j][1] = *(tmpptr+j+(5*frame_count));    // mic 1

        enum e_pipeline_processing_stages channel_0_stage = configuration_get_channel_0_stage();
        if (channel_0_stage == PIPELINE_STAGE_NONE) {
            // Original microphone audio
            tmp[j][0] = *(tmpptr + j + (4 * frame_count));
        } else if (channel_0_stage == PIPELINE_STAGE_AEC) {
            // AEC cancelled audio
            tmp[j][0] = *(tmpptr + j + (1 * frame_count)); 
        } else if (channel_0_stage == PIPELINE_STAGE_IC) {
            // AEC+IC processed audio
            tmp[j][0] = *(tmpptr + j + (2 * frame_count));
        } else if (channel_0_stage == PIPELINE_STAGE_NS) {
            // AEC+IC+NS processed audio
            tmp[j][0] = *(tmpptr + j + (3 * frame_count));
        } else if (channel_0_stage == PIPELINE_STAGE_AGC) {
            // AEC+IC+NS+AGC processed audio
            tmp[j][0] = *(tmpptr + j);
        }

        enum e_pipeline_processing_stages channel_1_stage = configuration_get_channel_1_stage();
        if (channel_1_stage == PIPELINE_STAGE_NONE) {
            // Original microphone audio
            tmp[j][1] = *(tmpptr + j + (5 * frame_count));
        } else if (channel_1_stage == PIPELINE_STAGE_AEC) {
            // AEC cancelled audio
            tmp[j][1] = *(tmpptr + j + (1 * frame_count)); 
        } else if (channel_1_stage == PIPELINE_STAGE_IC) {
            // AEC+IC processed audio
            tmp[j][1] = *(tmpptr + j + (2 * frame_count));
        } else if (channel_1_stage == PIPELINE_STAGE_NS) {
            // AEC+IC+NS processed audio
            tmp[j][1] = *(tmpptr + j + (3 * frame_count));
        } else if (channel_1_stage == PIPELINE_STAGE_AGC) {
            // AEC+IC+NS+AGC processed audio
            tmp[j][1] = *(tmpptr + j);
        }
    }

    // // TEST(jerry): output mic data on i2s1
    // rtos_i2s_tx(i2s1_ctx,
    //             (int32_t*) tmp,
    //             frame_count,
    //             portMAX_DELAY);
    
    // TEST(jerry): output mic data on i2s2
    rtos_i2s_tx(i2s2_ctx,
                (int32_t*) tmp,
                frame_count,
                portMAX_DELAY);
#else
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int i = 0; i < frame_count; i++) {
        /* output_audio_frames format is
         *   processed_audio_frame
         *   reference_audio_frame
         *   raw_mic_audio_frame
         */
        int32_t tdm_output[6];

        tdm_output[0] = *(tmpptr + i + (4 * frame_count)) & ~0x1;   // mic 0
        tdm_output[1] = *(tmpptr + i + (5 * frame_count)) & ~0x1;   // mic 1
        tdm_output[2] = *(tmpptr + i + (2 * frame_count)) & ~0x1;   // ref 0
        tdm_output[3] = *(tmpptr + i + (3 * frame_count)) & ~0x1;   // ref 1
        tdm_output[4] = *(tmpptr + i) | 0x1;                        // proc 0
        tdm_output[5] = *(tmpptr + i + frame_count) | 0x1;          // proc 1

        rtos_i2s_tx(i2s1_ctx,
                    tdm_output,
                    appconfI2S_AUDIO_SAMPLE_RATE / appconfAUDIO_PIPELINE_SAMPLE_RATE,
                    portMAX_DELAY);
    }
#endif
#elif appconfI2S_MODE == appconfI2S_MODE_SLAVE
    /* I2S expects sample channel format */
    int32_t tmp[appconfAUDIO_PIPELINE_FRAME_ADVANCE][appconfAUDIO_PIPELINE_CHANNELS];
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int j=0; j<appconfAUDIO_PIPELINE_FRAME_ADVANCE; j++) {
        /* ASR output is first */
        tmp[j][0] = *(tmpptr+j);
        tmp[j][1] = *(tmpptr+j+appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    }

    rtos_intertile_tx(intertile_ctx,
                      appconfI2S_OUTPUT_SLAVE_PORT,
                      tmp,
                      sizeof(tmp));
#endif
#endif

#if appconfUSB_ENABLED
    usb_audio_send(intertile_usb_audio_ctx,
                frame_count,
                output_audio_frames,
                6);
#endif

#if appconfWW_ENABLED
    ww_audio_send(intertile_ctx,
                  frame_count,
                  (int32_t(*)[2])output_audio_frames);
#endif

    return AUDIO_PIPELINE_FREE_FRAME;
}

RTOS_I2S_APP_SEND_FILTER_CALLBACK_ATTR
size_t i2s_send_upsample_cb(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *send_buf, size_t samples_available)
{
    static int i;
    static int32_t src_data[2][SRC_FF3V_FIR_TAPS_PER_PHASE] __attribute__((aligned(8)));

    xassert(i2s_frame_size == 2);

    i2s_frame[0] = send_buf[0];
    i2s_frame[1] = send_buf[1];

    switch (i) {
    case 0:
        i = 1;
        if (samples_available >= 2) {
            // i2s_frame[0] = src_us3_voice_input_sample(src_data[0], src_ff3v_fir_coefs[2], send_buf[0]);
            // i2s_frame[1] = src_us3_voice_input_sample(src_data[1], src_ff3v_fir_coefs[2], send_buf[1]);
            return 2;
        } else {
            // i2s_frame[0] = src_us3_voice_input_sample(src_data[0], src_ff3v_fir_coefs[2], 0);
            // i2s_frame[1] = src_us3_voice_input_sample(src_data[1], src_ff3v_fir_coefs[2], 0);
            return 0;
        }
    case 1:
        i = 2;
        // i2s_frame[0] = src_us3_voice_get_next_sample(src_data[0], src_ff3v_fir_coefs[1]);
        // i2s_frame[1] = src_us3_voice_get_next_sample(src_data[1], src_ff3v_fir_coefs[1]);
        return 0;
    case 2:
        i = 0;
        // i2s_frame[0] = src_us3_voice_get_next_sample(src_data[0], src_ff3v_fir_coefs[0]);
        // i2s_frame[1] = src_us3_voice_get_next_sample(src_data[1], src_ff3v_fir_coefs[0]);
        return 0;
    default:
        xassert(0);
        return 0;
    }
}

RTOS_I2S_APP_RECEIVE_FILTER_CALLBACK_ATTR
size_t i2s_send_downsample_cb(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *receive_buf, size_t sample_spaces_free)
{
    static int i;
    static int64_t sum[2];
    static int32_t src_data[2][SRC_FF3V_FIR_NUM_PHASES][SRC_FF3V_FIR_TAPS_PER_PHASE] __attribute__((aligned (8)));

    xassert(i2s_frame_size == 2);

    switch (i) {
    case 0:
        i = 1;
        sum[0] = src_ds3_voice_add_sample(0, src_data[0][0], src_ff3v_fir_coefs[0], i2s_frame[0]);
        sum[1] = src_ds3_voice_add_sample(0, src_data[1][0], src_ff3v_fir_coefs[0], i2s_frame[1]);
        return 0;
    case 1:
        i = 2;
        sum[0] = src_ds3_voice_add_sample(sum[0], src_data[0][1], src_ff3v_fir_coefs[1], i2s_frame[0]);
        sum[1] = src_ds3_voice_add_sample(sum[1], src_data[1][1], src_ff3v_fir_coefs[1], i2s_frame[1]);
        return 0;
    case 2:
        i = 0;
        if (sample_spaces_free >= 2) {
            receive_buf[0] = src_ds3_voice_add_final_sample(sum[0], src_data[0][2], src_ff3v_fir_coefs[2], i2s_frame[0]);
            receive_buf[1] = src_ds3_voice_add_final_sample(sum[1], src_data[1][2], src_ff3v_fir_coefs[2], i2s_frame[1]);
            return 2;
        } else {
            (void) src_ds3_voice_add_final_sample(sum[0], src_data[0][2], src_ff3v_fir_coefs[2], i2s_frame[0]);
            (void) src_ds3_voice_add_final_sample(sum[1], src_data[1][2], src_ff3v_fir_coefs[2], i2s_frame[1]);
            return 0;
        }
    default:
        xassert(0);
        return 0;
    }
}

void i2s_rate_conversion_enable(void)
{
#if !appconfI2S_TDM_ENABLED
    rtos_i2s_send_filter_cb_set(i2s1_ctx, i2s_send_upsample_cb, NULL);
#endif
    rtos_i2s_receive_filter_cb_set(i2s1_ctx, i2s_send_downsample_cb, NULL);
}

void i2s2_rate_conversion_enable(void)
{
#if !appconfI2S_TDM_ENABLED
    rtos_i2s_send_filter_cb_set(i2s2_ctx, i2s_send_upsample_cb, NULL);
#endif
    rtos_i2s_receive_filter_cb_set(i2s2_ctx, i2s_send_downsample_cb, NULL);
}

void vApplicationMallocFailedHook(void)
{
    rtos_printf("Malloc Failed on tile %d!\n", THIS_XCORE_TILE);
    xassert(0);
    for(;;);
}

static void mem_analysis(void)
{
	for (;;) {
		rtos_printf("Tile[%d]:\n\tMinimum heap free: %d\n\tCurrent heap free: %d\n", THIS_XCORE_TILE, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

void startup_task(void *arg)
{
    rtos_printf("Startup task running from tile %d on core %d\n", THIS_XCORE_TILE, portGET_CORE_ID());

    platform_start();

#if ON_TILE(1) && appconfI2S_ENABLED && (appconfI2S_MODE == appconfI2S_MODE_SLAVE)
    xTaskCreate((TaskFunction_t) i2s_slave_intertile,
                "i2s_slave_intertile",
                RTOS_THREAD_STACK_SIZE(i2s_slave_intertile),
                NULL,
                appconfAUDIO_PIPELINE_TASK_PRIORITY,
                NULL);
#endif

#if appconfI2C_DFU_ENABLED && ON_TILE(I2C_CTRL_TILE_NO)
    servicer_t servicer_cfg;
    configuration_servicer_init(&servicer_cfg);

    xTaskCreate(
        configuration_servicer,
        "CFG servicer",
        RTOS_THREAD_STACK_SIZE(configuration_servicer),
        &servicer_cfg,
        appconfDEVICE_CONTROL_I2C_PRIORITY,
        NULL
    );

    // Initialise control related things
    servicer_t servicer_dfu;
    dfu_servicer_init(&servicer_dfu);

    xTaskCreate(
        dfu_servicer,
        "DFU servicer",
        RTOS_THREAD_STACK_SIZE(dfu_servicer),
        &servicer_dfu,
        appconfDEVICE_CONTROL_I2C_PRIORITY,
        NULL
    );
#endif

#if appconfI2C_DFU_ENABLED && ON_TILE(MICARRAY_TILE_NO)
    servicer_t servicer_doa;
    doa_servicer_init(&servicer_doa);

    xTaskCreate(
        doa_servicer,
        "DoA servicer",
        RTOS_THREAD_STACK_SIZE(doa_servicer),
        &servicer_doa,
        appconfDEVICE_CONTROL_I2C_PRIORITY,
        NULL
    );
#endif

#if appconfINTENT_ENABLED && ON_TILE(FS_TILE_NO)
    rtos_fatfs_init(qspi_flash_ctx);
    // rtos_dfu_image_print_debug(dfu_image_ctx);
    // Setup flash low-level mode
    //   NOTE: must call rtos_qspi_flash_fast_read_shutdown_ll to use non low-level mode calls
    rtos_qspi_flash_fast_read_setup_ll(qspi_flash_ctx);
#endif

#if appconfWW_ENABLED && ON_TILE(WW_TILE_NO)
    ww_task_create(appconfWW_TASK_PRIORITY);
#endif

    audio_pipeline_init(NULL, NULL);

    mem_analysis();
}

void vApplicationMinimalIdleHook(void)
{
    rtos_printf("idle hook on tile %d core %d\n", THIS_XCORE_TILE, rtos_core_id_get());
    asm volatile("waiteu");
}

static void tile_common_init(chanend_t c)
{
    platform_init(c);
    chanend_free(c);

#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    usb_audio_init(intertile_usb_audio_ctx, appconfUSB_AUDIO_TASK_PRIORITY);
#endif

    xTaskCreate((TaskFunction_t) startup_task,
                "startup_task",
                RTOS_THREAD_STACK_SIZE(startup_task),
                NULL,
                appconfSTARTUP_TASK_PRIORITY,
                NULL);

    rtos_printf("start scheduler on tile %d\n", THIS_XCORE_TILE);
    vTaskStartScheduler();
}

#if ON_TILE(0)
void main_tile0(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
{
    (void) c0;
    (void) c2;
    (void) c3;

    tile_common_init(c1);
}
#endif

#if ON_TILE(1)
void main_tile1(chanend_t c0, chanend_t c1, chanend_t c2, chanend_t c3)
{
    (void) c1;
    (void) c2;
    (void) c3;

    tile_common_init(c0);
}
#endif
