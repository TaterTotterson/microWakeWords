// Copyright 2020-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <platform.h>
#include <xs1.h>
#include <xcore/channel.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

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
#include "usb_audio.h"
#include "usb_cdc.h"
#include "audio_pipeline.h"
#include "speaker_pipeline.h"
#include "dfu_servicer.h"
#include "gpio/gpio_servicer.h"
#include "led_ring/led_ring_servicer.h"
#include "doa/doa_estimator.h"
#include "doa/doa_servicer.h"


/* Config headers for sw_pll */
#include "sw_pll.h"

volatile int mic_from_usb = appconfMIC_SRC_DEFAULT;
volatile int aec_ref_source = appconfAEC_REF_DEFAULT;

#define APP_I2S_STEREO_CHANNELS 2


#if ON_TILE(SPEAKER_PIPELINE_TILE_NO)
rtos_osal_queue_t *ref_input_queue;
#endif

static int32_t saturate_i32_from_i64(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

#if MIC_ARRAY_CONFIG_MIC_COUNT >= 4 && appconfAUDIO_PIPELINE_CHANNELS >= 2
static void mix_four_mics_for_pipeline(
        int32_t *pipeline_mic_ptr,
        const int32_t full_mic_frames[MIC_ARRAY_CONFIG_MIC_COUNT][MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME],
        size_t frame_count)
{
    for (size_t i = 0; i < frame_count; i++) {
        int64_t east = full_mic_frames[0][i];
        int64_t west = full_mic_frames[1][i];
        int64_t north = full_mic_frames[2][i];
        int64_t south = full_mic_frames[3][i];

        int64_t primary = (east + west + north + south) / 4;
        int64_t spatial_ref = ((east - west) + (north - south)) / 4;
        int64_t speech_ref = primary + (spatial_ref / 4);

        pipeline_mic_ptr[i] = saturate_i32_from_i64(primary);
        pipeline_mic_ptr[i + frame_count] = saturate_i32_from_i64(speech_ref);
    }
}
#endif

void speaker_pipeline_input(void *input_app_data,
                        int32_t **input_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
#if ON_TILE(SPEAKER_PIPELINE_TILE_NO)
    if (!appconfUSB_AUDIO_ENABLED || aec_ref_source == appconfAEC_REF_I2S) {
        /* This shouldn't need to block given it shares a clock with the PDM mics */

        xassert(frame_count == appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE);
        /* I2S provides sample channel format */
        int32_t tmp[appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE][appconfI2S_AUDIO_INPUTS][APP_I2S_STEREO_CHANNELS];
        int32_t *tmpptr = (int32_t *)input_audio_frames;

        size_t rx_count =
        rtos_i2s_rx(i2s_ctx,
                    (int32_t*) tmp,
                    frame_count,
                    portMAX_DELAY);
        xassert(rx_count == frame_count);

        for (int i=0; i<frame_count; i++) {
            /* ref is first */
            *(tmpptr + i) = tmp[i][0][0];
            *(tmpptr + i + frame_count) = tmp[i][0][1];
        }
    }

#if appconfUSB_AUDIO_ENABLED
    int32_t **usb_mic_audio_frame = NULL;
    if (true) {
        usb_mic_audio_frame = input_audio_frames;
        /*
        * As noted above, this does not block.
        * and expects ref L, ref R, mic 0, mic 1
        */
        usb_audio_recv(intertile_usb_audio_ctx,
            frame_count,
            usb_mic_audio_frame,
            ch_count);
    }
#endif    
#endif
}

int speaker_pipeline_output(void *output_app_data,
                        int32_t **output_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
#if ON_TILE(SPEAKER_PIPELINE_TILE_NO)    
    (void) output_app_data;

    xassert(frame_count == appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE);
    
    /* I2S expects sample channel format */
    int32_t tmp[appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE][1][APP_I2S_STEREO_CHANNELS];
    int32_t *tmpptr = (int32_t *)output_audio_frames;
    for (int j=0; j<frame_count; j++) {
        tmp[j][0][0] = *(tmpptr+j+(0*frame_count));    // ref 0 -> DAC
        tmp[j][0][1] = *(tmpptr+j+(1*frame_count));    // ref 1 -> DAC
    }
    
    // send to DAC
    rtos_i2s_tx_1(i2s_ctx,
                (int32_t*) tmp,
                frame_count,
                portMAX_DELAY);
    
    void* frame_data;
    frame_data = pvPortMalloc( appconfAUDIO_PIPELINE_FRAME_ADVANCE * APP_I2S_STEREO_CHANNELS * sizeof( int32_t ));
    
    // down sample reference signal to 16kHz if needed 
    if (appconfI2S_AUDIO_SAMPLE_RATE == 3*appconfAUDIO_PIPELINE_SAMPLE_RATE) {
        static int64_t sum[2];
        static int32_t src_data[2][SRC_FF3V_FIR_NUM_PHASES][SRC_FF3V_FIR_TAPS_PER_PHASE] __attribute__((aligned (8)));
        int32_t tmp_out[appconfAUDIO_PIPELINE_FRAME_ADVANCE][1][APP_I2S_STEREO_CHANNELS];
        
        for( int frame=0; frame < frame_count; frame +=3 ){
            sum[0] = src_ds3_voice_add_sample(0, src_data[0][0], src_ff3v_fir_coefs[0], tmp[frame][0][0]);
            sum[1] = src_ds3_voice_add_sample(0, src_data[1][0], src_ff3v_fir_coefs[0], tmp[frame][0][1]);

            sum[0] = src_ds3_voice_add_sample(sum[0], src_data[0][1], src_ff3v_fir_coefs[1], tmp[frame+1][0][0]);
            sum[1] = src_ds3_voice_add_sample(sum[1], src_data[1][1], src_ff3v_fir_coefs[1], tmp[frame+1][0][1]);

            tmp_out[frame/3][0][0] = src_ds3_voice_add_final_sample(sum[0], src_data[0][2], src_ff3v_fir_coefs[2], tmp[frame+2][0][0]);
            tmp_out[frame/3][0][1] = src_ds3_voice_add_final_sample(sum[1], src_data[1][2], src_ff3v_fir_coefs[2], tmp[frame+2][0][1]);
        }
        memcpy( frame_data, tmp_out, appconfAUDIO_PIPELINE_FRAME_ADVANCE * APP_I2S_STEREO_CHANNELS * sizeof( int32_t ) );
    } else {
      memcpy( frame_data, tmp, appconfAUDIO_PIPELINE_FRAME_ADVANCE * APP_I2S_STEREO_CHANNELS * sizeof( int32_t ) );
    }
    
    // send to microphone pipeline as reference
    (void) rtos_osal_queue_send(ref_input_queue, &frame_data, RTOS_OSAL_WAIT_FOREVER);

#endif
    return AUDIO_PIPELINE_FREE_FRAME;
}


static size_t receive_mic_array_frame(int32_t **pipeline_mic_frames,
                                      size_t frame_count,
                                      unsigned timeout)
{
#if MIC_ARRAY_CONFIG_MIC_COUNT > appconfAUDIO_PIPELINE_CHANNELS
    static int32_t full_mic_frames[MIC_ARRAY_CONFIG_MIC_COUNT][MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME] __attribute__((aligned(8)));
    size_t received = rtos_mic_array_rx(mic_array_ctx,
                                        (int32_t **)full_mic_frames,
                                        frame_count,
                                        timeout);
    if (received == frame_count) {
        int32_t *pipeline_mic_ptr = (int32_t *)pipeline_mic_frames;
#if MIC_ARRAY_CONFIG_MIC_COUNT >= 4 && appconfAUDIO_PIPELINE_CHANNELS >= 2
        mix_four_mics_for_pipeline(pipeline_mic_ptr, full_mic_frames, frame_count);
#else
        for (int ch = 0; ch < appconfAUDIO_PIPELINE_CHANNELS; ch++) {
            memcpy(&pipeline_mic_ptr[ch * frame_count],
                   full_mic_frames[ch],
                   frame_count * sizeof(int32_t));
        }
#endif

#if MIC_ARRAY_CONFIG_MIC_COUNT >= 4
        doa_estimator_process_frame_4(full_mic_frames[0],
                                      full_mic_frames[1],
                                      full_mic_frames[2],
                                      full_mic_frames[3],
                                      frame_count);
#elif appconfAUDIO_PIPELINE_CHANNELS >= 2
        doa_estimator_process_frame(full_mic_frames[0],
                                    full_mic_frames[1],
                                    frame_count);
#endif
    }
    return received;
#else
    size_t received = rtos_mic_array_rx(mic_array_ctx,
                                        pipeline_mic_frames,
                                        frame_count,
                                        timeout);
#if MIC_ARRAY_CONFIG_MIC_COUNT >= 4 && appconfAUDIO_PIPELINE_CHANNELS >= 4
    if (received == frame_count) {
        int32_t *pipeline_mic_ptr = (int32_t *)pipeline_mic_frames;
        doa_estimator_process_frame_4(&pipeline_mic_ptr[0],
                                      &pipeline_mic_ptr[frame_count],
                                      &pipeline_mic_ptr[2 * frame_count],
                                      &pipeline_mic_ptr[3 * frame_count],
                                      frame_count);
    }
#elif appconfAUDIO_PIPELINE_CHANNELS >= 2
    if (received == frame_count) {
        int32_t *pipeline_mic_ptr = (int32_t *)pipeline_mic_frames;
        doa_estimator_process_frame(&pipeline_mic_ptr[0],
                                    &pipeline_mic_ptr[frame_count],
                                    frame_count);
    }
#endif
    return received;
#endif
}



void audio_pipeline_input(void *input_app_data,
                        int32_t **input_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
    (void) input_app_data;
    int32_t **mic_ptr = (int32_t **)(((int32_t *)input_audio_frames) +
                                     (appconfAUDIO_PIPELINE_CHANNELS * frame_count));

    static int flushed;
    while (!flushed) {
        size_t received;
        received = receive_mic_array_frame(mic_ptr,
                                           frame_count,
                                           0);
        if (received == 0) {
            receive_mic_array_frame(mic_ptr,
                                    frame_count,
                                    portMAX_DELAY);
            flushed = 1;
        }
    }

#if ON_TILE(SPEAKER_PIPELINE_TILE_NO)
    //read the speaker pipeline output as reference
    void *frame_data;
    (void) rtos_osal_queue_receive(ref_input_queue, &frame_data, RTOS_OSAL_WAIT_FOREVER);
    int32_t *tmpptr = (int32_t *)input_audio_frames;
    int32_t *refptr = (int32_t *)frame_data;

    for (int i=0; i<frame_count; i++) {
        /* ref is first */
        *(tmpptr + i) = *(refptr++);
        *(tmpptr + i + frame_count) = *(refptr++);
    }

    rtos_osal_free(frame_data);
#endif


    /*
     * NOTE: ALWAYS receive the next frame from the PDM mics,
     * even if USB is the current mic source. The controls the
     * timing since usb_audio_recv() does not block and will
     * receive all zeros if no frame is available yet.
     */
    receive_mic_array_frame(mic_ptr,
                            frame_count,
                            portMAX_DELAY);

}

int audio_pipeline_output(void *output_app_data,
                        int32_t **output_audio_frames,
                        size_t ch_count,
                        size_t frame_count)
{
    (void) output_app_data;


#if appconfI2S_ENABLED

    xassert(frame_count == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    /* I2S expects sample channel format */
    int32_t tmp[appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE][1][APP_I2S_STEREO_CHANNELS];
    int32_t *tmpptr = (int32_t *)output_audio_frames;
     
     // 0 : proc 0, AEC+IC+NS+AGC audio
     // 1 : proc 1, mic 1 audio with AEC applied
     // 2 : ref 0, overwritten by AEC+IC output
     // 3 : ref 1, overwritten by AEC+IC+NS output
     // 4 : mic 0
     // 5 : mic 1

     if (appconfI2S_AUDIO_SAMPLE_RATE == 3*appconfAUDIO_PIPELINE_SAMPLE_RATE) {    
        // duplicate to 48kHz
        for( int in_frame=0, out_frame=0; in_frame < frame_count; in_frame++, out_frame += 3 ){
            // CONF_CHANNEL_0_STAGE : AGC : AEC+IC+NS+AGC : indx 0       
            int32_t smpl_ch0 = *(tmpptr + in_frame + (0 * frame_count));
            
            // CONF_CHANNEL_1_STAGE : NS : AEC+IC+NS, stored in aec_reference_audio_samples[1]
            int32_t smpl_ch1 = *(tmpptr + in_frame + ((appconfAUDIO_PIPELINE_CHANNELS + 1) * frame_count));
            
            tmp[out_frame][0][0] = smpl_ch0;
            tmp[out_frame][0][1] = smpl_ch1;
            tmp[out_frame+1][0][0] = smpl_ch0;
            tmp[out_frame+1][0][1] = smpl_ch1;
            tmp[out_frame+2][0][0] = smpl_ch0;
            tmp[out_frame+2][0][1] = smpl_ch1;
        }
    } else {
        for (int j=0; j<frame_count; j++) {
            tmp[j][0][0] = *(tmpptr+j+(0*frame_count));    // (AEC+IC+NS+AGC) -> ESP32
            tmp[j][0][1] = *(tmpptr+j+((appconfAUDIO_PIPELINE_CHANNELS+1)*frame_count));    // (AEC+IC+NS) -> ESP32
        }
    }    
    
    
    rtos_i2s_tx(i2s_ctx,
                (int32_t*) tmp,
                appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE,
                portMAX_DELAY);
#endif

#if appconfUSB_AUDIO_ENABLED
    usb_audio_send(intertile_usb_audio_ctx,
                frame_count,
                output_audio_frames,
                6);
#endif

    return AUDIO_PIPELINE_FREE_FRAME;
}




void vApplicationMallocFailedHook(void)
{
    rtos_printf("Malloc Failed on tile %d!\n", THIS_XCORE_TILE);
    xassert(0);
    for(;;);
}

static void init_watchdog(void)
{
    //xin : 24 Mhz, decrement WATCHDOG_COUNT every 2.7 ms:
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_PRESCALER_WRAP_NUM, (0xFFFF));
    //trigger watchdog after ~11s of inactivity    
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_COUNT_NUM, 0xFFF );
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_CFG_NUM, (1 << XS1_WATCHDOG_COUNT_ENABLE_SHIFT) | (1 << XS1_WATCHDOG_TRIGGER_ENABLE_SHIFT) );
}

static void reset_watchdog(void)
{
    //reset watchdog to max
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_COUNT_NUM, 0xFFF );
}
static void mem_analysis(void)
{
	for (;;) {
		rtos_printf("Tile[%d]:\n\tMinimum heap free: %d\n\tCurrent heap free: %d\n", THIS_XCORE_TILE, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
        cdc_printf("Tile[%d]:\n\tMinimum heap free: %d\n\tCurrent heap free: %d\n", THIS_XCORE_TILE, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
#if ON_TILE(0)        
        reset_watchdog();
#endif        
        vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

void startup_task(void *arg)
{
    rtos_printf("Startup task running from tile %d on core %d\n", THIS_XCORE_TILE, portGET_CORE_ID());
    platform_start();


#if appconfDEVICE_CTRL_SPI
    device_control_t *device_control_ctx[1] = {device_control_spi_ctx}; 

#if ON_TILE(0)
    gpio_servicer_start(device_control_gpio_ctx, device_control_ctx, 1 );

    servicer_t dfu_servicer_ctx;
    dfu_servicer_init(&dfu_servicer_ctx);
    
    servicer_register_ctx_t dfu_servicer_reg_ctx = {
        &dfu_servicer_ctx,
        device_control_ctx,
        1,
        NULL
    };

    xTaskCreate(
        dfu_servicer,
        "dfu servicer",
        RTOS_THREAD_STACK_SIZE(dfu_servicer),
        &dfu_servicer_reg_ctx,
        appconfDEVICE_CONTROL_SPI_PRIORITY,
        NULL
    );
#endif

#if ON_TILE(WS2812_TILE_NO)
    servicer_t servicer_led_ring;
    led_ring_servicer_init(&servicer_led_ring);
    
    servicer_register_ctx_t led_ring_reg_ctx = {
        &servicer_led_ring,
        device_control_ctx,
        1,
        ws2812_ctx
    };
    
    xTaskCreate(
        led_ring_servicer,
        "LED-Ring servicer",
        RTOS_THREAD_STACK_SIZE(led_ring_servicer),
        &led_ring_reg_ctx,
        appconfDEVICE_CONTROL_SPI_PRIORITY,
        NULL
    );
#endif

#if ON_TILE(MICARRAY_TILE_NO)
    servicer_t doa_servicer_ctx;
    doa_servicer_init(&doa_servicer_ctx);

    servicer_register_ctx_t doa_servicer_reg_ctx = {
        &doa_servicer_ctx,
        device_control_ctx,
        1,
        NULL
    };

    xTaskCreate(
        doa_servicer,
        "DoA servicer",
        RTOS_THREAD_STACK_SIZE(doa_servicer),
        &doa_servicer_reg_ctx,
        appconfDEVICE_CONTROL_SPI_PRIORITY,
        NULL
    );
#endif

#endif


#if ON_TILE(SPEAKER_PIPELINE_TILE_NO)
    ref_input_queue = rtos_osal_malloc( sizeof(rtos_osal_queue_t) );
    rtos_osal_queue_create(ref_input_queue, NULL, 2, sizeof(void *));
    speaker_pipeline_init(NULL, NULL);
#endif

    audio_pipeline_init(NULL, NULL);
    
    init_watchdog();

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

#if appconfUSB_AUDIO_ENABLED && ON_TILE(USB_TILE_NO)
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
