// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"

/* Library headers */
#include "fs_support.h"

/* App headers */
#include "platform_conf.h"
#include "platform/driver_instances.h"
#include "usb_support.h"

// #if appconfI2C_CTRL_ENABLED
// #include "app_control/app_control.h"
#include "servicer.h"
#include "device_control_i2c.h"
#include "configuration_common.h"
// #endif

extern void i2s_rate_conversion_enable(void);
extern void i2s2_rate_conversion_enable(void);


static void gpio_start(void)
{
    rtos_gpio_rpc_config(gpio_ctx_t0, appconfGPIO_T0_RPC_PORT, appconfGPIO_RPC_PRIORITY);
    rtos_gpio_rpc_config(gpio_ctx_t1, appconfGPIO_T1_RPC_PORT, appconfGPIO_RPC_PRIORITY);

#if ON_TILE(0)
    rtos_gpio_start(gpio_ctx_t0);
    rtos_gpio_port_id_t gpo_port = rtos_gpio_port(PORT_GPO);
    rtos_gpio_port_id_t gpi_port = rtos_gpio_port(PORT_GPI);
    // rtos_gpio_port_id_t i2c2_port = rtos_gpio_port(PORT_I2C2_SCL);

    rtos_gpio_port_enable(gpio_ctx_t0, gpo_port);
    rtos_gpio_port_enable(gpio_ctx_t0, gpi_port);
    // rtos_gpio_port_enable(gpio_ctx_t0, i2c2_port);

    rtos_gpio_port_out(gpio_ctx_t0, gpo_port, 0);
    rtos_gpio_port_in(gpio_ctx_t0, gpi_port);
    // rtos_gpio_port_in(gpio_ctx_t0, i2c2_port);
#endif
#if ON_TILE(1)
    rtos_gpio_start(gpio_ctx_t1);
#endif
}

static void flash_start(void)
{
#if ON_TILE(FLASH_TILE_NO)
    uint32_t flash_core_map = ~((1 << appconfUSB_INTERRUPT_CORE) | (1 << appconfUSB_SOF_INTERRUPT_CORE));
    rtos_qspi_flash_start(qspi_flash_ctx, appconfQSPI_FLASH_TASK_PRIORITY);
    rtos_qspi_flash_op_core_affinity_set(qspi_flash_ctx, flash_core_map);
#endif
}

static void configuration_start(void)
{
#if ON_TILE(FLASH_TILE_NO)
    configuration_init();
    // read configuration
    uint8_t *tmp_buf = rtos_osal_malloc( sizeof(uint8_t) * 16);
    rtos_qspi_flash_read(
        qspi_flash_ctx,
        tmp_buf,
        rtos_qspi_flash_size_get(qspi_flash_ctx) - 4096 * 2,
        16);
    for(uint8_t i = 0; i < 16; i++)
    {
        rtos_printf("0x%02X, ", tmp_buf[i]);
    }
    rtos_printf("\n");
    rtos_osal_free(tmp_buf);
#endif
}

static void i2c_master_start(void)
{
// #if !appconfI2C_CTRL_ENABLED
    rtos_i2c_master_rpc_config(i2c_master_ctx, appconfI2C_MASTER_RPC_PORT, appconfI2C_MASTER_RPC_PRIORITY);

#if ON_TILE(I2C_TILE_NO)
    rtos_i2c_master_start(i2c_master_ctx);
#endif
// #endif
}

static void audio_codec_start(void)
{
// #if !appconfI2C_CTRL_ENABLED
#if appconfI2S_ENABLED
    int ret = 0;
#if ON_TILE(I2C_TILE_NO)
    //  <!-- RST_DAC X0D32/8C6/output -->
    rtos_gpio_port_id_t gpo_port = rtos_gpio_port(PORT_GPO);
    uint32_t val = rtos_gpio_port_in(gpio_ctx_t0, gpo_port);
    rtos_gpio_port_out(gpio_ctx_t0, gpo_port, val |= PIN_RST_DAC_OUT);

    rtos_intertile_tx(intertile_ctx, 0, &ret, sizeof(ret));
#else
    rtos_intertile_rx_len(intertile_ctx, 0, RTOS_OSAL_WAIT_FOREVER);
    rtos_intertile_rx_data(intertile_ctx, &ret, sizeof(ret));
#endif
#endif
// #endif
}

static void i2c_slave_start(void)
{
// #if appconfI2C_CTRL_ENABLED && ON_TILE(I2C_CTRL_TILE_NO)
#if appconfI2C_DFU_ENABLED && ON_TILE(I2C_CTRL_TILE_NO)
    // i2c_master_shutdown(&i2c_master_ctx->ctx); // It is better to deinit i2c master if we don't use it 
    rtos_i2c_slave_start(i2c_slave_ctx,
                         device_control_i2c_ctx,
                         (rtos_i2c_slave_start_cb_t) device_control_i2c_start_cb,
                         (rtos_i2c_slave_rx_cb_t) device_control_i2c_rx_cb,
                         (rtos_i2c_slave_tx_start_cb_t) device_control_i2c_tx_start_cb,
                         (rtos_i2c_slave_tx_done_cb_t) NULL,
                         NULL,
                         NULL,
                         appconfI2C_INTERRUPT_CORE,
                         appconfI2C_TASK_PRIORITY);
#endif
}

static void mics_start(void)
{
    rtos_mic_array_rpc_config(mic_array_ctx, appconfMIC_ARRAY_RPC_PORT, appconfMIC_ARRAY_RPC_PRIORITY);

#if ON_TILE(MICARRAY_TILE_NO)
    rtos_mic_array_start(
            mic_array_ctx,
            2 * MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME,
            appconfPDM_MIC_INTERRUPT_CORE);
#endif
}

static void i2s_start(void)
{
#if appconfI2S_ENABLED
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    rtos_i2s_rpc_config(i2s1_ctx, appconfI2S_RPC_PORT, appconfI2S_RPC_PRIORITY);
#endif
#if ON_TILE(I2S_TILE_NO)
    if (appconfI2S_AUDIO_SAMPLE_RATE == 3*appconfAUDIO_PIPELINE_SAMPLE_RATE) {
        i2s_rate_conversion_enable();
    }

    rtos_i2s_start(
            i2s1_ctx,
            rtos_i2s_mclk_bclk_ratio(appconfAUDIO_CLOCK_FREQUENCY, appconfI2S_AUDIO_SAMPLE_RATE),
            I2S_MODE_I2S,
            2.2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE,
            1.2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE * (appconfI2S_TDM_ENABLED ? 3 : 1),
            appconfI2S_INTERRUPT_CORE);
#endif
#endif
}

static void i2s2_start(void)
{
    // don't need rpc
    // rtos_i2s_rpc_config(i2s2_ctx, appconfI2S2_RPC_PORT, appconfI2S2_RPC_PRIORITY);

#if ON_TILE(I2S2_TILE_NO)

    // if (appconfI2S_AUDIO_SAMPLE_RATE == 3*appconfAUDIO_PIPELINE_SAMPLE_RATE) {
    //     i2s2_rate_conversion_enable();
    // }

    rtos_i2s_start(
            i2s2_ctx,
            rtos_i2s_mclk_bclk_ratio(
                appconfAUDIO_CLOCK_FREQUENCY,   //3.072M
                // MIC_ARRAY_CONFIG_MCLK_FREQ,  // 24.576M
                // appconfI2S_AUDIO_SAMPLE_RATE), // 48k
                16000), // 16k
            I2S_MODE_I2S,
            2.2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE,
            1.2 * appconfAUDIO_PIPELINE_FRAME_ADVANCE * (appconfI2S_TDM_ENABLED ? 3 : 1),
            appconfI2S2_INTERRUPT_CORE);
#endif
}

static void usb_start(void)
{
// #if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
#if (appconfUSB_ENABLED || appconfUSB_DFU_ONLY_ENABLED) && ON_TILE(USB_TILE_NO)
    usb_manager_start(appconfUSB_MGR_TASK_PRIORITY);
#endif
}

void platform_start(void)
{
    rtos_intertile_start(intertile_ctx);
    rtos_intertile_start(intertile_usb_audio_ctx);

    gpio_start();
    flash_start();
    i2c_master_start();
    // i2c_slave_start();
    // read codec configuration from flash first
    configuration_start();
    audio_codec_start();
    mics_start();
    i2s_start();
    i2s2_start();
    usb_start();
    // I2C slave can be started only after i2c_master_start() is completed
    i2c_slave_start();
}
