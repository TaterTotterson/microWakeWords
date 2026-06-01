// Copyright 2022-2024 XMOS LIMITED.
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
#include "aic3204.h"
#include "usb_support.h"
#include "usb_cdc.h"


#if appconfDEVICE_CTRL_SPI
#include "device_control_spi.h"
#endif


static void gpio_start(void)
{
    rtos_gpio_rpc_config(gpio_ctx_t0, appconfGPIO_T0_RPC_PORT, appconfGPIO_RPC_PRIORITY);
    rtos_gpio_rpc_config(gpio_ctx_t1, appconfGPIO_T1_RPC_PORT, appconfGPIO_RPC_PRIORITY);

#if ON_TILE(0)
    rtos_gpio_start(gpio_ctx_t0);
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

static void i2c_master_start(void)
{
    rtos_i2c_master_rpc_config(i2c_master_ctx, appconfI2C_MASTER_RPC_PORT, appconfI2C_MASTER_RPC_PRIORITY);

#if ON_TILE(I2C_TILE_NO)
    rtos_i2c_master_start(i2c_master_ctx);
#endif
}

static void audio_codec_start(void)
{
#if appconfI2S_ENABLED
    int ret = 0;
#if ON_TILE(I2C_TILE_NO)
    if (aic3204_init() != 0) {
        rtos_printf("DAC initialization failed\n");
    }
    rtos_intertile_tx(intertile_ctx, 0, &ret, sizeof(ret));
#else
    rtos_intertile_rx_len(intertile_ctx, 0, RTOS_OSAL_WAIT_FOREVER);
    rtos_intertile_rx_data(intertile_ctx, &ret, sizeof(ret));
#endif
#endif
}


static void spi_start(void)
{
#if appconfDEVICE_CTRL_SPI && ON_TILE(SPI_CLIENT_TILE_NO)
#if 0 //do we need this?
    const rtos_gpio_port_id_t wifi_rst_port = rtos_gpio_port(WIFI_WUP_RST_N);
    rtos_gpio_port_enable(gpio_ctx_t0, wifi_rst_port);
    rtos_gpio_port_out(gpio_ctx_t0, wifi_rst_port, 0x00);

    const rtos_gpio_port_id_t wifi_cs_port = rtos_gpio_port(WIFI_CS_N);
    rtos_gpio_port_enable(gpio_ctx_t0, wifi_cs_port);
    rtos_gpio_port_out(gpio_ctx_t0, wifi_cs_port, 0x0F);
#endif
    rtos_spi_slave_start(spi_slave_ctx,
                         device_control_spi_ctx,
                         (rtos_spi_slave_start_cb_t) device_control_spi_start_cb,
                         (rtos_spi_slave_xfer_done_cb_t) device_control_spi_xfer_done_cb,
                         appconfSPI_INTERRUPT_CORE,
                         appconfSPI_TASK_PRIORITY);
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
    rtos_i2s_rpc_config(i2s_ctx, appconfI2S_RPC_PORT, appconfI2S_RPC_PRIORITY);

#if ON_TILE(I2S_TILE_NO)
    rtos_i2s_start(
            i2s_ctx,
            rtos_i2s_mclk_bclk_ratio(appconfAUDIO_CLOCK_FREQUENCY, appconfI2S_AUDIO_SAMPLE_RATE),
            I2S_MODE_I2S,
            2.2 * MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME * 3,
            1.2 * MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME * 3,
            appconfI2S_INTERRUPT_CORE);
#endif
#endif
}

static void usb_start(void)
{
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    usb_manager_start(appconfUSB_MGR_TASK_PRIORITY);
#endif
}

static void ws2812_start(void)
{
#if ON_TILE(WS2812_TILE_NO)
    rtos_ws2812_start(ws2812_ctx);
#endif    
}

static void usb_cdc_start(void)
{
#if appconfUSB_CDC_ENABLED
    rtos_cdc_rpc_config(appconfUSB_CDC_PORT, appconfUSB_CDC_PRIORITY);
#if ON_TILE(USB_TILE_NO)
    rtos_cdc_start();
#endif
#endif
}



void platform_start(void)
{
    rtos_intertile_start(intertile_ctx);
    rtos_intertile_start(intertile_usb_audio_ctx);

    gpio_start();
    flash_start();
    i2c_master_start();
    audio_codec_start();
    spi_start();
    mics_start();
    i2s_start();
    usb_start();
    ws2812_start();
    usb_cdc_start();
}
