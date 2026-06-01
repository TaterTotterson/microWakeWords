// Copyright 2022-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>

/* App headers */
#include "platform_conf.h"
#include "platform/app_pll_ctrl.h"
#include "platform/driver_instances.h"
#include "platform/platform_init.h"
#include "adaptive_rate_adjust.h"
#include "usb_support.h"
#include "usb_cdc.h"

static void mclk_init(chanend_t other_tile_c)
{
#if ON_TILE(1) && !appconfEXTERNAL_MCLK
    app_pll_init();
#endif
#if appconfUSB_AUDIO_ENABLED && ON_TILE(USB_TILE_NO)
    adaptive_rate_adjust_init();
#endif
}

static void flash_init(void)
{
#if ON_TILE(FLASH_TILE_NO)
    fl_QuadDeviceSpec qspi_spec = BOARD_QSPI_SPEC;
    fl_QSPIPorts qspi_ports = {
        .qspiCS = PORT_SQI_CS,
        .qspiSCLK = PORT_SQI_SCLK,
        .qspiSIO = PORT_SQI_SIO,
        .qspiClkblk = FLASH_CLKBLK,
    };

    rtos_dfu_image_init(
            dfu_image_ctx,
            &qspi_ports,
            &qspi_spec,
            1);

    rtos_qspi_flash_init(
            qspi_flash_ctx,
            FLASH_CLKBLK,
            PORT_SQI_CS,
            PORT_SQI_SCLK,
            PORT_SQI_SIO,
            NULL);
#endif
}

static void gpio_init(void)
{
    static rtos_driver_rpc_t gpio_rpc_config_t0;
    static rtos_driver_rpc_t gpio_rpc_config_t1;
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};

#if ON_TILE(0)
    rtos_gpio_init(gpio_ctx_t0);

    rtos_gpio_rpc_host_init(
            gpio_ctx_t0,
            &gpio_rpc_config_t0,
            client_intertile_ctx,
            1);

    rtos_gpio_rpc_client_init(
            gpio_ctx_t1,
            &gpio_rpc_config_t1,
            intertile_ctx);
#endif

#if ON_TILE(1)
    rtos_gpio_init(gpio_ctx_t1);

    rtos_gpio_rpc_client_init(
            gpio_ctx_t0,
            &gpio_rpc_config_t0,
            intertile_ctx);

    rtos_gpio_rpc_host_init(
            gpio_ctx_t1,
            &gpio_rpc_config_t1,
            client_intertile_ctx,
            1);
#endif
}



static void spi_init(void)
{
#if appconfDEVICE_CTRL_SPI 
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
#if ON_TILE(SPI_CLIENT_TILE_NO)
    rtos_spi_slave_init(spi_slave_ctx,
                        (1 << appconfSPI_IO_CORE),
                        SPI_CLKBLK,
                        SPI_MODE_3,
                        PORT_XSPI_CLK,
                        PORT_XSPI_MOSI,
                        PORT_XSPI_MISO,
                        PORT_XSPI_CS);
    
    device_control_init(device_control_spi_ctx,
                        DEVICE_CONTROL_HOST_MODE,
                        3, //number of servicers
                        client_intertile_ctx,
                        1); 
    
    device_control_start(device_control_spi_ctx,
                         appconfSPI_DEV_CTRL_PORT,
                         -1);
#else
    device_control_init(device_control_spi_ctx,
                        DEVICE_CONTROL_CLIENT_MODE,
                        0,
                        client_intertile_ctx,
                        1); 
    
    device_control_start(device_control_spi_ctx,
                         appconfSPI_DEV_CTRL_PORT,
                         appconfSPI_DEV_CTRL_PRIORITY);
#endif
#endif
}

static void mics_init(void)
{
    static rtos_driver_rpc_t mic_array_rpc_config;
#if ON_TILE(MICARRAY_TILE_NO)
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    rtos_mic_array_init(
            mic_array_ctx,
            (1 << appconfPDM_MIC_IO_CORE),
            RTOS_MIC_ARRAY_CHANNEL_SAMPLE);
    rtos_mic_array_rpc_host_init(
            mic_array_ctx,
            &mic_array_rpc_config,
            client_intertile_ctx,
            1);
#else
    rtos_mic_array_rpc_client_init(
            mic_array_ctx,
            &mic_array_rpc_config,
            intertile_ctx);
#endif
}

static void i2s_init(void)
{
#if appconfI2S_ENABLED
    static rtos_driver_rpc_t i2s_rpc_config;
#if ON_TILE(I2S_TILE_NO)
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    port_t p_i2s_dout[appconfI2S_AUDIO_OUTPUTS] = {
           PORT_I2S_DOUT1,
           PORT_I2S_DOUT2
    };
    port_t p_i2s_din[appconfI2S_AUDIO_INPUTS] = {
           PORT_I2S_DIN
    };

    rtos_i2s_master_init(
            i2s_ctx,
            (1 << appconfI2S_IO_CORE),
            p_i2s_dout,
            appconfI2S_AUDIO_OUTPUTS,
            p_i2s_din,
            appconfI2S_AUDIO_INPUTS,
            PORT_I2S_BCLK,
            PORT_I2S_LRCLK,
            PORT_MCLK,
            I2S_CLKBLK);
    rtos_i2s_rpc_host_init(
            i2s_ctx,
            &i2s_rpc_config,
            client_intertile_ctx,
            1);
#else
    rtos_i2s_rpc_client_init(
            i2s_ctx,
            &i2s_rpc_config,
            intertile_ctx);
#endif
#endif

}

static void usb_init(void)
{
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    usb_manager_init();
#endif
}

static void ws2812_init(void)
{
#if ON_TILE(WS2812_TILE_NO)
   rtos_ws2812_init(ws2812_ctx, PORT_LED_RING, LED_RING_PORT_PIN, LED_RING_NUM_LEDS); 
#endif    
}

static void servicer_init(void)
{
#if ON_TILE(0)
    static device_control_gpio_ports_t gpio_res_info[GPIO_CONTROLLER_MAX_RESOURCES];      
    gpio_res_info[0].resource_idx = RESOURCE_IN_A;
    gpio_res_info[0].writeable = false;
    gpio_res_info[0].port_id = PORT_BUTTONS;
    gpio_res_info[0].bit_mask = 240;
    gpio_res_info[0].bit_shift = 4;
    gpio_res_info[0].status_register = 1;

    
    gpio_res_info[1].resource_idx = RESOURCE_IN_B;
    gpio_res_info[1].writeable = false;
    gpio_res_info[1].port_id = PORT_ROTARY_ENC;
    gpio_res_info[1].bit_mask = 14;
    gpio_res_info[1].bit_shift = 1;
    gpio_res_info[1].status_register = 2;

    gpio_servicer_init( device_control_gpio_ctx,
                        gpio_ctx_t0,
                        gpio_res_info,
                        1 );
#endif
}

static void usb_cdc_init(){
#if appconfUSB_CDC_ENABLED
#if ON_TILE(USB_TILE_NO)
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    rtos_cdc_rpc_host_init(client_intertile_ctx,1);    
#else
    rtos_cdc_rpc_client_init(intertile_ctx);
#endif
#endif
}



void platform_init(chanend_t other_tile_c)
{
    rtos_intertile_init(intertile_ctx, other_tile_c);
#if appconfUSB_AUDIO_ENABLED
    rtos_intertile_init(intertile_usb_audio_ctx, other_tile_c);
#endif
    mclk_init(other_tile_c);
    gpio_init();
    flash_init();
    spi_init();
    mics_init();
    i2s_init();
    usb_init();
    ws2812_init();
    servicer_init();
    usb_cdc_init();    
}
