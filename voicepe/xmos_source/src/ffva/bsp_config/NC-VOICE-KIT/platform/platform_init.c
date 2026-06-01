// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>

/* App headers */
#include "platform_conf.h"
#include "platform/app_pll_ctrl.h"
#include "platform/driver_instances.h"
#include "platform/platform_init.h"
// #include "adaptive_rate_adjust.h"
#include "usb_support.h"
#include "device_control.h"
#include "servicer.h"

extern device_control_t *device_control_i2c_ctx;

static void mclk_init(chanend_t other_tile_c)
{
#if !appconfEXTERNAL_MCLK && ON_TILE(1)
    app_pll_init();
#endif
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    adaptive_rate_adjust_init();
#endif

#if ON_TILE(I2S2_TILE_NO)
    /*
     * Configure the MCLK input port on tile 0.
     * This is wired to appPLL/MCLK output from tile 1.
     * It is set up to clock itself. This allows GETTS to
     * be called on it to count its clock cycles. This
     * count is used to adjust its frequency to match the
     * USB host.
   */

    port_enable(PORT_MCLK2_IN_OUT);
    clock_enable(MCLK_CLKBLK_0);
    clock_set_source_port(MCLK_CLKBLK_0, PORT_MCLK2_IN_OUT);
    port_set_clock(PORT_MCLK2_IN_OUT, MCLK_CLKBLK_0);
    clock_start(MCLK_CLKBLK_0);
#endif
}

static void flash_init(void)
{
#if ON_TILE(FLASH_TILE_NO)
    fl_QuadDeviceSpec qspi_spec[2] = {FL_QUADDEVICE_W25Q32JV, FL_QUADDEVICE_ZB25VQ32D};
    fl_QSPIPorts qspi_ports = {
        .qspiCS = PORT_SQI_CS,
        .qspiSCLK = PORT_SQI_SCLK,
        .qspiSIO = PORT_SQI_SIO,
        .qspiClkblk = FLASH_CLKBLK,
    };

    // try to connect to spi flash and set the boot part
    xassert(fl_connectToDevice(&qspi_ports, qspi_spec, 2) == 0);
    // We have 4MB SPI Flash, 2MB for firmware storage and 2MB for data storage
    fl_setBootPartitionSize(0x200000);

    rtos_dfu_image_init(
            dfu_image_ctx,
            &qspi_ports,
            qspi_spec,
            2);

    rtos_qspi_flash_init(
            qspi_flash_ctx,
            FLASH_CLKBLK,
            PORT_SQI_CS,
            PORT_SQI_SCLK,
            PORT_SQI_SIO,
            NULL);

    rtos_dfu_image_print_debug(dfu_image_ctx);
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

static void i2c_init(void)
{
    static rtos_driver_rpc_t i2c_rpc_config;

// #if appconfI2C_CTRL_ENABLED
// #if ON_TILE(I2C_CTRL_TILE_NO)
#if appconfI2C_DFU_ENABLED && ON_TILE(I2C_CTRL_TILE_NO)
    rtos_i2c_slave_init(i2c_slave_ctx,
                        (1 << appconfI2C_IO_CORE),
                        PORT_I2C_SCL,
                        PORT_I2C_SDA,
                        appconf_CONTROL_I2C_DEVICE_ADDR);
#endif
// #else
#if ON_TILE(I2C_TILE_NO)
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    rtos_i2c_master_init(
            i2c_master_ctx,
            PORT_I2C_SCL, 0, 0,
            PORT_I2C_SDA, 0, 0,
            0,
            100);

    rtos_i2c_master_rpc_host_init(
            i2c_master_ctx,
            &i2c_rpc_config,
            client_intertile_ctx,
            1);
#else
    rtos_i2c_master_rpc_client_init(
            i2c_master_ctx,
            &i2c_rpc_config,
            intertile_ctx);
#endif
// #endif
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
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    static rtos_driver_rpc_t i2s_rpc_config;
#endif
#if ON_TILE(I2S_TILE_NO)
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    port_t p_i2s_dout[1] = {
            PORT_I2S_DAC_DATA
    };
    port_t p_i2s_din[1] = {
            PORT_I2S_ADC_DATA
    };

    rtos_i2s_master_init(
            i2s1_ctx,
            (1 << appconfI2S_IO_CORE),
            p_i2s_dout,
            1,
            p_i2s_din,
            1,
            PORT_I2S_BCLK,
            PORT_I2S_LRCLK,
            PORT_MCLK,
            I2S_CLKBLK);

    rtos_i2s_rpc_host_init(
            i2s1_ctx,
            &i2s_rpc_config,
            client_intertile_ctx,
            1);
#elif appconfI2S_MODE == appconfI2S_MODE_SLAVE
    port_t p_i2s_dout[1] = {
            PORT_I2S_ADC_DATA
    };
    port_t p_i2s_din[1] = {
            PORT_I2S_DAC_DATA
    };
    rtos_i2s_slave_init(
            i2s1_ctx,
            (1 << appconfI2S_IO_CORE),
            p_i2s_dout,
            1,
            p_i2s_din,
            1,
            PORT_I2S_BCLK,
            PORT_I2S_LRCLK,
            I2S_CLKBLK);
#endif
#else
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    rtos_i2s_rpc_client_init(
            i2s1_ctx,
            &i2s_rpc_config,
            intertile_ctx);
#endif
#endif
#endif
}

static void i2s2_init(void)
{
    // don't need rpc
//    static rtos_driver_rpc_t i2s2_rpc_config;

#if ON_TILE(I2S2_TILE_NO)
    // rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    port_t p_i2s2_out[1] = {
            I2S2_DATA
    };

    rtos_i2s2_master_init(
        i2s2_ctx,
        (1 << appconfI2S2_IO_CORE),
        p_i2s2_out,
        1,
        NULL,
        0,
        PORT_I2S2_BCLK,
        PORT_I2S2_LRCLK,
        PORT_MCLK2_IN_OUT,
        I2S2_CLKBLK);

    // rtos_i2s2_rpc_host_init(
    //         i2s2_ctx,
    //         &i2s2_rpc_config,
    //         client_intertile_ctx,
    //         1);

#else
    // rtos_i2s2_rpc_client_init(
    //         i2s2_ctx,
    //         &i2s2_rpc_config,
    //         intertile_ctx);
#endif
}

static void usb_init(void)
{
// #if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
#if (appconfUSB_ENABLED || appconfUSB_DFU_ONLY_ENABLED) && ON_TILE(USB_TILE_NO)
    usb_manager_init();
#endif
}

void control_init() {
#if appconfI2C_DFU_ENABLED
    control_ret_t ret = CONTROL_SUCCESS;
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};

#if ON_TILE(I2C_CTRL_TILE_NO)
    ret = device_control_init(device_control_i2c_ctx,
                                DEVICE_CONTROL_HOST_MODE,
                                (NUM_TILE_0_SERVICERS + NUM_TILE_1_SERVICERS),
                                client_intertile_ctx, 1);
    xassert(ret == CONTROL_SUCCESS);

    ret = device_control_start(device_control_i2c_ctx,
                                appconfDEVICE_CONTROL_I2C_PORT,
                                -1);
    xassert(ret == CONTROL_SUCCESS);
#else
    ret = device_control_init(device_control_i2c_ctx,
                                DEVICE_CONTROL_CLIENT_MODE,
                                0,
                                client_intertile_ctx, 1);
    xassert(ret == CONTROL_SUCCESS);

    ret = device_control_start(device_control_i2c_ctx,
                                appconfDEVICE_CONTROL_I2C_PORT,
                                appconfDEVICE_CONTROL_I2C_PRIORITY);
    xassert(ret == CONTROL_SUCCESS);
#endif
#endif
}

void platform_init(chanend_t other_tile_c)
{
    rtos_intertile_init(intertile_ctx, other_tile_c);
    rtos_intertile_init(intertile_usb_audio_ctx, other_tile_c);

    mclk_init(other_tile_c);
    gpio_init();
    flash_init();
    i2c_init();
    mics_init();
    i2s_init();
    i2s2_init();
    usb_init();
    control_init();
}
