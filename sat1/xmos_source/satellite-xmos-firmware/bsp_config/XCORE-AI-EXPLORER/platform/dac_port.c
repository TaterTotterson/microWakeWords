// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <platform.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"

/* App headers */
#include "platform_conf.h"
#include "platform/driver_instances.h"
#include "aic3204.h"

int aic3204_reg_write(uint8_t reg, uint8_t val)
{
    i2c_regop_res_t ret;

    ret = rtos_i2c_master_reg_write(i2c_master_ctx, AIC3204_I2C_DEVICE_ADDR, reg, val);

    if (ret == I2C_REGOP_SUCCESS) {
        return 0;
    } else {
        return -1;
    }
}

void aic3204_codec_reset(void)
{
    const rtos_gpio_port_id_t codec_rst_port = rtos_gpio_port(PORT_CODEC_RST_N);
    rtos_gpio_port_enable(gpio_ctx_t1, codec_rst_port);
    rtos_gpio_port_out(gpio_ctx_t1, codec_rst_port, 0xF);
}

void aic3204_wait(uint32_t wait_ms)
{
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
}
