// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include "platform/driver_instances.h"

static rtos_intertile_t intertile_ctx_s;
rtos_intertile_t *intertile_ctx = &intertile_ctx_s;

static rtos_intertile_t intertile_usb_audio_ctx_s;
rtos_intertile_t *intertile_usb_audio_ctx = &intertile_usb_audio_ctx_s;

static rtos_qspi_flash_t qspi_flash_ctx_s;
rtos_qspi_flash_t *qspi_flash_ctx = &qspi_flash_ctx_s;

static rtos_gpio_t gpio_ctx_t0_s;
rtos_gpio_t *gpio_ctx_t0 = &gpio_ctx_t0_s;

static rtos_gpio_t gpio_ctx_t1_s;
rtos_gpio_t *gpio_ctx_t1 = &gpio_ctx_t1_s;

static rtos_mic_array_t mic_array_ctx_s;
rtos_mic_array_t *mic_array_ctx = &mic_array_ctx_s;

static rtos_i2c_master_t i2c_master_ctx_s;
rtos_i2c_master_t *i2c_master_ctx = &i2c_master_ctx_s;

static rtos_i2s_t i2s1_ctx_s;
rtos_i2s_t *i2s1_ctx = &i2s1_ctx_s;

static rtos_i2s_t i2s2_ctx_s;
rtos_i2s_t *i2s2_ctx = &i2s2_ctx_s;

static rtos_i2c_slave_t i2c_slave_ctx_s;
rtos_i2c_slave_t *i2c_slave_ctx = &i2c_slave_ctx_s;

static rtos_spi_slave_t spi_slave_ctx_s;
rtos_spi_slave_t *spi_slave_ctx = &spi_slave_ctx_s;

static rtos_dfu_image_t dfu_image_ctx_s;
rtos_dfu_image_t *dfu_image_ctx = &dfu_image_ctx_s;
