// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef DRIVER_INSTANCES_H_
#define DRIVER_INSTANCES_H_

#include "rtos_gpio.h"
#include "rtos_i2c_master.h"
#include "rtos_intertile.h"
#include "rtos_i2s.h"
#include "rtos_mic_array.h"
#include "rtos_qspi_flash.h"
#include "rtos_dfu_image.h"
#include "rtos_spi_slave.h"
#include "rtos_ws2812.h"
#include "device_control.h"
#include "gpio/gpio_servicer.h"

/* Tile specifiers */
#define FLASH_TILE_NO      0
#define I2C_TILE_NO        0
#define SPI_CLIENT_TILE_NO 0

#define MICARRAY_TILE_NO   1
#define I2S_TILE_NO        1
#define SPEAKER_PIPELINE_TILE_NO I2S_TILE_NO 
#define WS2812_TILE_NO     1


/** TILE 0 Clock Blocks */
#define FLASH_CLKBLK  XS1_CLKBLK_1
#define MCLK_CLKBLK   XS1_CLKBLK_2
#define SPI_CLKBLK    XS1_CLKBLK_3
#define XUD_CLKBLK_1  XS1_CLKBLK_4 /* Reserved for lib_xud */
#define XUD_CLKBLK_2  XS1_CLKBLK_5 /* Reserved for lib_xud */

/** TILE 1 Clock Blocks */
#define PDM_CLKBLK_1  XS1_CLKBLK_1
#define PDM_CLKBLK_2  XS1_CLKBLK_2
#define I2S_CLKBLK    XS1_CLKBLK_3
// #define UNUSED_CLKBLK XS1_CLKBLK_4
// #define UNUSED_CLKBLK XS1_CLKBLK_5

/* Port definitions */
#define PORT_MCLK           PORT_MCLK_IN
#define PORT_SPI_CS         XS1_PORT_1A
#define PORT_SPI_SCLK       WIFI_CLK
#define PORT_SPI_MOSI       WIFI_MOSI
#define PORT_SPI_MISO       WIFI_MISO

/*LED RING*/
#define LED_RING_NUM_LEDS   12
#define LED_RING_PORT_PIN    2

extern rtos_intertile_t *intertile_ctx;
extern rtos_intertile_t *intertile_usb_audio_ctx;
extern rtos_qspi_flash_t *qspi_flash_ctx;
extern rtos_gpio_t *gpio_ctx_t0;
extern rtos_gpio_t *gpio_ctx_t1;
extern rtos_mic_array_t *mic_array_ctx;
extern rtos_i2c_master_t *i2c_master_ctx;
extern rtos_spi_slave_t *spi_slave_ctx;
extern rtos_i2s_t *i2s_ctx;
extern rtos_dfu_image_t *dfu_image_ctx;
extern rtos_ws2812_t *ws2812_ctx;

extern device_control_t *device_control_spi_ctx;
extern device_control_gpio_ctx_t *device_control_gpio_ctx;

#endif /* DRIVER_INSTANCES_H_ */
