// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef DRIVER_INSTANCES_H_
#define DRIVER_INSTANCES_H_

#include "rtos_gpio.h"
#include "rtos_i2c_master.h"
#include "rtos_i2c_slave.h"
#include "rtos_intertile.h"
#include "rtos_i2s.h"
#include "rtos_mic_array.h"
#include "rtos_qspi_flash.h"
#include "rtos_dfu_image.h"
#include "rtos_spi_slave.h"

/* Tile specifiers */
#define FLASH_TILE_NO      0
#define I2C_TILE_NO        0
#define I2C_CTRL_TILE_NO   I2C_TILE_NO
#define SPI_OUTPUT_TILE_NO 0
#define MICARRAY_TILE_NO   1
#define I2S_TILE_NO        1
#define I2S2_TILE_NO       0

/** TILE 0 Clock Blocks */
#define FLASH_CLKBLK  XS1_CLKBLK_1
#define I2S2_CLKBLK   XS1_CLKBLK_2
#define SPI_CLKBLK    XS1_CLKBLK_3
#define MCLK_CLKBLK_0    XS1_CLKBLK_3
#define XUD_CLKBLK_1  XS1_CLKBLK_4 /* Reserved for lib_xud */
#define XUD_CLKBLK_2  XS1_CLKBLK_5 /* Reserved for lib_xud */

/** TILE 1 Clock Blocks */
#define PDM_CLKBLK_1  XS1_CLKBLK_1
#define PDM_CLKBLK_2  XS1_CLKBLK_2
#define I2S_CLKBLK    XS1_CLKBLK_3
#define MCLK_CLKBLK   XS1_CLKBLK_4
// #define UNUSED_CLKBLK XS1_CLKBLK_5

/* Port definitions */
#define PORT_MCLK           PORT_MCLK_IN_OUT
#define PORT_SQI_CS         PORT_SQI_CS_0
#define PORT_SQI_SCLK       PORT_SQI_SCLK_0
#define PORT_SQI_SIO        PORT_SQI_SIO_0
#define PORT_I2S_DAC_DATA   I2S_DATA_IN
#define PORT_I2S_ADC_DATA   I2S_MIC_DATA
#define PORT_I2C_SLAVE_SCL  PORT_I2C_SCL
#define PORT_I2C_SLAVE_SDA  PORT_I2C_SDA
#define PORT_SPI_CS         PORT_SSB
#define PORT_SPI_SCLK       PORT_SQI_SCLK_0

/* Pin definitions */
#define PIN_MUTE_DET_IN     (1<<4) /* PORT_GPI, x0d40, 8d4 */
#define PIN_BUT_A_IN        (1<<5) /* PORT_GPI, x0d41, 8d5, not used */
#define PIN_WS2812_OUT      (1<<6) /* PORT_GPI, x0d42, 8d6, not used */
#define PIN_ESPIO40         (1<<7) /* PORT_GPI, x0d43, 8d7 */
#define PIN_ESPIO41         (1<<7) /* PORT_GPO, x0d33, 8c7 */
#define PIN_RST_DAC_OUT     (1<<6) /* PORT_GPO, x0d32, 8c6 */
#define PIN_AMP_EN_OUT      (1<<5) /* PORT_GPO, x0d31, 8c5 */
#define PIN_I2C2_SCL        (1<<1) /* PORT_I2C2_SCL, x0d29, 4f1 */
#define PIN_I2C2_SDA        (1<<2) /* PORT_I2C2_SDA, x0d30, 4f2 */

extern rtos_intertile_t *intertile_ctx;
extern rtos_intertile_t *intertile_usb_audio_ctx;
extern rtos_qspi_flash_t *qspi_flash_ctx;
extern rtos_gpio_t *gpio_ctx_t0;
extern rtos_gpio_t *gpio_ctx_t1;
extern rtos_mic_array_t *mic_array_ctx;
extern rtos_i2c_master_t *i2c_master_ctx;
extern rtos_i2c_slave_t *i2c_slave_ctx;
extern rtos_spi_slave_t *spi_slave_ctx;
extern rtos_i2s_t *i2s1_ctx;
extern rtos_i2s_t *i2s2_ctx;
extern rtos_dfu_image_t *dfu_image_ctx;

#endif /* DRIVER_INSTANCES_H_ */
