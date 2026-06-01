// Copyright 2020-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef AIC3204_H_
#define AIC3204_H_

#include <stdint.h>

// TLV320AIC3204 Device I2C Address
#define AIC3204_I2C_DEVICE_ADDR 0x18

// TLV320AIC3204 Register Addresses
// Page 0
#define AIC3204_PAGE_CTRL     0x00 // Register 0  - Page Control
#define AIC3204_SW_RST        0x01 // Register 1  - Software Reset
#define AIC3204_NDAC          0x0B // Register 11 - NDAC Divider Value
#define AIC3204_MDAC          0x0C // Register 12 - MDAC Divider Value
#define AIC3204_DOSR          0x0E // Register 14 - DOSR Divider Value (LS Byte)
#define AIC3204_NADC          0x12 // Register 18 - NADC Divider Value
#define AIC3204_MADC          0x13 // Register 19 - MADC Divider Value
#define AIC3204_AOSR          0x14 // Register 20 - AOSR Divider Value
#define AIC3204_CODEC_IF      0x1B // Register 27 - CODEC Interface Control
#define AIC3204_DAC_SIG_PROC  0x3C // Register 60 - DAC Sig Processing Block Control
#define AIC3204_ADC_SIG_PROC  0x3D // Register 61 - ADC Sig Processing Block Control
#define AIC3204_DAC_CH_SET1   0x3F // Register 63 - DAC Channel Setup 1
#define AIC3204_DAC_CH_SET2   0x40 // Register 64 - DAC Channel Setup 2
#define AIC3204_DACL_VOL_D    0x41 // Register 65 - DAC Left Digital Vol Control
#define AIC3204_DACR_VOL_D    0x42 // Register 66 - DAC Right Digital Vol Control
#define AIC3204_ADC_CH_SET    0x51 // Register 81 - ADC Channel Setup
#define AIC3204_ADC_FGA_MUTE  0x52 // Register 82 - ADC Fine Gain Adjust/Mute

// Page 1
#define AIC3204_PWR_CFG       0x01 // Register 1  - Power Config
#define AIC3204_LDO_CTRL      0x02 // Register 2  - LDO Control
#define AIC3204_PLAY_CFG1     0x03 // Register 3  - Playback Config 1
#define AIC3204_PLAY_CFG2     0x04 // Register 4  - Playback Config 2
#define AIC3204_OP_PWR_CTRL   0x09 // Register 9  - Output Driver Power Control
#define AIC3204_CM_CTRL       0x0A // Register 10 - Common Mode Control
#define AIC3204_HPL_ROUTE     0x0C // Register 12 - HPL Routing Select
#define AIC3204_HPR_ROUTE     0x0D // Register 13 - HPR Routing Select
#define AIC3204_HPL_GAIN      0x10 // Register 16 - HPL Driver Gain
#define AIC3204_HPR_GAIN      0x11 // Register 17 - HPR Driver Gain
#define AIC3204_HP_START      0x14 // Register 20 - Headphone Driver Startup
#define AIC3204_LPGA_P_ROUTE  0x34 // Register 52 - Left PGA Positive Input Route
#define AIC3204_LPGA_N_ROUTE  0x36 // Register 54 - Left PGA Negative Input Route
#define AIC3204_RPGA_P_ROUTE  0x37 // Register 55 - Right PGA Positive Input Route
#define AIC3204_RPGA_N_ROUTE  0x39 // Register 57 - Right PGA Negative Input Route
#define AIC3204_LPGA_VOL      0x3B // Register 59 - Left PGA Volume
#define AIC3204_RPGA_VOL      0x3C // Register 60 - Right PGA Volume
#define AIC3204_ADC_PTM       0x3D // Register 61 - ADC Power Tune Config
#define AIC3204_AN_IN_CHRG    0x47 // Register 71 - Analog Input Quick Charging Config
#define AIC3204_REF_STARTUP   0x7B // Register 123 - Reference Power Up Config

/**
 * Initialize the DAC
 *
 * \returns   0 on success
 *            -1 otherwise
 */
int aic3204_init(void);

/**
 * User defined function to perform the reg write
 *
 * \returns   0 on success
 *            -1 otherwise
 */
int aic3204_reg_write(uint8_t reg, uint8_t val);

/**
 * User defined function to perform the reset the device
 */
void aic3204_codec_reset(void);

/**
 * User defined function to perform a wait
 *
 * When called, this function must not return until
 * at least wait_ms milliseconds has passed
 */
void aic3204_wait(uint32_t wait_ms);

#endif /* AIC3204_H_ */
