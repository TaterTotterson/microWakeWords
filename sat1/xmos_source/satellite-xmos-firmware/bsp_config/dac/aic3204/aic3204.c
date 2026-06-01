// Copyright 2020-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* App headers */
#include "platform/driver_instances.h"
#include "aic3204.h"

/*
 * Example configuration of the TLV320AIC3204 DAC using i2c.
 *
 * For details on the TLV320AIC3204 registers and configuration sequence,
 * see chapters 4 and 5 here: https://www.ti.com/lit/ml/slaa557/slaa557.pdf
 *
 * Must be called after the RTOS scheduler is started.
 */
int aic3204_init(void)
{
    aic3204_codec_reset();

	if (
		// Set register page to 0
		aic3204_reg_write(AIC3204_PAGE_CTRL, 0x00) == 0 &&

		// Initiate SW reset (PLL is powered off as part of reset)
		aic3204_reg_write(AIC3204_SW_RST, 0x01) == 0 &&

		// Program clock settings

		// Default is CODEC_CLKIN is from MCLK pin. Don't need to change this.
		// Power up NDAC and set to 1
		aic3204_reg_write(AIC3204_NDAC, 0x81) == 0 &&
		// Power up MDAC and set to 4
		aic3204_reg_write(AIC3204_MDAC, 0x84) == 0 &&
        // Power up NADC and set to 1
		aic3204_reg_write(AIC3204_NADC, 0x81) == 0 &&
        // Power up MADC and set to 4
	    aic3204_reg_write(AIC3204_MADC, 0x84) == 0 &&
		// Program DOSR = 128
		aic3204_reg_write(AIC3204_DOSR, 0x80) == 0 &&
        // Program AOSR = 128
        aic3204_reg_write(AIC3204_AOSR, 0x80) == 0 &&
		// Set Audio Interface Config: I2S, 24 bits, slave mode, DOUT always driving.
		aic3204_reg_write(AIC3204_CODEC_IF, 0x20) == 0 &&
		// Program the DAC processing block to be used - PRB_P1
		aic3204_reg_write(AIC3204_DAC_SIG_PROC, 0x01) == 0 &&
		// Program the ADC processing block to be used - PRB_R1
        aic3204_reg_write(AIC3204_ADC_SIG_PROC, 0x01) == 0 &&
		// Select Page 1
		aic3204_reg_write(AIC3204_PAGE_CTRL, 0x01) == 0 &&
		// Enable the internal AVDD_LDO:
		aic3204_reg_write(AIC3204_LDO_CTRL, 0x09) == 0 &&

		//
		// Program Analog Blocks
		// ---------------------
		//
		// Disable Internal Crude AVdd in presence of external AVdd supply or before powering up internal AVdd LDO
		aic3204_reg_write(AIC3204_PWR_CFG, 0x08) == 0 &&
		// Enable Master Analog Power Control
		aic3204_reg_write(AIC3204_LDO_CTRL, 0x01) == 0 &&
		// Set Common Mode voltages: Full Chip CM to 0.9V and Output Common Mode for Headphone to 1.65V and HP powered from LDOin @ 3.3V.
		aic3204_reg_write(AIC3204_CM_CTRL, 0x33) == 0 &&
		// Set PowerTune Modes
		// Set the Left & Right DAC PowerTune mode to PTM_P3/4. Use Class-AB driver.
		aic3204_reg_write(AIC3204_PLAY_CFG1, 0x00) == 0 &&
		aic3204_reg_write(AIC3204_PLAY_CFG2, 0x00) == 0 &&
		// Set ADC PowerTune mode PTM_R4.
		aic3204_reg_write(AIC3204_ADC_PTM, 0x00) == 0 &&
		// Set MicPGA startup delay to 3.1ms
		aic3204_reg_write(AIC3204_AN_IN_CHRG, 0x31) == 0 &&
		// Set the REF charging time to 40ms
		aic3204_reg_write(AIC3204_REF_STARTUP, 0x01) == 0 &&
		// HP soft stepping settings for optimal pop performance at power up
		// Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling
		// capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs “pop” sound.
		aic3204_reg_write(AIC3204_HP_START, 0x25) == 0 &&
		// Route Left DAC to HPL
		aic3204_reg_write(AIC3204_HPL_ROUTE, 0x08) == 0 &&
		// Route Right DAC to HPR
		aic3204_reg_write(AIC3204_HPR_ROUTE, 0x08) == 0 &&
		// We are using Line input with low gain for PGA so can use 40k input R but lets stick to 20k for now.
		// Route IN2_L to LEFT_P with 20K input impedance
		aic3204_reg_write(AIC3204_LPGA_P_ROUTE, 0x20) == 0 &&
		// Route IN2_R to LEFT_M with 20K input impedance
		aic3204_reg_write(AIC3204_LPGA_N_ROUTE, 0x20) == 0 &&
		// Route IN1_R to RIGHT_P with 20K input impedance
		aic3204_reg_write(AIC3204_RPGA_P_ROUTE, 0x80) == 0 &&
		// Route IN1_L to RIGHT_M with 20K input impedance
		aic3204_reg_write(AIC3204_RPGA_N_ROUTE, 0x20) == 0 &&
		// Unmute HPL and set gain to 0dB
		aic3204_reg_write(AIC3204_HPL_GAIN, 0x00) == 0 &&
		// Unmute HPR and set gain to 0dB
		aic3204_reg_write(AIC3204_HPR_GAIN, 0x00) == 0 &&
		// Unmute Left MICPGA, Set Gain to 0dB.
		aic3204_reg_write(AIC3204_LPGA_VOL, 0x00) == 0 &&
		// Unmute Right MICPGA, Set Gain to 0dB.
		aic3204_reg_write(AIC3204_RPGA_VOL, 0x00) == 0 &&
		// Power up HPL and HPR drivers
		aic3204_reg_write(AIC3204_OP_PWR_CTRL, 0x30) == 0
	) {
		// Wait for 2.5 sec for soft stepping to take effect
        aic3204_wait(2500);
	} else {
		return -1;
	}

	if (
		//
		// Power Up DAC/ADC
		// ----------------
		//
		// Select Page 0
		aic3204_reg_write(AIC3204_PAGE_CTRL, 0x00) == 0 &&
		// Power up the Left and Right DAC Channels. Route Left data to Left DAC and Right data to Right DAC.
		// DAC Vol control soft step 1 step per DAC word clock.
		aic3204_reg_write(AIC3204_DAC_CH_SET1, 0xd4) == 0 &&
		// Power up Left and Right ADC Channels, ADC vol ctrl soft step 1 step per ADC word clock.
		aic3204_reg_write(AIC3204_ADC_CH_SET, 0xc0) == 0 &&
		// Unmute Left and Right DAC digital volume control
		aic3204_reg_write(AIC3204_DAC_CH_SET2, 0x00) == 0 &&
		// Unmute Left and Right ADC Digital Volume Control.
		aic3204_reg_write(AIC3204_ADC_FGA_MUTE, 0x00) == 0
	) {
		return 0;
	} else {
		return -1;
	}
}
