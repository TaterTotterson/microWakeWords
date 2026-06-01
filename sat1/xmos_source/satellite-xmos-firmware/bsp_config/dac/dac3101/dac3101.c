// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <xcore/assert.h>

/* App headers */
#include "platform/driver_instances.h"
#include "dac3101.h"

/*
 * Example configuration of the TLV320DAC3101 DAC using i2c.
 *
 * Must be called after the RTOS scheduler is started.
 */
int dac3101_init(uint32_t sample_rate)
{
    dac3101_codec_reset();

    xassert((sample_rate == 16000) || (sample_rate == 48000));

    // This setup is for 1.024MHz in (BCLK), PLL of 98.304MHz 24.576MHz out and fs of 16kHz or
    // or 3.072MHz BCLK, PLL of 98.304MHz 24.576MHz out and fs of 48kHz
    const unsigned PLLP = 1;
    const unsigned PLLR = 4;
    const unsigned PLLJ = (sample_rate == 16000) ? 24 : 8;
    const unsigned PLLD = 0;
    const unsigned NDAC = 4;
    const unsigned MDAC = (sample_rate == 16000) ? 6 : 4;
    const unsigned DOSR = (sample_rate == 16000) ? 256 : 128;

	if (
		// Set register page to 0
		dac3101_reg_write(DAC3101_PAGE_CTRL, 0x00) == 0 &&

		// Initiate SW reset (PLL is powered off as part of reset)
		dac3101_reg_write(DAC3101_SW_RST, 0x01) == 0 &&

		// Program clock settings
        // Set PLL J Value
		dac3101_reg_write(DAC3101_PLL_J, PLLJ) == 0 &&
        // Set PLL D to...
        dac3101_reg_write(DAC3101_PLL_D_LSB, PLLD & 0xff) == 0 &&
        dac3101_reg_write(DAC3101_PLL_D_MSB, (PLLD & 0xff00) >> 8) == 0 &&

        // Set BCLK divider to 1
        dac3101_reg_write(DAC3101_B_DIV_VAL, 0x80 + 1) == 0
    ) {
		// Wait for 1 ms
		dac3101_wait(1);
	} else {
		return -1;
	}

	if (
        // Set PLL_CLKIN = BCLK (device pin), CODEC_CLKIN = PLL_CLK (generated on-chip)
        dac3101_reg_write(DAC3101_CLK_GEN_MUX, (0b01 << 2) + 0b11) == 0 &&

        // Set PLL P and R values and power up.
        dac3101_reg_write(DAC3101_PLL_P_R, 0x80 + (PLLP << 4)+ PLLR) == 0 &&

        // Set NDAC clock divider and power up.
        dac3101_reg_write(DAC3101_NDAC_VAL, 0x80 + NDAC) == 0 &&
        // Set MDAC clock divider and power up.
        dac3101_reg_write(DAC3101_MDAC_VAL, 0x80 + MDAC) == 0 &&
        // Set OSR clock divider to 256.
        dac3101_reg_write(DAC3101_DOSR_VAL_LSB, DOSR & 0xff) == 0 &&
        dac3101_reg_write(DAC3101_DOSR_VAL_MSB, (DOSR & 0xff00) >> 8) == 0 &&

        // Set CLKOUT Mux to DAC_CLK
        dac3101_reg_write(DAC3101_CLKOUT_MUX, 0x04) == 0 &&
        // Set CLKOUT M divider to 1 and power up.
        dac3101_reg_write(DAC3101_CLKOUT_M_VAL, 0x81) == 0 &&
        // Set GPIO1 output to come from CLKOUT output.
        dac3101_reg_write(DAC3101_GPIO1_IO, 0x10) == 0 &&

        // Set CODEC interface mode: I2S, 24 bit, slave mode (BCLK, WCLK both inputs).
        dac3101_reg_write(DAC3101_CODEC_IF, 0x20) == 0 &&
        // Set register page to 1
        dac3101_reg_write(DAC3101_PAGE_CTRL, 0x01) == 0 &&
        // Program common-mode voltage to mid scale 1.65V.
        dac3101_reg_write(DAC3101_HP_DRVR, 0x14) == 0 &&
        // Program headphone-specific depop settings.
        // De-pop, Power on = 800 ms, Step time = 4 ms
        dac3101_reg_write(DAC3101_HP_DEPOP, 0x4E) == 0 &&
        // Program routing of DAC output to the output amplifier (headphone/lineout or speaker)
        // LDAC routed to left channel mixer amp, RDAC routed to right channel mixer amp
        dac3101_reg_write(DAC3101_DAC_OP_MIX, 0x44) == 0 &&
        // Unmute and set gain of output driver
        // Unmute HPL, set gain = 0 db
        dac3101_reg_write(DAC3101_HPL_DRVR, 0x06) == 0 &&
        // Unmute HPR, set gain = 0 dB
        dac3101_reg_write(DAC3101_HPR_DRVR, 0x06) == 0 &&
        // Unmute Left Class-D, set gain = 12 dB
        dac3101_reg_write(DAC3101_SPKL_DRVR, 0x0C) == 0 &&
        // Unmute Right Class-D, set gain = 12 dB
        dac3101_reg_write(DAC3101_SPKR_DRVR, 0x0C) == 0 &&
        // Power up output drivers
        // HPL and HPR powered up
        dac3101_reg_write(DAC3101_HP_DRVR, 0xD4) == 0 &&
        // Power-up L and R Class-D drivers
        dac3101_reg_write(DAC3101_SPK_AMP, 0xC6) == 0 &&
        // Enable HPL output analog volume, set = -9 dB
        dac3101_reg_write(DAC3101_HPL_VOL_A, 0x92) == 0 &&
        // Enable HPR output analog volume, set = -9 dB
        dac3101_reg_write(DAC3101_HPR_VOL_A, 0x92) == 0 &&
        // Enable Left Class-D output analog volume, set = -9 dB
        dac3101_reg_write(DAC3101_SPKL_VOL_A, 0x92) == 0 &&
        // Enable Right Class-D output analog volume, set = -9 dB
        dac3101_reg_write(DAC3101_SPKR_VOL_A, 0x92) == 0
    ) {
		// Wait for 100 ms
        dac3101_wait(100);
	} else {
		return -1;
	}

	if (
        // Power up DAC
        // Set register page to 0
        dac3101_reg_write(DAC3101_PAGE_CTRL, 0x00) == 0 &&
        // Power up DAC channels and set digital gain
        // Powerup DAC left and right channels (soft step enabled)
        dac3101_reg_write(DAC3101_DAC_DAT_PATH, 0xD4) == 0 &&
        // DAC Left gain = 0dB
        dac3101_reg_write(DAC3101_DACL_VOL_D, 0x00) == 0 &&
        // DAC Right gain = 0dB
        dac3101_reg_write(DAC3101_DACR_VOL_D, 0x00) == 0 &&
        // Unmute digital volume control
        // Unmute DAC left and right channels
        dac3101_reg_write(DAC3101_DAC_VOL, 0x00) == 0
    ) {
		// Wait for 100 ms
        dac3101_wait(100);
	} else {
		return -1;
	}

    return 0;
}
