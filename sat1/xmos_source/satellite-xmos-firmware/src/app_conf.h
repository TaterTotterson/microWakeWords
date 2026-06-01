// Copyright 2022-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef APP_CONF_H_
#define APP_CONF_H_

/* Auto-generated firmware-version file */
#include "version.h"

/* Intertile port settings */
#define appconfUSB_AUDIO_PORT          0
#define appconfGPIO_T0_RPC_PORT        1
#define appconfGPIO_T1_RPC_PORT        2
#define appconfAUDIOPIPELINE_PORT      7


/* Application tile specifiers */
#include "platform/driver_instances.h"
#define FS_TILE_NO                      FLASH_TILE_NO //currently not used, is it?


/* Audio Pipeline Configuration */
#define appconfAUDIO_CLOCK_FREQUENCY            MIC_ARRAY_CONFIG_MCLK_FREQ
#define appconfPDM_CLOCK_FREQUENCY              MIC_ARRAY_CONFIG_PDM_FREQ
#define appconfAUDIO_PIPELINE_SAMPLE_RATE       16000
#ifndef appconfAUDIO_PIPELINE_CHANNELS
#define appconfAUDIO_PIPELINE_CHANNELS          MIC_ARRAY_CONFIG_MIC_COUNT
#endif

#ifndef appconfAUDIO_SPK_PL_SR_FACTOR
#define appconfAUDIO_SPK_PL_SR_FACTOR           3
#endif

/* If in channel sample format, appconfAUDIO_PIPELINE_FRAME_ADVANCE == MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME*/
#define appconfAUDIO_PIPELINE_FRAME_ADVANCE     MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME
#define appconfAUDIO_SPK_PIPELINE_FRAME_ADVANCE appconfAUDIO_SPK_PL_SR_FACTOR * appconfAUDIO_PIPELINE_FRAME_ADVANCE

/**
 * A positive delay will delay mics
 * A negative delay will delay ref
 */
#define appconfINPUT_SAMPLES_MIC_DELAY_MS        0

#ifndef appconfPIPELINE_BYPASS
#define appconfPIPELINE_BYPASS 0
#endif

#if appconfPIPELINE_BYPASS
#define appconfAUDIO_PIPELINE_SKIP_STATIC_DELAY  1
#define appconfAUDIO_PIPELINE_SKIP_AEC           1
#define appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR    1
#define appconfAUDIO_PIPELINE_SKIP_NS            1
#define appconfAUDIO_PIPELINE_SKIP_AGC           1
#endif

#ifndef appconfAUDIO_PIPELINE_SKIP_STATIC_DELAY
#define appconfAUDIO_PIPELINE_SKIP_STATIC_DELAY  0
#endif

#ifndef appconfAUDIO_PIPELINE_SKIP_AEC
#define appconfAUDIO_PIPELINE_SKIP_AEC           0
#endif

#ifndef appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR
#define appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR    0
#endif

#ifndef appconfAUDIO_PIPELINE_SKIP_NS
#define appconfAUDIO_PIPELINE_SKIP_NS            0
#endif

#ifndef appconfAUDIO_PIPELINE_SKIP_AGC
#define appconfAUDIO_PIPELINE_SKIP_AGC           0
#endif

#ifndef appconfAUDIO_PIPELINE_STORE_IC_AUDIO 
#define appconfAUDIO_PIPELINE_STORE_IC_AUDIO     1
#endif

#ifndef appconfAUDIO_PIPELINE_STORE_NS_AUDIO
#define appconfAUDIO_PIPELINE_STORE_NS_AUDIO     1
#endif

#ifndef appconfI2S_ENABLED
#define appconfI2S_ENABLED         1
#endif

#ifndef appconfUSB_ENABLED
#define appconfUSB_ENABLED         0
#endif

#ifndef appconfUSB_AUDIO_SAMPLE_RATE
#define appconfUSB_AUDIO_SAMPLE_RATE appconfAUDIO_PIPELINE_SAMPLE_RATE
#endif

#ifndef appconfI2S_AUDIO_SAMPLE_RATE
#define appconfI2S_AUDIO_SAMPLE_RATE appconfAUDIO_PIPELINE_SAMPLE_RATE
#endif

#ifndef appconfI2S_ESP_ENABLED
#define appconfI2S_ESP_ENABLED     0
#endif

#define appconfI2S_AUDIO_INPUTS    1
#define appconfI2S_AUDIO_OUTPUTS   2

#ifndef appconfEXTERNAL_MCLK
#define appconfEXTERNAL_MCLK       0
#endif

/*
 * This option sends all 6 16 KHz channels (two channels of processed audio,
 * stereo reference audio, and stereo microphone audio) out over a single
 * 48 KHz I2S line.
 */
#ifndef appconfI2S_TDM_ENABLED
#define appconfI2S_TDM_ENABLED     0
#endif

#define appconfI2S_MODE_MASTER     0
#define appconfI2S_MODE_SLAVE      1
#ifndef appconfI2S_MODE
#define appconfI2S_MODE            appconfI2S_MODE_MASTER
#endif

#define appconfAEC_REF_USB         0
#define appconfAEC_REF_I2S         1
#ifndef appconfAEC_REF_DEFAULT
#define appconfAEC_REF_DEFAULT     appconfAEC_REF_I2S
#endif

#define appconfMIC_SRC_MICS        0
#define appconfMIC_SRC_USB         1
#ifndef appconfMIC_SRC_DEFAULT
#define appconfMIC_SRC_DEFAULT     appconfMIC_SRC_MICS
#endif

#define appconfUSB_AUDIO_RELEASE   0
#define appconfUSB_AUDIO_TESTING   1
#ifndef appconfUSB_AUDIO_MODE
#define appconfUSB_AUDIO_MODE      appconfUSB_AUDIO_RELEASE
#endif

#define appconfSPI_AUDIO_RELEASE   0
#define appconfSPI_AUDIO_TESTING   1
#ifndef appconfSPI_AUDIO_MODE
#define appconfSPI_AUDIO_MODE      appconfSPI_AUDIO_TESTING
#endif


#include "app_conf_check.h"

/* I/O and interrupt cores for Tile 0 */
#define appconfSPI_IO_CORE                      1 /* Must be kept off core 0 with the RTOS tick ISR */
#define appconfSPI_INTERRUPT_CORE               2 /* Must be kept off I/O cores. */

#define appconfXUD_IO_CORE                      3 /* Must be kept off core 0 with the RTOS tick ISR */
#define appconfUSB_INTERRUPT_CORE               4 /* Must be kept off I/O cores. Best kept off core 0 with the tick ISR. */
#define appconfUSB_SOF_INTERRUPT_CORE           5 /* Must be kept off I/O cores. Best kept off cores with other ISRs. */



/* I/O and interrupt cores for Tile 1 */
#define appconfPDM_MIC_IO_CORE                  1 /* Must be kept off core 0 with the RTOS tick ISR */
#define appconfI2S_IO_CORE                      2 /* Must be kept off core 0 with the RTOS tick ISR */
#define appconfI2C_IO_CORE                      5 /* Must be kept off core 0 with the RTOS tick ISR */
#define appconfPDM_MIC_INTERRUPT_CORE           4 /* Must be kept off I/O cores. Best kept off core 0 with the tick ISR. */
#define appconfI2S_INTERRUPT_CORE               5 /* Must be kept off I/O cores. Best kept off core 0 with the tick ISR. */
#define appconfI2C_INTERRUPT_CORE               4 /* Must be kept off I/O cores. */

/* Task Priorities */
#define appconfSTARTUP_TASK_PRIORITY              (configMAX_PRIORITIES/2 + 5)
#define appconfAUDIO_PIPELINE_TASK_PRIORITY       (configMAX_PRIORITIES / 2)
#define appconfGPIO_RPC_HOST_PRIORITY             (configMAX_PRIORITIES/2 + 2)
#define appconfGPIO_TASK_PRIORITY                 (configMAX_PRIORITIES/2 + 2)
#define appconfI2C_TASK_PRIORITY                  (configMAX_PRIORITIES/2 + 2)
#define appconfUSB_MGR_TASK_PRIORITY              (configMAX_PRIORITIES/2 + 1)
#define appconfUSB_AUDIO_TASK_PRIORITY            (configMAX_PRIORITIES/2 + 1)
#define appconfSPI_TASK_PRIORITY                  (configMAX_PRIORITIES/2 + 1)
#define appconfQSPI_FLASH_TASK_PRIORITY           (configMAX_PRIORITIES/2 + 0)
#define appconfLED_TASK_PRIORITY                  (configMAX_PRIORITIES / 2 - 1)

/* Software PLL settings for mclk recovery configurations */
/* see fractions.h and register_setup.h for other pll settings */
#define appconfLRCLK_NOMINAL_HZ     appconfI2S_AUDIO_SAMPLE_RATE
#define appconfBCLK_NOMINAL_HZ      (appconfLRCLK_NOMINAL_HZ * 64)
#define PLL_RATIO                   (MIC_ARRAY_CONFIG_MCLK_FREQ / appconfLRCLK_NOMINAL_HZ)
#define PLL_CONTROL_LOOP_COUNT_INT  512  // How many refclk ticks (LRCLK) per control loop iteration. Aim for ~100Hz
#define PLL_PPM_RANGE               1000 // Max allowable diff in clk count. For the PID constants we
                                         // have chosen, this number should be larger than the number
                                         // of elements in the look up table as the clk count diff is
                                         // added to the LUT index with a multiplier of 1. Only used for INT mclkless


#endif /* APP_CONF_H_ */
