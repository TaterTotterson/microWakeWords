// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef PLATFORM_CONF_H_
#define PLATFORM_CONF_H_

/*
 * Board support package for XK_VOICE_L71
 */

#if __has_include("app_conf.h")
#include "app_conf.h"
#endif /* __has_include("app_conf.h") */

/*****************************************/
/* Intertile Communication Configuration */
/*****************************************/
#ifndef appconfI2C_MASTER_RPC_PORT
#define appconfI2C_MASTER_RPC_PORT 10
#endif /* appconfI2C_MASTER_RPC_PORT */

#ifndef appconfI2C_MASTER_RPC_PRIORITY
#define appconfI2C_MASTER_RPC_PRIORITY (configMAX_PRIORITIES/2)
#endif /* appconfI2C_MASTER_RPC_PRIORITY */

#ifndef appconfGPIO_T0_RPC_PORT
#define appconfGPIO_T0_RPC_PORT 11
#endif /* appconfGPIO_T0_RPC_PORT */

#ifndef appconfGPIO_T1_RPC_PORT
#define appconfGPIO_T1_RPC_PORT 12
#endif /* appconfGPIO_T1_RPC_PORT */

#ifndef appconfGPIO_RPC_PRIORITY
#define appconfGPIO_RPC_PRIORITY (configMAX_PRIORITIES/2)
#endif /* appconfGPIO_RPC_PRIORITY */

#ifndef appconfMIC_ARRAY_RPC_PORT
#define appconfMIC_ARRAY_RPC_PORT 13
#endif /* appconfMIC_ARRAY_RPC_PORT */

#ifndef appconfMIC_ARRAY_RPC_PRIORITY
#define appconfMIC_ARRAY_RPC_PRIORITY (configMAX_PRIORITIES-2)
#endif /* appconfMIC_ARRAY_RPC_PRIORITY */

#ifndef appconfI2S_RPC_PORT
#define appconfI2S_RPC_PORT 14
#endif /* appconfI2S_RPC_PORT */

#ifndef appconfI2S_RPC_PRIORITY
#define appconfI2S_RPC_PRIORITY (configMAX_PRIORITIES-2)
#endif /* appconfI2S_RPC_PRIORITY */

// Not used
// #ifndef appconfI2S2_RPC_PORT
// #define appconfI2S2_RPC_PORT 15
// #endif /* appconfI2S2_RPC_PORT */

// #ifndef appconfI2S2_RPC_PRIORITY
// #define appconfI2S2_RPC_PRIORITY (configMAX_PRIORITIES-2)
// #endif /* appconfI2S2_RPC_PRIORITY */

/*****************************************/
/*  I/O and interrupt cores              */
/*****************************************/
#ifndef appconfPDM_MIC_IO_CORE
#define appconfPDM_MIC_IO_CORE                  1 /* Must be kept off I/O cores. Must be kept off core 0 with the RTOS tick ISR */
#endif /* appconfPDM_MIC_IO_CORE */

#ifndef appconfI2S_IO_CORE
#define appconfI2S_IO_CORE                      2 /* Must be kept off core 0 with the RTOS tick ISR */
#endif /* appconfI2S_IO_CORE */

#ifndef appconfPDM_MIC_INTERRUPT_CORE
#define appconfPDM_MIC_INTERRUPT_CORE           3 /* Must be kept off I/O cores. Best kept off core 0 with the tick ISR. */
#endif /* appconfPDM_MIC_INTERRUPT_CORE */

#ifndef appconfI2S_INTERRUPT_CORE
#define appconfI2S_INTERRUPT_CORE               4 /* Must be kept off I/O cores. Best kept off core 0 with the tick ISR. */
#endif /* appconfI2S_INTERRUPT_CORE */

#ifndef appconfSPI_IO_CORE
#define appconfSPI_IO_CORE                      1 /* Must be kept off core 0 with the RTOS tick ISR */
#endif /* appconfSPI_IO_CORE */

#ifndef appconfI2C_IO_CORE
#define appconfI2C_IO_CORE                      3 /* Must be kept off core 0 with the RTOS tick ISR */
#endif /* appconfI2C_IO_CORE */

#ifndef appconfI2C_INTERRUPT_CORE
#define appconfI2C_INTERRUPT_CORE               0 /* Must be kept off I/O cores. */
#endif /* appconfI2C_INTERRUPT_CORE */

#ifndef appconfSPI_INTERRUPT_CORE
#define appconfSPI_INTERRUPT_CORE               2 /* Must be kept off I/O cores. */
#endif /* appconfSPI_INTERRUPT_CORE */

/*****************************************/
/*  Other required defines               */
/*****************************************/
#ifndef appconfPDM_CLOCK_FREQUENCY
#define appconfPDM_CLOCK_FREQUENCY          MIC_ARRAY_CONFIG_MCLK_FREQ
#endif /* appconfPDM_CLOCK_FREQUENCY */

#ifndef appconfAUDIO_CLOCK_FREQUENCY
#define appconfAUDIO_CLOCK_FREQUENCY        MIC_ARRAY_CONFIG_PDM_FREQ
#endif /* appconfAUDIO_CLOCK_FREQUENCY */

#ifndef appconfPIPELINE_AUDIO_SAMPLE_RATE
#define appconfPIPELINE_AUDIO_SAMPLE_RATE   16000
#endif /* appconfPIPELINE_AUDIO_SAMPLE_RATE */

#ifndef appconfI2C_CTRL_ENABLED
/*
 * When this is enabled on the XVF3610_Q60A board, the board
 * cannot function as an I2C master and will not configure the
 * DAC. In this case the DAC should be configured externally.
 * MCLK will also default to be external if this is set on
 * the XVF3610_Q60A board.
 */
#define appconfI2C_CTRL_ENABLED    0
#endif /* appconfI2C_CTRL_ENABLED */

#ifndef appconfEXTERNAL_MCLK
#if appconfI2C_CTRL_ENABLED
#define appconfEXTERNAL_MCLK       1
#else
#define appconfEXTERNAL_MCLK       0
#endif /* appconfI2C_CTRL_ENABLED */
#endif /* appconfEXTERNAL_MCLK */

#ifndef appconfI2C_DFU_ENABLED
#define appconfI2C_DFU_ENABLED 1
#endif /* appconfI2C_DFU_ENABLED */

#ifndef APP_CONTROL_TRANSPORT_COUNT
#define APP_CONTROL_TRANSPORT_COUNT appconfI2C_DFU_ENABLED
#endif /* APP_CONTROL_TRANSPORT_COUNT */ 

#ifndef appconf_CONTROL_I2C_DEVICE_ADDR
#define appconf_CONTROL_I2C_DEVICE_ADDR 0x42
#endif /* appconf_CONTROL_I2C_DEVICE_ADDR*/

#ifndef appconfSPI_OUTPUT_ENABLED
#define appconfSPI_OUTPUT_ENABLED  0
#endif /* appconfSPI_OUTPUT_ENABLED */

#ifndef appconfI2S_MODE_MASTER
#define appconfI2S_MODE_MASTER     0
#endif /* appconfI2S_MODE_MASTER */
#ifndef appconfI2S_MODE_SLAVE
#define appconfI2S_MODE_SLAVE      1
#endif /* appconfI2S_MODE_SLAVE */
#ifndef appconfI2S_MODE
#define appconfI2S_MODE            appconfI2S_MODE_MASTER
#endif /* appconfI2S_MODE */

/*
 * This option sends all 6 16 KHz channels (two channels of processed audio,
 * stereo reference audio, and stereo microphone audio) out over a single
 * 48 KHz I2S line.
 */
#ifndef appconfI2S_TDM_ENABLED
#define appconfI2S_TDM_ENABLED     0
#endif

/*****************************************/
/*  I/O Task Priorities                  */
/*****************************************/
#ifndef appconfQSPI_FLASH_TASK_PRIORITY
#define appconfQSPI_FLASH_TASK_PRIORITY		    ( configMAX_PRIORITIES - 1 )
#endif /* appconfQSPI_FLASH_TASK_PRIORITY */

#ifndef appconfI2C_TASK_PRIORITY
#define appconfI2C_TASK_PRIORITY                (configMAX_PRIORITIES/2)
#endif /* appconfI2C_TASK_PRIORITY */

#ifndef appconfSPI_TASK_PRIORITY
#define appconfSPI_TASK_PRIORITY                (configMAX_PRIORITIES/2)
#endif /* appconfSPI_TASK_PRIORITY */

#ifndef appconfDEVICE_CONTROL_I2C_PRIORITY
#define appconfDEVICE_CONTROL_I2C_PRIORITY      (configMAX_PRIORITIES-1)
#endif // appconfDEVICE_CONTROL_I2C_PRIORITY

/*****************************************/
/*  DFU Settings                         */
/*****************************************/
#define FL_QUADDEVICE_W25Q64JW \
{ \
    0,                      /* Just specify 0 as flash_id */ \
    256,                    /* page size */ \
    32768,                  /* num pages */ \
    3,                      /* address size */ \
    4,                      /* log2 clock divider */ \
    0x9F,                   /* QSPI_RDID */ \
    0,                      /* id dummy bytes */ \
    3,                      /* id size in bytes */ \
    0xEF6017,               /* device id (determined from xflash --spi-read-id 0x9F)*/ \
    0x20,                   /* QSPI_SE */ \
    4096,                   /* Sector erase is always 4KB */ \
    0x06,                   /* QSPI_WREN */ \
    0x04,                   /* QSPI_WRDI */ \
    PROT_TYPE_SR,           /* Protection via SR */ \
    {{0x18,0x00},{0,0}},    /* QSPI_SP, QSPI_SU */ \
    0x02,                   /* QSPI_PP */ \
    0xEB,                   /* QSPI_READ_FAST */ \
    1,                      /* 1 read dummy byte */ \
    SECTOR_LAYOUT_REGULAR,  /* mad sectors */ \
    {4096,{0,{0}}},         /* regular sector sizes */ \
    0x05,                   /* QSPI_RDSR */ \
    0x01,                   /* QSPI_WRSR */ \
    0x01,                   /* QSPI_WIP_BIT_MASK */ \
}

#define FL_QUADDEVICE_W25Q32JV \
{ \
    0,                      /* Just specify 0 as flash_id */ \
    256,                    /* page size */ \
    16384,                  /* num pages */ \
    3,                      /* address size */ \
    4,                      /* log2 clock divider */ \
    0x9F,                   /* QSPI_RDID */ \
    0,                      /* id dummy bytes */ \
    3,                      /* id size in bytes */ \
    0xEF4016,               /* device id (determined from xflash --spi-read-id 0x9F)*/ \
    0x20,                   /* QSPI_SE */ \
    4096,                   /* Sector erase is always 4KB */ \
    0x06,                   /* QSPI_WREN */ \
    0x04,                   /* QSPI_WRDI */ \
    PROT_TYPE_SR,           /* Protection via SR */ \
    {{0x18,0x00},{0,0}},    /* QSPI_SP, QSPI_SU */ \
    0x02,                   /* QSPI_PP */ \
    0xEB,                   /* QSPI_READ_FAST */ \
    1,                      /* 1 read dummy byte */ \
    SECTOR_LAYOUT_REGULAR,  /* mad sectors */ \
    {4096,{0,{0}}},         /* regular sector sizes */ \
    0x05,                   /* QSPI_RDSR */ \
    0x01,                   /* QSPI_WRSR */ \
    0x01,                   /* QSPI_WIP_BIT_MASK */ \
}

#define FL_QUADDEVICE_ZB25VQ32D \
{ \
    0,                      /* Just specify 0 as flash_id */ \
    256,                    /* page size */ \
    16384,                  /* num pages */ \
    3,                      /* address size */ \
    4,                      /* log2 clock divider */ \
    0x9F,                   /* QSPI_RDID */ \
    0,                      /* id dummy bytes */ \
    3,                      /* id size in bytes */ \
    0x5E4016,               /* device id (determined from xflash --spi-read-id 0x9F)*/ \
    0x20,                   /* QSPI_SE */ \
    4096,                   /* Sector erase is always 4KB */ \
    0x06,                   /* QSPI_WREN */ \
    0x04,                   /* QSPI_WRDI */ \
    PROT_TYPE_SR,           /* Protection via SR */ \
    {{0x18,0x00},{0,0}},    /* QSPI_SP, QSPI_SU */ \
    0x02,                   /* QSPI_PP */ \
    0xEB,                   /* QSPI_READ_FAST */ \
    1,                      /* 1 read dummy byte */ \
    SECTOR_LAYOUT_REGULAR,  /* mad sectors */ \
    {4096,{0,{0}}},         /* regular sector sizes */ \
    0x05,                   /* QSPI_RDSR */ \
    0x01,                   /* QSPI_WRSR */ \
    0x01,                   /* QSPI_WIP_BIT_MASK */ \
}

#ifndef BOARD_QSPI_SPEC
/* Set up a default SPI spec if the app has not provided
 * one explicitly.
 * Note: The version checks only work in XTC Tools >15.2.0 
 *       By default FL_QUADDEVICE_W25Q64JW is used 
 */
#ifdef __XMOS_XTC_VERSION_MAJOR__
#if (__XMOS_XTC_VERSION_MAJOR__ == 15)      \
    && (__XMOS_XTC_VERSION_MINOR__ >= 2)    \
    && (__XMOS_XTC_VERSION_PATCH__ >= 0)
/* In XTC >15.2.0 some SFDP support enables a generic 
 * default spec
 */
#define BOARD_QSPI_SPEC     FL_QUADDEVICE_DEFAULT
#else
#define BOARD_QSPI_SPEC     FL_QUADDEVICE_W25Q64JW
#endif
#else
#define BOARD_QSPI_SPEC     FL_QUADDEVICE_W25Q64JW
#endif /* __XMOS_XTC_VERSION_MAJOR__ */
#endif /* BOARD_QSPI_SPEC */

#endif /* PLATFORM_CONF_H_ */
