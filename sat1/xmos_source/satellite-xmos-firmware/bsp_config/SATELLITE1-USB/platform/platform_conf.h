// Copyright 2022-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef PLATFORM_CONF_H_
#define PLATFORM_CONF_H_

/*
 * Board support package for FPH Satellite1
 */

#if __has_include("app_conf.h")
#include "app_conf.h"
#endif /* __has_include("app_conf.h") */

/*****************************************/
/* Intertile Communication Configuration */
/*****************************************/

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

#ifndef appconfSPI_DEV_CTRL_PORT
#define appconfSPI_DEV_CTRL_PORT 15
#endif /* appconfSPI_DEV_CTRL_PORT */

#ifndef appconfSPI_DEV_CTRL_PRIORITY
#define appconfSPI_DEV_CTRL_PRIORITY (configMAX_PRIORITIES/2)
#endif /* appconfSPI_DEV_CTRL_PRIORITY */

#ifndef appconfUSB_CDC_PORT
#define appconfUSB_CDC_PORT 16
#endif /* appconfUSP_CDC_PORT */

#ifndef appconfUSB_CDC_PRIORITY
#define appconfUSB_CDC_PRIORITY (configMAX_PRIORITIES/2)
#endif /* appconfSPI_DEV_CTRL_PRIORITY */



/*****************************************/
/*  I/O and interrupt cores for Tile 0   */
/*****************************************/
#ifndef appconfSPI_IO_CORE
#define appconfSPI_IO_CORE                      1 /* Must be kept off core 0 with the RTOS tick ISR */
#endif /* appconfSPI_IO_CORE */

#ifndef appconfSPI_INTERRUPT_CORE
#define appconfSPI_INTERRUPT_CORE               2 /* Must be kept off I/O cores. */
#endif /* appconfSPI_INTERRUPT_CORE */




/*****************************************/
/*  I/O and interrupt cores for Tile 1   */
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


/*****************************************/
/*  I/O Task Priorities                  */
/*****************************************/
#ifndef appconfQSPI_FLASH_TASK_PRIORITY
#define appconfQSPI_FLASH_TASK_PRIORITY		    ( configMAX_PRIORITIES - 1 )
#endif /* appconfQSPI_FLASH_TASK_PRIORITY */

#ifndef appconfSPI_TASK_PRIORITY
#define appconfSPI_TASK_PRIORITY                (configMAX_PRIORITIES/2)
#endif /* appconfSPI_TASK_PRIORITY */

#ifndef appconfDEVICE_CONTROL_SPI_PRIORITY
#define appconfDEVICE_CONTROL_SPI_PRIORITY      (configMAX_PRIORITIES-2)
#endif // appconfDEVICE_CONTROL_SPI_PRIORITY


/*****************************************/
/*  CONFIGURATION   */
/*****************************************/

#ifndef appconfAUDIO_CLOCK_FREQUENCY
#define appconfAUDIO_CLOCK_FREQUENCY        MIC_ARRAY_CONFIG_MCLK_FREQ
#endif /* appconfAUDIO_CLOCK_FREQUENCY */

#ifndef appconfPDM_CLOCK_FREQUENCY
#define appconfPDM_CLOCK_FREQUENCY          MIC_ARRAY_CONFIG_PDM_FREQ
#endif /* appconfPDM_CLOCK_FREQUENCY */

#ifndef appconfPIPELINE_AUDIO_SAMPLE_RATE
#define appconfPIPELINE_AUDIO_SAMPLE_RATE   16000
#endif /* appconfPIPELINE_AUDIO_SAMPLE_RATE */

#ifndef appconfDEVICE_CTRL_SPI
#define appconfDEVICE_CTRL_SPI    0
#endif /* appconfDEVICE_CTRL_SPI */

#ifndef APP_CONTROL_TRANSPORT_COUNT
#define APP_CONTROL_TRANSPORT_COUNT (appconfDEVICE_CTRL_SPI)
#endif // APP_CONTROL_TRANSPORT_COUNT

#ifndef appconfEXTERNAL_MCLK
#define appconfEXTERNAL_MCLK       0
#endif /* appconfEXTERNAL_MCLK */

#ifndef appconfI2S_MODE_MASTER
#define appconfI2S_MODE_MASTER     0
#endif /* appconfI2S_MODE_MASTER */
#ifndef appconfI2S_MODE_SLAVE
#define appconfI2S_MODE_SLAVE      1
#endif /* appconfI2S_MODE_SLAVE */
#ifndef appconfI2S_MODE
#define appconfI2S_MODE            appconfI2S_MODE_MASTER
#endif /* appconfI2S_MODE */


/*****************************************/
/*  DFU Settings                         */
/*****************************************/
#define FL_QUADDEVICE_W25Q64JV \
{ \
    0,                      /* Just specify 0 as flash_id */ \
    256,                    /* page size */ \
    32768,                  /* num pages */ \
    3,                      /* address size */ \
    4,                      /* log2 clock divider */ \
    0x9F,                   /* QSPI_RDID */ \
    0,                      /* id dummy bytes */ \
    3,                      /* id size in bytes */ \
    0xEF4017,               /* device id (determined from xflash --spi-read-id 0x9F)*/ \
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
 *       By default FL_QUADDEVICE_W25Q64JV is used
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
#define BOARD_QSPI_SPEC     FL_QUADDEVICE_W25Q64JV
#endif
#else
#define BOARD_QSPI_SPEC     FL_QUADDEVICE_W25Q64JV
#endif /* __XMOS_XTC_VERSION_MAJOR__ */
#endif /* BOARD_QSPI_SPEC */

#endif /* PLATFORM_CONF_H_ */
