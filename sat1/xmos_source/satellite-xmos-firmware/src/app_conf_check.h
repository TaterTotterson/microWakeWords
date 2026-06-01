// Copyright 2021-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef APP_CONF_CHECK_H_
#define APP_CONF_CHECK_H_

#if appconfUSB_ENABLED && appconfSPI_OUTPUT_ENABLED
#error Cannot use both USB and SPI interfaces
#endif

#if appconfUSB_ENABLED && appconfEXTERNAL_MCLK
#error Cannot use USB with an external mclk source
#endif

#if appconfUSB_ENABLED && appconfINTENT_ENABLED
#error Cannot use wakeword engine in USB configurations
#endif

#if appconfI2S_TDM_ENABLED
#error TDM mode not longer supported in this firmware version
#endif

#if (appconfI2S_MODE == appconfI2S_MODE_SLAVE)
#error I2S_MODE_SLAVE not longer supported in this firmware version
#endif


#endif /* APP_CONF_CHECK_H_ */
