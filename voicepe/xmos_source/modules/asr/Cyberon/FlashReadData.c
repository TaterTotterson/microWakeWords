// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <stdint.h>
#include <xs1.h>
#include <xcore/assert.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"

/* Library headers */
#include "asr.h"
#include "rtos_qspi_flash.h"

/* App headers */
#include "platform/driver_instances.h"
#include "FlashReadData.h"

#define DBG_TRACE                 // Output log by DbgTrace(), if not defined only output by rtos_printf.
#include "DbgTrace.h"

extern devmem_manager_t *devmem_ctx;

size_t FlashReadData(void * dst, const void * src, size_t size)
{
    devmem_read_ext(devmem_ctx, dst, src, size);
	return size;
}

