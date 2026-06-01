// Copyright 2016-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#define DEBUG_UNIT RESOURCE_TABLE
#include <rtos_printf.h>

#include <stdint.h>
#include <stddef.h>
#include "device_control.h"

#define RESOURCE_TABLE_SIZE 256
#define NO_SERVICER         255

int resource_table_add(device_control_t *ctx,
                       const control_resid_t resources[],
                       size_t num_resources,
                       uint8_t servicer)
{
    control_resid_t resid;

    if (servicer == NO_SERVICER) {
        rtos_printf("cannot use reserved servicer number %d\n", NO_SERVICER);
        return 1;
    }

    for (size_t i = 0; i < num_resources; i++) {
        resid = resources[i];

        rtos_printf("register resource %d for servicer %d\n", resid, servicer);

        if (resid == CONTROL_SPECIAL_RESID) {
            rtos_printf("can't use reserved resource number %d\n", CONTROL_SPECIAL_RESID);
            return 2;
        }

        if (ctx->resource_table[resid] != NO_SERVICER) {
            rtos_printf("resource %d already registered for servicer %d\n", resid, ctx->resource_table[resid]);
            return 3;
        }

        ctx->resource_table[resid] = servicer;
    }

    return 0;
}

int resource_table_search(device_control_t *ctx,
                          control_resid_t resid,
                          uint8_t *servicer)
{
    *servicer = ctx->resource_table[resid];

    if (resid == CONTROL_SPECIAL_RESID || ctx->resource_table[resid] != NO_SERVICER) {
        return 0;
    } else {
        return 1;
    }
}

void resource_table_init(device_control_t *ctx)
{
    ctx->resource_table = rtos_osal_malloc(sizeof(ctx->resource_table[0]) * RESOURCE_TABLE_SIZE);

    for (size_t i = 0; i < RESOURCE_TABLE_SIZE; i++) {
        ctx->resource_table[i] = NO_SERVICER;
    }
}
