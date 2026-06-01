#pragma once

#include "tusb.h"
#include "rtos_intertile.h"
#include "rtos_driver_rpc.h"


int cdc_printf(const char *format, ...);



void rtos_cdc_rpc_client_init(rtos_intertile_t *host_intertile_ctx);

void rtos_cdc_rpc_host_init(
        rtos_intertile_t *client_intertile_ctx[],
        size_t remote_client_count);

void rtos_cdc_rpc_config(
        unsigned intertile_port,
        unsigned host_task_priority);


void rtos_cdc_start();