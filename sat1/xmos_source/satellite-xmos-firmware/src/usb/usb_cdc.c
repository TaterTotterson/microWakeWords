#include "usb_cdc.h"
#include "rtos_rpc.h"
#include <stdio.h>
#include <stdarg.h>

#if appconfUSB_CDC_ENABLED
static rtos_driver_rpc_t rpc_config_s;
static rtos_driver_rpc_t* rpc_config = &rpc_config_s;


enum {
    fcode_rx,
    fcode_tx,
};
#endif


// Callback when CDC data is received from the host
void tud_cdc_rx_cb(uint8_t itf) {
#if appconfUSB_CDC_ENABLED
    char buf[128] = "received: ";

    // Read data from the host
    uint32_t count = tud_cdc_read(buf + 10, sizeof(buf) - 10);

    // Echo back to the host
    tud_cdc_write(buf, 10 + count);
    tud_cdc_write_flush();
#endif
}


int cdc_printf(const char *format, ...) {
#if appconfUSB_CDC_ENABLED    
    char buffer[256];  // Buffer to store the formatted output
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);  // Format the string
    va_end(args);
    
    if (rpc_config->remote_client_count == 0) {
        /* This is a RPC client */
        
        rtos_intertile_address_t *host_address = &rpc_config->host_address;

        xassert(host_address->port >= 0);
        const rpc_param_desc_t rpc_param_desc[] = {
            RPC_PARAM_IN_BUFFER(buffer, len),
            RPC_PARAM_TYPE(len),
            RPC_PARAM_LIST_END
        };

        rpc_client_call_generic(
            host_address->intertile_ctx, host_address->port, fcode_tx, rpc_param_desc,
            &buffer, &len);    

    } else {
        /* This is the RPC host */
        
        if (!tud_cdc_connected()) {
            return 0;  // Return early if not connected
        }
        if (len > 0) {
            tud_cdc_write(buffer, len);  // Send formatted output through USB CDC
            tud_cdc_write_flush();       // Ensure data is sent immediately
        } else {
            const char *error_msg = "Error: Failed to format string\n";
            tud_cdc_write(error_msg, strlen(error_msg));
            tud_cdc_write_flush();
        }           
    }
    
    return len;  // Return the number of characters sent
#else 
    return 0;
#endif
}


#if appconfUSB_CDC_ENABLED    
static int cdc_tx_rpc_host(rpc_msg_t *rpc_msg, uint8_t **resp_msg)
{
    int msg_length;
    char *char_buf;
    size_t len;

    rpc_request_unmarshall(
            rpc_msg,
            &char_buf, &len);

    if (len > 0) {
        tud_cdc_write(char_buf, len);  // Send formatted output through USB CDC
        tud_cdc_write_flush();       // Ensure data is sent immediately
    } else {
            const char *error_msg = "Error: Failed to format string\n";
            tud_cdc_write(error_msg, strlen(error_msg));
            tud_cdc_write_flush();
    }           

    msg_length = rpc_response_marshall(
            resp_msg, rpc_msg,
            char_buf, len);

    return msg_length;
}


static void cdc_rpc_thread(rtos_intertile_address_t *client_address)
{
    int msg_length;
    uint8_t *req_msg;
    uint8_t *resp_msg;
    rpc_msg_t rpc_msg;
    rtos_intertile_t *intertile_ctx = client_address->intertile_ctx;
    uint8_t intertile_port = client_address->port;

    for (;;) {
        /* receive RPC request message from client */
        msg_length = rtos_intertile_rx(intertile_ctx, intertile_port, (void **) &req_msg, RTOS_OSAL_WAIT_FOREVER);

        rpc_request_parse(&rpc_msg, req_msg);

        switch (rpc_msg.fcode) {
        case fcode_tx:
            msg_length = cdc_tx_rpc_host(&rpc_msg, &resp_msg);
            break;
        }

        rtos_osal_free(req_msg);

        /* send RPC response message to client */
        rtos_intertile_tx(intertile_ctx, intertile_port, resp_msg, msg_length);
        rtos_osal_free(resp_msg);
    }
}



__attribute__((fptrgroup("rtos_driver_rpc_host_start_fptr_grp")))
static void cdc_rpc_start(
        rtos_driver_rpc_t *rpc_config_)
{
    xassert(rpc_config_->host_task_priority >= 0);

    for (int i = 0; i < rpc_config_->remote_client_count; i++) {

        rtos_intertile_address_t *client_address = &rpc_config_->client_address[i];

        xassert(client_address->port >= 0);

        rtos_osal_thread_create(
                NULL,
                "cdc_rpc_thread",
                (rtos_osal_entry_function_t) cdc_rpc_thread,
                client_address,
                RTOS_THREAD_STACK_SIZE(cdc_rpc_thread),
                rpc_config_->host_task_priority);
    }
}

void rtos_cdc_start()
{
    if (rpc_config != NULL && rpc_config->rpc_host_start != NULL) {
        rpc_config->rpc_host_start(rpc_config);
    }
}


void rtos_cdc_rpc_config(
        unsigned intertile_port,
        unsigned host_task_priority)
{

    if (rpc_config->remote_client_count == 0) {
        /* This is a client */
        rpc_config->host_address.port = intertile_port;
    } else {
        for (int i = 0; i < rpc_config->remote_client_count; i++) {
            rpc_config->client_address[i].port = intertile_port;
        }
        rpc_config->host_task_priority = host_task_priority;
    }
}


void rtos_cdc_rpc_client_init(rtos_intertile_t *host_intertile_ctx)
{
    rpc_config->rpc_host_start = NULL;
    rpc_config->remote_client_count = 0;
    rpc_config->host_task_priority = -1;

    /* This must be configured later with rtos_i2s_rpc_config() */
    rpc_config->host_address.port = -1;

    rpc_config->host_address.intertile_ctx = host_intertile_ctx;
}



void rtos_cdc_rpc_host_init(
        rtos_intertile_t *client_intertile_ctx[],
        size_t remote_client_count)
{
    rpc_config->rpc_host_start = cdc_rpc_start;
    rpc_config->remote_client_count = remote_client_count;

    /* This must be configured later with rtos_i2s_rpc_config() */
    rpc_config->host_task_priority = -1;

    for (int i = 0; i < remote_client_count; i++) {
        rpc_config->client_address[i].intertile_ctx = client_intertile_ctx[i];
        /* This must be configured later with rtos_i2s_rpc_config() */
        rpc_config->client_address[i].port = -1;
    }
}
#endif
