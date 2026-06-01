// Copyright 2016-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#define DEBUG_UNIT CONTROL
#include <rtos_printf.h>
#include <string.h>

#include "rtos_osal.h"

#include "device_control.h"

#define MAX_STATUS_BUFFER_LEN 10

typedef struct {
    int cmd;
    void *rsvd;
    uint8_t *data;
} req_from_servicer_t;

typedef struct {
    rtos_osal_queue_t *queue;
    size_t num_resources;
    control_resid_t resources[];
} servicer_init_data_t;

typedef struct {
    device_control_t *dev_ctrl_ctx;
    rtos_osal_queue_t *queue;
    size_t payload_len;
    control_resid_t resid;
    control_cmd_t cmd;
    uint8_t *payload;
    uint8_t buf[];
} cmd_to_servicer_t;

void resource_table_init(device_control_t *ctx);

int resource_table_add(device_control_t *ctx,
                       const control_resid_t resources[],
                       size_t num_resources,
                       uint8_t servicer);

int resource_table_search(device_control_t *ctx,
                          control_resid_t resid,
                          uint8_t *servicer);

control_ret_t device_control_servicer_cmd_recv(device_control_servicer_t *ctx,
                                               DEVICE_CONTROL_CALLBACK_ATTR device_control_read_cmd_cb_t read_cmd_cb,
                                               DEVICE_CONTROL_CALLBACK_ATTR device_control_write_cmd_cb_t write_cmd_cb,
                                               void *app_data,
                                               unsigned timeout)
{
    rtos_osal_status_t status;
    control_ret_t ret = CONTROL_ERROR;
    cmd_to_servicer_t *c_ptr;

    status = rtos_osal_queue_receive(&ctx->queue, &c_ptr, timeout);
    if (status == RTOS_OSAL_SUCCESS) {

        device_control_t *device_control_ctx = c_ptr->dev_ctrl_ctx;

        if (IS_CONTROL_CMD_READ(c_ptr->cmd)) {
            ret = read_cmd_cb(c_ptr->resid, c_ptr->cmd, c_ptr->payload, c_ptr->payload_len, app_data);
        } else {
            ret = write_cmd_cb(c_ptr->resid, c_ptr->cmd, c_ptr->payload, c_ptr->payload_len, app_data);
        }

        if (device_control_ctx->resource_table != NULL) {
            status = rtos_osal_queue_send(&device_control_ctx->gateway_queue, &ret, 0); /* This should not block. As long as everything
                                                                                           is working as designed, this queue will always
                                                                                           be empty here. */
            xassert(status == RTOS_OSAL_SUCCESS);
        } else {
            /* gateway is on another tile */

            if (IS_CONTROL_CMD_READ(c_ptr->cmd)) {
                rtos_intertile_tx_len(device_control_ctx->host_intertile, device_control_ctx->intertile_port, sizeof(ret) + c_ptr->payload_len);
                rtos_intertile_tx_data(device_control_ctx->host_intertile, &ret, sizeof(ret));
                rtos_intertile_tx_data(device_control_ctx->host_intertile, c_ptr->payload, c_ptr->payload_len);
                /*
                 * the thread that received this over the intertile channel
                 * malloc'd this buffer, so it must be freed here.
                 */
                rtos_osal_free(c_ptr->payload);
            } else {
                rtos_intertile_tx(device_control_ctx->host_intertile, device_control_ctx->intertile_port, &ret, sizeof(ret));
            }

            /*
             * the thread that received this over the intertile channel
             * malloc'd this buffer, so it must be freed here.
             * In the above case where it came from the same tile, this
             * is still owned by the application.
             */
            rtos_osal_free(c_ptr);
        }
    }
    return status == RTOS_OSAL_SUCCESS ? CONTROL_SUCCESS : CONTROL_ERROR;
}

static void device_control_client_thread(device_control_t *ctx)
{
    uint32_t msg_length;
    cmd_to_servicer_t *c_ptr;

    for (;;) {
        msg_length = rtos_intertile_rx(ctx->host_intertile,
                                       ctx->intertile_port,
                                       (void **) &c_ptr,
                                       RTOS_OSAL_WAIT_FOREVER);

        c_ptr->dev_ctrl_ctx = ctx;

        if (msg_length != 0) {

            if (IS_CONTROL_CMD_READ(c_ptr->cmd)) {
                xassert(msg_length == sizeof(cmd_to_servicer_t));
                xassert(c_ptr->payload_len > 0);
                /* Allocate a buffer for the read data to be written to */
                c_ptr->payload = rtos_osal_malloc(c_ptr->payload_len);
            } else {
                xassert(msg_length >= sizeof(cmd_to_servicer_t));
                xassert(c_ptr->payload_len == msg_length - sizeof(cmd_to_servicer_t));
                c_ptr->payload = c_ptr->buf;
            }

            rtos_osal_queue_send(c_ptr->queue, &c_ptr, RTOS_OSAL_WAIT_FOREVER);
        }
    }
}

static control_ret_t special_read_command(device_control_t *ctx,
                                          control_cmd_t cmd,
                                          uint8_t payload[],
                                          unsigned payload_len)
{
    switch (cmd) {
    case CONTROL_GET_VERSION:
        rtos_printf("read version %d\n", CONTROL_VERSION);
        if (payload_len != sizeof(control_version_t)) {
            rtos_printf("wrong payload size %d for read version command, need %d\n",
                    payload_len, sizeof(control_version_t));

            return CONTROL_BAD_COMMAND;
        } else {
            *((control_version_t*) payload) = CONTROL_VERSION;
            return CONTROL_SUCCESS;
        }

    case CONTROL_GET_LAST_COMMAND_STATUS:
        rtos_printf("read last command status %d\n", ctx->last_status);
        if (payload_len != sizeof(control_status_t)) {
            rtos_printf("wrong payload size %d for read version command, need %d\n",
                    payload_len, sizeof(control_version_t));

            return CONTROL_BAD_COMMAND;
        } else {
            *((control_status_t*) payload) = ctx->last_status;
            return CONTROL_SUCCESS;
        }

    default:
        rtos_printf("unrecognised special resource command %d\n", cmd);
        return CONTROL_BAD_COMMAND;
    }
}

static control_ret_t do_command(device_control_t *ctx,
                                uint8_t servicer,
                                control_resid_t resid,
                                control_cmd_t cmd,
                                uint8_t payload[],
                                unsigned payload_len)
{
    control_ret_t ret;

    if (resid == CONTROL_SPECIAL_RESID) {
        if (IS_CONTROL_CMD_READ(cmd)) {
            return special_read_command(ctx, cmd, payload, payload_len);
        } else {
            rtos_printf("ignoring write to special resource %d\n", CONTROL_SPECIAL_RESID);
            return CONTROL_BAD_COMMAND;
        }
    } else {

        rtos_intertile_t *intertile_ctx = ctx->servicer_table[servicer].intertile_ctx;
        rtos_osal_queue_t *queue = ctx->servicer_table[servicer].queue;

        cmd_to_servicer_t c = {
                .dev_ctrl_ctx = ctx,
                .queue = queue,
                .resid = resid,
                .cmd = cmd,
                .payload_len = payload_len,
                .payload = payload,
                .buf = {}
        };
        cmd_to_servicer_t *c_ptr = &c;

        if (intertile_ctx == NULL) { /* on tile case */

            rtos_osal_queue_send(queue, &c_ptr, RTOS_OSAL_WAIT_FOREVER);
            rtos_osal_queue_receive(&ctx->gateway_queue, &ret, RTOS_OSAL_WAIT_FOREVER);

        } else { /* off tile case */
            size_t xfer_len = sizeof(cmd_to_servicer_t);

            if (!IS_CONTROL_CMD_READ(cmd)) {
                xfer_len += payload_len;
            }

            rtos_intertile_tx_len(intertile_ctx, ctx->intertile_port, xfer_len);
            rtos_intertile_tx_data(intertile_ctx, c_ptr, sizeof(cmd_to_servicer_t));
            if (!IS_CONTROL_CMD_READ(cmd)) {
                rtos_intertile_tx_data(intertile_ctx, payload, payload_len);
            }

            xfer_len = rtos_intertile_rx_len(intertile_ctx, ctx->intertile_port, RTOS_OSAL_WAIT_FOREVER);

            if (xfer_len >= sizeof(ret)) {
                rtos_intertile_rx_data(intertile_ctx, &ret, sizeof(ret));

                if (IS_CONTROL_CMD_READ(cmd)) {
                    xassert(payload_len == xfer_len - sizeof(ret));
                    rtos_intertile_rx_data(intertile_ctx, payload, payload_len);
                } else {
                    xassert(xfer_len == sizeof(ret));
                }

            } else {
                ret = CONTROL_ERROR;
            }
        }

        if (IS_CONTROL_CMD_READ(cmd)) {
            rtos_printf("%d read command %d, %d, %d\n", servicer, resid, cmd, payload_len);
        } else {
            rtos_printf("%d write command %d, %d, %d\n", servicer, resid, cmd, payload_len);
        }

        return ret;
    }
}

void device_control_payload_transfer_bidir(device_control_t *ctx,
                                              uint8_t *rx_buf,
                                              const size_t rx_size,
                                              uint8_t *tx_buf,
                                              size_t *tx_size)
{
    control_ret_t ret;
    uint8_t servicer;

    const size_t requested_payload_len = ctx->requested_payload_len;
    const control_resid_t requested_resid = ctx->requested_resid;
    const control_cmd_t requested_cmd = ctx->requested_cmd;

    if (resource_table_search(ctx, requested_resid, &servicer) == 0) {
        if(!IS_CONTROL_CMD_READ(requested_cmd)) // Write command
        {
            if(requested_payload_len > rx_size)
            {
                tx_buf[0] = CONTROL_DATA_LENGTH_ERROR;
                ret = CONTROL_DATA_LENGTH_ERROR;
            }
            else
            {
                ret = do_command(ctx, servicer, requested_resid, requested_cmd, rx_buf, requested_payload_len);
                tx_buf[0] = ret;
            }
        }
        else // Read command
        {
            rtos_printf("do_read_command(), requested_resid %d, requested_cmd %d, requested_payload_len %d\n", requested_resid, requested_cmd, requested_payload_len);
            ret = do_command(ctx, servicer, requested_resid, requested_cmd, tx_buf, requested_payload_len);
            //tx_buf[0] = ret;
            *tx_size = requested_payload_len;
        }
    }
    else // Resource not found
    {
        rtos_printf("resource %d not found\n", requested_resid);
        tx_buf[0] = CONTROL_BAD_RESOURCE;
        ret = CONTROL_BAD_RESOURCE;
    }

    ctx->last_status = ret;
}

control_ret_t device_control_payload_transfer(device_control_t *ctx,
                                              uint8_t *payload_buf,
                                              size_t *buf_size,
                                              control_direction_t direction)
{
    control_ret_t ret;
    uint8_t servicer;

    const size_t requested_payload_len = ctx->requested_payload_len;
    const control_resid_t requested_resid = ctx->requested_resid;
    const control_cmd_t requested_cmd = ctx->requested_cmd;

    if (resource_table_search(ctx, requested_resid, &servicer) == 0) {
        if(direction == CONTROL_HOST_TO_DEVICE) //Write request from host
        {
            if(!IS_CONTROL_CMD_READ(requested_cmd)) // Write request for a write command
            {
                rtos_printf("Write request for write command %d\n", requested_cmd);
                // Forward the command to the servicer. Save the status
                if (requested_payload_len <= *buf_size) {
                    ret = do_command(ctx, servicer, requested_resid, requested_cmd, payload_buf, requested_payload_len);
                }
                else {
                    ret = CONTROL_DATA_LENGTH_ERROR;
                }
                // Save status to return if the host follows up with a read request
                ctx->last_status = ret;
            }
            else // Write request for a read command
            {
                rtos_printf("Write request for read command %d\n", requested_cmd);
                // Do nothing. Command only forwarded to the servicer when there's a read request for a read command
                // Set last_status to Success
                ctx->last_status = CONTROL_SUCCESS;
            }
        }
        else //Read request from host
        {
            if(!IS_CONTROL_CMD_READ(requested_cmd)) // Read request for a write command
            {
                rtos_printf("Read request for write command %d\n", requested_cmd);
                // Return status
                payload_buf[0] = ctx->last_status;
            }
            else // Read request for a read command
            {
                rtos_printf("Read request for read command %d\n", requested_cmd);
                // Forward the command to the servicer. Update returned status in the first byte of payload
                ret = do_command(ctx, servicer, requested_resid, requested_cmd, payload_buf, requested_payload_len);
                //payload_buf[0] = ret;
            }

        }

    }
    else 
    {
        ctx->last_status = CONTROL_BAD_RESOURCE;
        if(direction == CONTROL_DEVICE_TO_HOST) //read request from host
        {
            payload_buf[0] = ctx->last_status;
        }
    }

    return CONTROL_SUCCESS;
}

control_ret_t device_control_request(device_control_t *ctx,
                                     control_resid_t resid,
                                     control_cmd_t cmd,
                                     size_t payload_len)
{
    control_ret_t ret;
    uint8_t servicer;

    ctx->requested_resid = resid;

    if (resource_table_search(ctx, resid, &servicer) == 0) {
        ctx->requested_cmd = cmd;
        ctx->requested_payload_len = payload_len;
        ret = CONTROL_SUCCESS;
    } else {
        ret = CONTROL_BAD_COMMAND;
    }

    return ret;
}

control_ret_t device_control_set_resource_status( device_control_t *ctx,
                                                  uint8_t status_id,
                                                  uint8_t resource_status )
{
    if ( ctx->resource_table == NULL ) /* off tile case */
    {  
        rtos_printf("Attempt to write status buffer from 'off tile'\n");
    } else { /* on tile case */
        xassert( status_id < ctx->status_buffer_len);
        ctx->status_buffer[status_id] = resource_status;
    }
    return CONTROL_SUCCESS;
}




control_ret_t device_control_servicer_register(device_control_servicer_t *ctx,
                                               device_control_t *device_control_ctx[],
                                               size_t device_control_ctx_count,
                                               const control_resid_t resources[],
                                               size_t num_resources)
{
    const size_t len = sizeof(servicer_init_data_t) + sizeof(control_resid_t) * num_resources;

    rtos_osal_queue_create(&ctx->queue, "servicer_q", 1, sizeof(void *));

    /*
     * TODO: Perhaps wait for an ACK. Then this would not need malloc()
     * and would also detect failures.
     */

    for (int i = 0; i < device_control_ctx_count; i++) {
        servicer_init_data_t *init_data = rtos_osal_malloc(len);

        init_data->num_resources = num_resources;
        init_data->queue = &ctx->queue;
        memcpy(init_data->resources, resources, sizeof(control_resid_t) * num_resources);

        if (device_control_ctx[i]->resource_table != NULL) {

            rtos_osal_queue_send(&device_control_ctx[i]->gateway_queue, &init_data, RTOS_OSAL_WAIT_FOREVER);

        } else {
            /* Resource table is NULL on client tiles */

            rtos_intertile_tx(device_control_ctx[i]->host_intertile, device_control_ctx[i]->intertile_port, init_data, len);
            rtos_osal_free(init_data);
        }
    }

    return CONTROL_SUCCESS;
}

static int servicer_register(device_control_t *ctx,
                              servicer_init_data_t *init_cmd,
                              rtos_intertile_t *intertile_ctx,
                              int servicer_index)
{
    int ret;
    ctx->servicer_table[servicer_index].queue = init_cmd->queue;
    ctx->servicer_table[servicer_index].intertile_ctx = intertile_ctx;
    ret = resource_table_add(ctx, init_cmd->resources, init_cmd->num_resources, servicer_index);
    rtos_osal_free(init_cmd);
    return ret;
}

control_ret_t device_control_resources_register(device_control_t *ctx,
                                                unsigned timeout)
{
    servicer_init_data_t *init_cmd;
    int registered_count = 0;
    int ret = 0;
    rtos_osal_tick_t start_time;

    ctx->servicer_table = rtos_osal_malloc(ctx->servicer_count * sizeof(*ctx->servicer_table));
    start_time = rtos_osal_tick_get();
    while (registered_count < ctx->servicer_count && ret == 0 && rtos_osal_tick_get() - start_time < timeout) {

        if (rtos_osal_queue_receive(&ctx->gateway_queue, &init_cmd, 1) == RTOS_OSAL_SUCCESS) {
            ret = servicer_register(ctx, init_cmd, NULL, registered_count);
            if (ret == 0) {
                registered_count++;
            }
        }

        for (int i = 0; i < ctx->intertile_count && registered_count < ctx->servicer_count && ret == 0; i++) {
            if (rtos_intertile_rx(ctx->client_intertile[i], ctx->intertile_port, (void **) &init_cmd, 1) != 0) {
                ret = servicer_register(ctx, init_cmd, ctx->client_intertile[i], registered_count);
                if (ret == 0) {
                    registered_count++;
                }
            }
        }
    }

    if (registered_count == ctx->servicer_count) {
        ctx->status_buffer = rtos_osal_malloc( registered_count );
        ctx->status_buffer_len = MAX_STATUS_BUFFER_LEN;
        memset( ctx->status_buffer, 0, ctx->status_buffer_len );
        return CONTROL_SUCCESS;
    } else {
        rtos_printf("Device-Control: %d servicer registered, %d expexted\n", registered_count, ctx->servicer_count );
        return CONTROL_REGISTRATION_FAILED;
    }
}

control_ret_t device_control_start(device_control_t *ctx,
                                   uint8_t intertile_port,
                                   unsigned priority)
{
    control_ret_t ret;

    ctx->intertile_port = intertile_port;

    if (ctx->resource_table == NULL) {
        /* Resource table is NULL on client tiles */
        rtos_osal_status_t status;

        status = rtos_osal_thread_create(
                NULL,
                "dc_client",
                (rtos_osal_entry_function_t) device_control_client_thread,
                ctx,
                RTOS_THREAD_STACK_SIZE(device_control_client_thread),
                priority);

        if (status == RTOS_OSAL_SUCCESS) {
            ret = CONTROL_SUCCESS;
        } else {
            ret = CONTROL_REGISTRATION_FAILED;
        }

    } else {
        rtos_osal_queue_create(&ctx->gateway_queue, "dc_gw_q", 1, sizeof(void *));
        ret = CONTROL_SUCCESS;
    }

    return ret;
}

control_ret_t device_control_init(device_control_t *ctx,
                                  int mode,
                                  size_t servicer_count,
                                  rtos_intertile_t *intertile_ctx[],
                                  size_t intertile_count)
{
    if (mode == DEVICE_CONTROL_HOST_MODE) {
        memset(ctx, 0, sizeof(device_control_t));
        resource_table_init(ctx);

        ctx->servicer_count = servicer_count;

        xassert(intertile_count <= 3);
        if (intertile_count > 3) {
            return CONTROL_REGISTRATION_FAILED;
        }
        ctx->intertile_count = intertile_count;
        for (int i = 0; i < intertile_count; i++) {
            ctx->client_intertile[i] = intertile_ctx[i];
        }
    } else {
        memset(ctx, 0, sizeof(device_control_client_t));
        xassert(intertile_count == 1);
        if (intertile_count != 1) {
            return CONTROL_REGISTRATION_FAILED;
        }

        ctx->host_intertile = intertile_ctx[0];
    }

    return CONTROL_SUCCESS;
}
