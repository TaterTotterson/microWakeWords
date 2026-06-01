// Copyright 2020-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include "rtos_rpc.h"
#include "rtos_i2s.h"

enum {
    fcode_rx,
    fcode_tx,
};

__attribute__((fptrgroup("rtos_i2s_rx_fptr_grp")))
static size_t i2s_remote_rx(
        rtos_i2s_t *ctx,
        int32_t *i2s_sample_buf,
        size_t frame_count,
        unsigned timeout)
{
    rtos_intertile_address_t *host_address = &ctx->rpc_config->host_address;
    rtos_i2s_t *host_ctx_ptr = ctx->rpc_config->host_ctx_ptr;
    size_t ret;

    const rpc_param_desc_t rpc_param_desc[] = {
            RPC_PARAM_TYPE(ctx),
            RPC_PARAM_OUT_BUFFER(i2s_sample_buf, frame_count * (2 * ctx->num_in)),
            RPC_PARAM_TYPE(frame_count),
            RPC_PARAM_TYPE(timeout),
            RPC_PARAM_RETURN(size_t),
            RPC_PARAM_LIST_END
    };

    rtos_osal_mutex_get(&ctx->mutex, RTOS_OSAL_WAIT_FOREVER);

    rpc_client_call_generic(
            host_address->intertile_ctx, host_address->port, fcode_rx, rpc_param_desc,
            &host_ctx_ptr, i2s_sample_buf, &frame_count, &timeout, &ret);

    rtos_osal_mutex_put(&ctx->mutex);

    return ret;
}

__attribute__((fptrgroup("rtos_i2s_tx_fptr_grp")))
static size_t i2s_remote_tx(
        rtos_i2s_t *ctx,
        int32_t *i2s_sample_buf,
        size_t frame_count,
        unsigned timeout)
{
    rtos_intertile_address_t *host_address = &ctx->rpc_config->host_address;
    rtos_i2s_t *host_ctx_ptr = ctx->rpc_config->host_ctx_ptr;
    size_t ret;
    int out_select = 0;
    const rpc_param_desc_t rpc_param_desc[] = {
            RPC_PARAM_TYPE(ctx),
            RPC_PARAM_IN_BUFFER(i2s_sample_buf, frame_count * 2),
            RPC_PARAM_TYPE(frame_count),
            RPC_PARAM_TYPE(timeout),
            RPC_PARAM_TYPE(out_select),
            RPC_PARAM_RETURN(size_t),
            RPC_PARAM_LIST_END
    };

    rtos_osal_mutex_get(&ctx->mutex, RTOS_OSAL_WAIT_FOREVER);

    rpc_client_call_generic(
            host_address->intertile_ctx, host_address->port, fcode_tx, rpc_param_desc,
            &host_ctx_ptr, i2s_sample_buf, &frame_count, &timeout, &out_select, &ret);

    rtos_osal_mutex_put(&ctx->mutex);

    return ret;
}

__attribute__((fptrgroup("rtos_i2s_tx_fptr_grp")))
static size_t i2s_remote_1_tx(
        rtos_i2s_t *ctx,
        int32_t *i2s_sample_buf,
        size_t frame_count,
        unsigned timeout)
{
    rtos_intertile_address_t *host_address = &ctx->rpc_config->host_address;
    rtos_i2s_t *host_ctx_ptr = ctx->rpc_config->host_ctx_ptr;
    size_t ret;
    int out_select = 1;
    const rpc_param_desc_t rpc_param_desc[] = {
            RPC_PARAM_TYPE(ctx),
            RPC_PARAM_IN_BUFFER(i2s_sample_buf, frame_count * 2),
            RPC_PARAM_TYPE(frame_count),
            RPC_PARAM_TYPE(timeout),
            RPC_PARAM_TYPE(out_select),
            RPC_PARAM_RETURN(size_t),
            RPC_PARAM_LIST_END
    };

    rtos_osal_mutex_get(&ctx->mutex, RTOS_OSAL_WAIT_FOREVER);

    rpc_client_call_generic(
            host_address->intertile_ctx, host_address->port, fcode_tx, rpc_param_desc,
            &host_ctx_ptr, i2s_sample_buf, &frame_count, &timeout, &out_select, &ret);

    rtos_osal_mutex_put(&ctx->mutex);

    return ret;
}


static int i2s_rx_rpc_host(rpc_msg_t *rpc_msg, uint8_t **resp_msg)
{
    int msg_length;

    rtos_i2s_t *i2s_ctx;
    int32_t *i2s_sample_buf;
    size_t frame_count;
    unsigned timeout;
    size_t ret;
    
    /* Here, because the buffer is output only, the buffer
    pointer will not get set. */
    rpc_request_unmarshall(
            rpc_msg,
            &i2s_ctx, &i2s_sample_buf, &frame_count, &timeout, &ret);

    /* Instead allocate a buffer with the length parameter
    before calling, as this is what would be done by
    the client. */
    i2s_sample_buf = rtos_osal_malloc(frame_count * (2 * i2s_ctx->num_out) * sizeof(i2s_sample_buf[0]));

    ret = rtos_i2s_rx(i2s_ctx, i2s_sample_buf, frame_count, timeout);

    msg_length = rpc_response_marshall(
            resp_msg, rpc_msg,
            i2s_ctx, i2s_sample_buf, frame_count, timeout, ret);

    /* The data from buffer has been copied into the response
    message. Since buffer was allocated on the heap, free
    it now. */
    rtos_osal_free(i2s_sample_buf);

    return msg_length;
}

static int i2s_tx_rpc_host(rpc_msg_t *rpc_msg, uint8_t **resp_msg)
{
    int msg_length;

    rtos_i2s_t *i2s_ctx;
    int32_t *i2s_sample_buf;
    size_t frame_count;
    unsigned timeout;
    size_t ret;
    int out_select;

    rpc_request_unmarshall(
            rpc_msg,
            &i2s_ctx, &i2s_sample_buf, &frame_count, &timeout, &out_select, &ret);

    if( out_select == 0 ){
        ret = rtos_i2s_tx(i2s_ctx, i2s_sample_buf, frame_count, timeout);
    }
    else {
        ret = rtos_i2s_tx_1(i2s_ctx, i2s_sample_buf, frame_count, timeout);
    }

    msg_length = rpc_response_marshall(
            resp_msg, rpc_msg,
            i2s_ctx, i2s_sample_buf, frame_count, timeout, ret);

    return msg_length;
}

static void i2s_rpc_thread(rtos_intertile_address_t *client_address)
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
        case fcode_rx:
            msg_length = i2s_rx_rpc_host(&rpc_msg, &resp_msg);
            break;
        case fcode_tx:
            msg_length = i2s_tx_rpc_host(&rpc_msg, &resp_msg);
            break;
        }

        rtos_osal_free(req_msg);

        /* send RPC response message to client */
        rtos_intertile_tx(intertile_ctx, intertile_port, resp_msg, msg_length);
        rtos_osal_free(resp_msg);
    }
}

__attribute__((fptrgroup("rtos_driver_rpc_host_start_fptr_grp")))
static void i2s_rpc_start(
        rtos_driver_rpc_t *rpc_config)
{
    xassert(rpc_config->host_task_priority >= 0);

    for (int i = 0; i < rpc_config->remote_client_count; i++) {

        rtos_intertile_address_t *client_address = &rpc_config->client_address[i];

        xassert(client_address->port >= 0);

        rtos_osal_thread_create(
                NULL,
                "i2s_rpc_thread",
                (rtos_osal_entry_function_t) i2s_rpc_thread,
                client_address,
                RTOS_THREAD_STACK_SIZE(i2s_rpc_thread),
                rpc_config->host_task_priority);
    }
}

void rtos_i2s_rpc_config(
        rtos_i2s_t *i2s_ctx,
        unsigned intertile_port,
        unsigned host_task_priority)
{
    rtos_driver_rpc_t *rpc_config = i2s_ctx->rpc_config;

    if (rpc_config->remote_client_count == 0) {
        /* This is a client */
        rpc_config->host_address.port = intertile_port;

        rtos_osal_mutex_create(&i2s_ctx->mutex, "i2s_lock", RTOS_OSAL_NOT_RECURSIVE);

    } else {
        for (int i = 0; i < rpc_config->remote_client_count; i++) {
            rpc_config->client_address[i].port = intertile_port;
        }
        rpc_config->host_task_priority = host_task_priority;
    }
}

void rtos_i2s_rpc_client_init(
        rtos_i2s_t *i2s_ctx,
        rtos_driver_rpc_t *rpc_config,
        rtos_intertile_t *host_intertile_ctx)
{
    i2s_ctx->rpc_config = rpc_config;
    i2s_ctx->rx = i2s_remote_rx;
    i2s_ctx->tx = i2s_remote_tx;
    rpc_config->rpc_host_start = NULL;
    rpc_config->remote_client_count = 0;
    rpc_config->host_task_priority = -1;

    /* This must be configured later with rtos_i2s_rpc_config() */
    rpc_config->host_address.port = -1;

    rpc_config->host_address.intertile_ctx = host_intertile_ctx;
    rpc_config->host_ctx_ptr = (void *) s_chan_in_word(host_intertile_ctx->c);
    i2s_ctx->num_out = s_chan_in_word(host_intertile_ctx->c);
    i2s_ctx->num_in = s_chan_in_word(host_intertile_ctx->c);

    if( i2s_ctx->num_out == 2){
        i2s_ctx->tx_1 = i2s_remote_1_tx;
    }
}

void rtos_i2s_rpc_host_init(
        rtos_i2s_t *i2s_ctx,
        rtos_driver_rpc_t *rpc_config,
        rtos_intertile_t *client_intertile_ctx[],
        size_t remote_client_count)
{
    xassert(i2s_ctx->is_slave == false);

    i2s_ctx->rpc_config = rpc_config;
    rpc_config->rpc_host_start = i2s_rpc_start;
    rpc_config->remote_client_count = remote_client_count;

    /* This must be configured later with rtos_i2s_rpc_config() */
    rpc_config->host_task_priority = -1;

    for (int i = 0; i < remote_client_count; i++) {
        rpc_config->client_address[i].intertile_ctx = client_intertile_ctx[i];
        s_chan_out_word(client_intertile_ctx[i]->c, (uint32_t) i2s_ctx);
        s_chan_out_word(client_intertile_ctx[i]->c, (uint32_t) i2s_ctx->num_out);
        s_chan_out_word(client_intertile_ctx[i]->c, (uint32_t) i2s_ctx->num_in);

        /* This must be configured later with rtos_i2s_rpc_config() */
        rpc_config->client_address[i].port = -1;
    }
}
