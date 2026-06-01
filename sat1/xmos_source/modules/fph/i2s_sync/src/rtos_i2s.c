// Copyright 2020-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#define DEBUG_UNIT RTOS_I2S

#include <string.h>

#include "rtos_printf.h"
#include <xcore/assert.h>
#include <xcore/triggerable.h>

#include "rtos_interrupt.h"
#include "rtos_i2s.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define ISR_RESUME_SEND_BM   0x01
#define ISR_RESUME_RECV_BM   0x02
#define ISR_RESUME_SEND_1_BM 0x04

DEFINE_RTOS_INTERRUPT_CALLBACK(rtos_i2s_isr, arg)
{
    rtos_i2s_t *ctx = arg;
    int isr_action;

    isr_action = s_chan_in_byte(ctx->c_i2s_isr.end_b);

    if (isr_action & ISR_RESUME_SEND_BM) {
        rtos_printf("send put\n");
        rtos_osal_semaphore_put(&ctx->send_sem);
    }

    if (isr_action & ISR_RESUME_RECV_BM) {
        rtos_printf("recv put\n");
        rtos_osal_semaphore_put(&ctx->recv_sem);
    }
    
    if (isr_action & ISR_RESUME_SEND_1_BM) {
        rtos_printf("send put\n");
        rtos_osal_semaphore_put(&ctx->send_1_sem);
    }
}

I2S_CALLBACK_ATTR
static void i2s_init(rtos_i2s_t *ctx, i2s_config_t *i2s_config)
{
    i2s_config->mode = ctx->mode;
    i2s_config->mclk_bclk_ratio = ctx->mclk_bclk_ratio;
}

I2S_CALLBACK_ATTR
static i2s_restart_t i2s_restart_check(rtos_i2s_t *ctx)
{
    return I2S_NO_RESTART;
}

I2S_CALLBACK_ATTR
static void i2s_receive(rtos_i2s_t *ctx, size_t num_in, const int32_t *i2s_sample_buf)
{
    size_t words_available = ctx->recv_buffer.total_written - ctx->recv_buffer.total_read;
    size_t words_free = ctx->recv_buffer.buf_size - words_available;
    size_t buffer_words_written = 0;

    if (ctx->receive_filter_cb == NULL) {
        if (num_in <= words_free) {
            memcpy(&ctx->recv_buffer.buf[ctx->recv_buffer.write_index], i2s_sample_buf, num_in * sizeof(int32_t));
            buffer_words_written = num_in;
        } else {
            // rtos_printf("i2s rx overrun\n");
        }
    } else {
        /*
         * The callback can't write past the end of the receive buffer,
         * even if more sample spaces are actually free
         */
        size_t sample_spaces_free = MIN(words_free, ctx->recv_buffer.buf_size - ctx->recv_buffer.write_index);
        buffer_words_written = ctx->receive_filter_cb(ctx, ctx->send_filter_app_data, (int32_t *)i2s_sample_buf, num_in, &ctx->recv_buffer.buf[ctx->recv_buffer.write_index], sample_spaces_free);
    }

    if (buffer_words_written > 0) {
        ctx->recv_buffer.write_index += num_in;
        if (ctx->recv_buffer.write_index >= ctx->recv_buffer.buf_size) {
            ctx->recv_buffer.write_index = 0;
        }
        RTOS_MEMORY_BARRIER();
        ctx->recv_buffer.total_written += num_in;
    }

    if (ctx->recv_buffer.required_available_count > 0) {
        words_available = ctx->recv_buffer.total_written - ctx->recv_buffer.total_read;

        if (words_available >= ctx->recv_buffer.required_available_count) {
            ctx->recv_buffer.required_available_count = 0;
            ctx->isr_cmd |= ISR_RESUME_RECV_BM;
        }
    }

    if (ctx->num_out == 0 && ctx->isr_cmd != 0) {
        s_chan_out_byte(ctx->c_i2s_isr.end_a, ctx->isr_cmd);
        ctx->isr_cmd = 0;
    }
}

I2S_CALLBACK_ATTR
static void i2s_send(rtos_i2s_t *ctx, size_t num_out, int32_t *i2s_sample_buf)
{
    num_out >>= 1;
    
    size_t words_available = ctx->send_buffer.total_written - ctx->send_buffer.total_read;
    size_t buffer_words_read = 0;

    if (ctx->send_filter_cb == NULL) {
        if (words_available >= 2) {
            memcpy(i2s_sample_buf, &ctx->send_buffer.buf[ctx->send_buffer.read_index], 2 * sizeof(int32_t));
            buffer_words_read = 2;
        } else {
            // rtos_printf("i2s tx underrun\n");
        }
    } else {
        /*
         * The callback can't read past the end of the send buffer,
         * even if more samples are actually available
         */
        size_t samples_available = MIN(words_available, ctx->send_buffer.buf_size - ctx->send_buffer.read_index);
        buffer_words_read = ctx->send_filter_cb(ctx, ctx->send_filter_app_data, i2s_sample_buf, 2, &ctx->send_buffer.buf[ctx->send_buffer.read_index], samples_available);
    }

    if (buffer_words_read > 0) {
        ctx->send_buffer.read_index += buffer_words_read;
        if (ctx->send_buffer.read_index >= ctx->send_buffer.buf_size) {
            ctx->send_buffer.read_index = 0;
        }
        RTOS_MEMORY_BARRIER();
        ctx->send_buffer.total_read += buffer_words_read;
    }

    if (ctx->send_buffer.required_free_count > 0) {
        words_available = ctx->send_buffer.total_written - ctx->send_buffer.total_read;
        size_t words_free = ctx->send_buffer.buf_size - words_available;

        if (words_free >= ctx->send_buffer.required_free_count) {
            ctx->send_buffer.required_free_count = 0;
            ctx->isr_cmd |= ISR_RESUME_SEND_BM;
        }
    }
    
    if( num_out > 1 ){
        size_t words_available_1 = ctx->send_buffer_1.total_written - ctx->send_buffer_1.total_read;
        size_t buffer_words_read_1 = 0;
    
        //no callback support for second send buffer yet
        if (words_available_1 >= 2) {
            memcpy(i2s_sample_buf + 2, &ctx->send_buffer_1.buf[ctx->send_buffer_1.read_index], 2 * sizeof(int32_t));
            buffer_words_read_1 = 2;
        } else {
            // rtos_printf("i2s tx underrun\n");
        }

        if (buffer_words_read_1 > 0) {
            ctx->send_buffer_1.read_index += buffer_words_read_1;
            if (ctx->send_buffer_1.read_index >= ctx->send_buffer_1.buf_size) {
                ctx->send_buffer_1.read_index = 0;
            }
            RTOS_MEMORY_BARRIER();
            ctx->send_buffer_1.total_read += buffer_words_read_1;
        }

        if (ctx->send_buffer_1.required_free_count > 0) {
            words_available_1 = ctx->send_buffer_1.total_written - ctx->send_buffer_1.total_read;
            size_t words_free = ctx->send_buffer_1.buf_size - words_available_1;

            if (words_free >= ctx->send_buffer_1.required_free_count) {
                ctx->send_buffer_1.required_free_count = 0;
                ctx->isr_cmd |= ISR_RESUME_SEND_1_BM;
            }
        }  
    }
        
    
    
    if (ctx->isr_cmd != 0) {
        s_chan_out_byte(ctx->c_i2s_isr.end_a, ctx->isr_cmd);
        ctx->isr_cmd = 0;
    }
}

static void i2s_master_thread(rtos_i2s_t *ctx)
{
    i2s_callback_group_t i2s_cbg = {
            (i2s_init_t) i2s_init,
            (i2s_restart_check_t) i2s_restart_check,
            (i2s_receive_t) i2s_receive,
            (i2s_send_t) i2s_send,
            ctx
    };

    (void) s_chan_in_byte(ctx->c_i2s_isr.end_a);

    rtos_printf("I2S on tile %d core %d\n", THIS_XCORE_TILE, rtos_core_id_get());
    i2s_master(
               &i2s_cbg,
               ctx->p_dout,
               ctx->num_out,
               ctx->p_din,
               ctx->num_in,
               ctx->p_bclk,
               ctx->p_lrclk,
               ctx->p_mclk,
               ctx->bclk);
}

static void i2s_master_ext_clock_thread(rtos_i2s_t *ctx)
{
    i2s_callback_group_t i2s_cbg = {
            (i2s_init_t) i2s_init,
            (i2s_restart_check_t) i2s_restart_check,
            (i2s_receive_t) i2s_receive,
            (i2s_send_t) i2s_send,
            ctx
    };

    (void) s_chan_in_byte(ctx->c_i2s_isr.end_a);

    rtos_printf("I2S on tile %d core %d\n", THIS_XCORE_TILE, rtos_core_id_get());
    i2s_master_external_clock(
            &i2s_cbg,
            ctx->p_dout,
            ctx->num_out,
            ctx->p_din,
            ctx->num_in,
            ctx->p_bclk,
            ctx->p_lrclk,
            ctx->bclk);
}

static void i2s_slave_thread(rtos_i2s_t *ctx)
{
    i2s_callback_group_t i2s_cbg = {
            (i2s_init_t) i2s_init,
            (i2s_restart_check_t) i2s_restart_check,
            (i2s_receive_t) i2s_receive,
            (i2s_send_t) i2s_send,
            ctx
    };

    (void) s_chan_in_byte(ctx->c_i2s_isr.end_a);

    rtos_printf("I2S on tile %d core %d\n", THIS_XCORE_TILE, rtos_core_id_get());
    i2s_slave(
            &i2s_cbg,
            ctx->p_dout,
            ctx->num_out,
            ctx->p_din,
            ctx->num_in,
            ctx->p_bclk,
            ctx->p_lrclk,
            ctx->bclk);
}

__attribute__((fptrgroup("rtos_i2s_rx_fptr_grp")))
static size_t i2s_local_rx(rtos_i2s_t *ctx,
                           int32_t *i2s_sample_buf,
                           size_t frame_count,
                           unsigned timeout)
{
    size_t frames_recvd = 0;
    size_t words_remaining = frame_count * (2 * ctx->num_in);
    int32_t *sample_buf_ptr = (int32_t *) i2s_sample_buf;

    xassert(words_remaining <= ctx->recv_buffer.buf_size);
    if (words_remaining > ctx->recv_buffer.buf_size) {
        return frames_recvd;
    }

    if (!ctx->recv_blocked) {
        size_t words_available = ctx->recv_buffer.total_written - ctx->recv_buffer.total_read;
        if (words_remaining > words_available) {
            ctx->recv_buffer.required_available_count = words_remaining;
            ctx->recv_blocked = 1;
        }
    }

    if (ctx->recv_blocked) {
        rtos_printf("recv get\n");
        if (rtos_osal_semaphore_get(&ctx->recv_sem, timeout) == RTOS_OSAL_SUCCESS) {
            ctx->recv_blocked = 0;
        }
    }

    if (!ctx->recv_blocked) {
        while (words_remaining) {
            size_t words_to_copy = MIN(words_remaining, ctx->recv_buffer.buf_size - ctx->recv_buffer.read_index);
            memcpy(sample_buf_ptr, &ctx->recv_buffer.buf[ctx->recv_buffer.read_index], words_to_copy * sizeof(int32_t));
            ctx->recv_buffer.read_index += words_to_copy;

            sample_buf_ptr += words_to_copy;
            words_remaining -= words_to_copy;

            if (ctx->recv_buffer.read_index >= ctx->recv_buffer.buf_size) {
                ctx->recv_buffer.read_index = 0;
            }
        }

        RTOS_MEMORY_BARRIER();
        ctx->recv_buffer.total_read += frame_count * (2 * ctx->num_in);

        frames_recvd = frame_count;
    }

    return frames_recvd;
}

__attribute__((fptrgroup("rtos_i2s_tx_fptr_grp")))
static size_t i2s_local_tx(rtos_i2s_t *ctx,
                           int32_t *i2s_sample_buf,
                           size_t frame_count,
                           unsigned timeout)
{
    size_t frames_sent = 0;
    size_t words_remaining = frame_count * 2;

    xassert(words_remaining <= ctx->send_buffer.buf_size);
    if (words_remaining > ctx->send_buffer.buf_size) {
        return frames_sent;
    }

    if (!ctx->send_blocked) {
        size_t words_free = ctx->send_buffer.buf_size - (ctx->send_buffer.total_written - ctx->send_buffer.total_read);
        if (words_remaining > words_free) {
            ctx->send_buffer.required_free_count = words_remaining;
            ctx->send_blocked = 1;
        }
    }

    if (ctx->send_blocked) {
        rtos_printf("send get\n");
        if (rtos_osal_semaphore_get(&ctx->send_sem, timeout) == RTOS_OSAL_SUCCESS) {
            ctx->send_blocked = 0;
        }
    }

    if (!ctx->send_blocked) {
        while (words_remaining) {
            size_t words_to_copy = MIN(words_remaining, ctx->send_buffer.buf_size - ctx->send_buffer.write_index);
            memcpy(&ctx->send_buffer.buf[ctx->send_buffer.write_index], i2s_sample_buf, words_to_copy * sizeof(int32_t));
            ctx->send_buffer.write_index += words_to_copy;

            i2s_sample_buf += words_to_copy;
            words_remaining -= words_to_copy;

            if (ctx->send_buffer.write_index >= ctx->send_buffer.buf_size) {
                ctx->send_buffer.write_index = 0;
            }
        }

        RTOS_MEMORY_BARRIER();
        ctx->send_buffer.total_written += frame_count * 2;

        frames_sent = frame_count;
    }

    return frames_sent;
}

__attribute__((fptrgroup("rtos_i2s_tx_fptr_grp")))
static size_t i2s_local_tx_1(rtos_i2s_t *ctx,
                           int32_t *i2s_sample_buf,
                           size_t frame_count,
                           unsigned timeout)
{
    size_t frames_sent = 0;
    size_t words_remaining = frame_count * 2;

    xassert(words_remaining <= ctx->send_buffer_1.buf_size);
    
    if (words_remaining > ctx->send_buffer_1.buf_size) {
        return frames_sent;
    }

    if (!ctx->send_1_blocked) {
        size_t words_free = ctx->send_buffer_1.buf_size - (ctx->send_buffer_1.total_written - ctx->send_buffer_1.total_read);
        if (words_remaining > words_free) {
            ctx->send_buffer_1.required_free_count = words_remaining;
            ctx->send_1_blocked = 1;
        }
    }

    if (ctx->send_1_blocked) {
        rtos_printf("send get\n");
        if (rtos_osal_semaphore_get(&ctx->send_1_sem, timeout) == RTOS_OSAL_SUCCESS) {
            ctx->send_1_blocked = 0;
        }
    }

    if (!ctx->send_1_blocked) {
        while (words_remaining) {
            size_t words_to_copy = MIN(words_remaining, ctx->send_buffer_1.buf_size - ctx->send_buffer_1.write_index);
            memcpy(&ctx->send_buffer_1.buf[ctx->send_buffer_1.write_index], i2s_sample_buf, words_to_copy * sizeof(int32_t));
            ctx->send_buffer_1.write_index += words_to_copy;

            i2s_sample_buf += words_to_copy;
            words_remaining -= words_to_copy;

            if (ctx->send_buffer_1.write_index >= ctx->send_buffer.buf_size) {
                ctx->send_buffer_1.write_index = 0;
            }
        }

        RTOS_MEMORY_BARRIER();
        ctx->send_buffer_1.total_written += frame_count * 2;

        frames_sent = frame_count;
    }

    return frames_sent;
}


void rtos_i2s_start(
        rtos_i2s_t *i2s_ctx,
        unsigned mclk_bclk_ratio,
        i2s_mode_t mode,
        size_t recv_buffer_size,
        size_t send_buffer_size,
        unsigned interrupt_core_id)
{
    uint32_t core_exclude_map;

    i2s_ctx->mclk_bclk_ratio = mclk_bclk_ratio;
    i2s_ctx->mode = mode;
    i2s_ctx->isr_cmd = 0;

    memset(&i2s_ctx->recv_buffer, 0, sizeof(i2s_ctx->send_buffer));
    if (i2s_ctx->num_in > 0) {
        i2s_ctx->recv_buffer.buf_size = recv_buffer_size * (2 * i2s_ctx->num_in);
        i2s_ctx->recv_buffer.buf = rtos_osal_malloc(i2s_ctx->recv_buffer.buf_size * sizeof(int32_t));
        rtos_osal_semaphore_create(&i2s_ctx->recv_sem, "i2s_recv_sem", 1, 0);
    }

    memset(&i2s_ctx->send_buffer, 0, sizeof(i2s_ctx->send_buffer));
    if (i2s_ctx->num_out > 0) {
        i2s_ctx->send_buffer.buf_size = send_buffer_size * 2;
        i2s_ctx->send_buffer.buf = rtos_osal_malloc(i2s_ctx->send_buffer.buf_size * sizeof(int32_t));
        rtos_osal_semaphore_create(&i2s_ctx->send_sem, "i2s_send_sem", 1, 0);
    }
    
    memset(&i2s_ctx->send_buffer_1, 0, sizeof(i2s_ctx->send_buffer_1));
    if (i2s_ctx->num_out > 1) {
        i2s_ctx->send_buffer_1.buf_size = send_buffer_size * 2;
        i2s_ctx->send_buffer_1.buf = rtos_osal_malloc(i2s_ctx->send_buffer_1.buf_size * sizeof(int32_t));
        rtos_osal_semaphore_create(&i2s_ctx->send_1_sem, "i2s_send_1_sem", 1, 0);
    }
    
    /* Ensure that the I2S interrupt is enabled on the requested core */
    rtos_osal_thread_core_exclusion_get(NULL, &core_exclude_map);
    rtos_osal_thread_core_exclusion_set(NULL, ~(1 << interrupt_core_id));

    triggerable_enable_trigger(i2s_ctx->c_i2s_isr.end_b);

    /* Tells the task running the I2S I/O to start */
    s_chan_out_byte(i2s_ctx->c_i2s_isr.end_b, 0);

    /* Restore the core exclusion map for the calling thread */
    rtos_osal_thread_core_exclusion_set(NULL, core_exclude_map);

    if (i2s_ctx->rpc_config != NULL && i2s_ctx->rpc_config->rpc_host_start != NULL) {
        i2s_ctx->rpc_config->rpc_host_start(i2s_ctx->rpc_config);
    }
}

static void rtos_i2s_init(
        rtos_i2s_t *ctx,
        uint32_t io_core_mask,
        port_t p_dout[],
        size_t num_out,
        port_t p_din[],
        size_t num_in,
        port_t p_bclk,
        port_t p_lrclk,
        port_t p_mclk,
        xclock_t bclk,
        rtos_osal_entry_function_t driver_thread_entry,
        size_t driver_thread_entry_size)
{
    xassert(num_out <= I2S_MAX_DATALINES);
    xassert(num_in <= I2S_MAX_DATALINES);

    memcpy(ctx->p_dout, p_dout, num_out * sizeof(port_t));
    memcpy(ctx->p_din, p_din, num_in * sizeof(port_t));

    ctx->num_out = num_out;
    ctx->num_in = num_in;

    ctx->p_bclk = p_bclk;
    ctx->p_lrclk = p_lrclk;
    ctx->p_mclk = p_mclk;
    ctx->bclk = bclk;

    ctx->c_i2s_isr = s_chan_alloc();

    ctx->rpc_config = NULL;
    ctx->rx = i2s_local_rx;
    ctx->tx = i2s_local_tx;
    ctx->is_slave = false;

    xassert( ctx->num_out <= 2 );
    if( ctx->num_out == 2){
        ctx->tx_1 = i2s_local_tx_1;
    }

    triggerable_setup_interrupt_callback(ctx->c_i2s_isr.end_b, ctx, RTOS_INTERRUPT_CALLBACK(rtos_i2s_isr));

    rtos_osal_thread_create(
            &ctx->hil_thread,
            "i2s_thread",
            driver_thread_entry,
            ctx,
            driver_thread_entry_size,
            RTOS_OSAL_HIGHEST_PRIORITY);

    /* Ensure the I2S thread is never preempted */
    rtos_osal_thread_preemption_disable(&ctx->hil_thread);
    /* And ensure it only runs on one of the specified cores */
    rtos_osal_thread_core_exclusion_set(&ctx->hil_thread, ~io_core_mask);
}

void rtos_i2s_master_init(
        rtos_i2s_t *i2s_ctx,
        uint32_t io_core_mask,
        port_t p_dout[],
        size_t num_out,
        port_t p_din[],
        size_t num_in,
        port_t p_bclk,
        port_t p_lrclk,
        port_t p_mclk,
        xclock_t bclk)
{
    port_enable(p_mclk);

    rtos_i2s_init(i2s_ctx,
                  io_core_mask,
                  p_dout,
                  num_out,
                  p_din,
                  num_in,
                  p_bclk,
                  p_lrclk,
                  p_mclk,
                  bclk,
                  (rtos_osal_entry_function_t) i2s_master_thread,
                  RTOS_THREAD_STACK_SIZE(i2s_master_thread));
}

void rtos_i2s_master_ext_clock_init(
        rtos_i2s_t *i2s_ctx,
        uint32_t io_core_mask,
        port_t p_dout[],
        size_t num_out,
        port_t p_din[],
        size_t num_in,
        port_t p_bclk,
        port_t p_lrclk,
        xclock_t bclk)
{
    rtos_i2s_init(i2s_ctx,
                  io_core_mask,
                  p_dout,
                  num_out,
                  p_din,
                  num_in,
                  p_bclk,
                  p_lrclk,
                  0,
                  bclk,
                  (rtos_osal_entry_function_t) i2s_master_ext_clock_thread,
                  RTOS_THREAD_STACK_SIZE(i2s_master_ext_clock_thread));
}

void rtos_i2s_slave_init(
        rtos_i2s_t *i2s_ctx,
        uint32_t io_core_mask,
        port_t p_dout[],
        size_t num_out,
        port_t p_din[],
        size_t num_in,
        port_t p_bclk,
        port_t p_lrclk,
        xclock_t bclk)
{
    rtos_i2s_init(i2s_ctx,
                  io_core_mask,
                  p_dout,
                  num_out,
                  p_din,
                  num_in,
                  p_bclk,
                  p_lrclk,
                  0,
                  bclk,
                  (rtos_osal_entry_function_t) i2s_slave_thread,
                  RTOS_THREAD_STACK_SIZE(i2s_slave_thread));

    i2s_ctx->is_slave = true;
}
