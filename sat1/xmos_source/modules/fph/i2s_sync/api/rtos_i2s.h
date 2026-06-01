// Copyright 2020-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef RTOS_I2S_H_
#define RTOS_I2S_H_

/**
 * \addtogroup rtos_i2s_driver rtos_i2s_driver
 *
 * The public API for using the RTOS I2S driver.
 * @{
 */
#include <stdbool.h>
#include <xcore/clock.h>
#include <xcore/port.h>
#include "i2s.h"

#include "rtos_osal.h"
#include "rtos_driver_rpc.h"

/**
 * This attribute must be specified on all RTOS I2S send filter callback functions
 * provided by the application.
 */
#define RTOS_I2S_APP_SEND_FILTER_CALLBACK_ATTR __attribute__((fptrgroup("rtos_i2s_send_filter_cb_fptr_grp")))

/**
 * This attribute must be specified on all RTOS I2S receive filter callback functions
 * provided by the application.
 */
#define RTOS_I2S_APP_RECEIVE_FILTER_CALLBACK_ATTR __attribute__((fptrgroup("rtos_i2s_receive_filter_cb_fptr_grp")))

/**
 * Typedef to the RTOS I2S driver instance struct.
 */
typedef struct rtos_i2s_struct rtos_i2s_t;

/**
 * Function pointer type for application provided RTOS I2S send filter callback functions.
 *
 * These callback functions are called when an I2S driver instance needs output the next
 * audio frame to its interface. By default, audio frames in the driver's send buffer are
 * output directly to its interface. However, this gives the application an opportunity to
 * override this and provide filtering.
 *
 * These functions must not block.
 *
 * \param ctx               A pointer to the associated I2C slave driver instance.
 * \param app_data          A pointer to application specific data provided
 *                          by the application. Used to share data between
 *                          this callback function and the application.
 * \param i2s_frame         A pointer to the buffer where the callback should
 *                          write the next frame to send.
 * \param i2s_frame_size    The number of samples that should be written to
 *                          \p i2s_frame.
 * \param send_buf          A pointer to the next frame in the driver's send
 *                          buffer. The callback should use this as the input
 *                          to its filter.
 * \param samples_available The number of samples available in \p send_buf.
 *
 * \returns the number of samples read out of \p send_buf.
 */
typedef size_t (*rtos_i2s_send_filter_cb_t)(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *send_buf, size_t samples_available);

/**
 * Function pointer type for application provided RTOS I2S receive filter callback functions.
 *
 * These callback functions are called when an I2S driver instance has received the next audio
 * frame from its interface. By default, audio frames received from the driver's interface are
 * put directly into its receive buffer. However, this gives the application an opportunity to
 * override this and provide filtering.
 *
 * These functions must not block.
 *
 * \param ctx                A pointer to the associated I2C slave driver instance.
 * \param app_data           A pointer to application specific data provided
 *                           by the application. Used to share data between
 *                           this callback function and the application.
 * \param i2s_frame          A pointer to the buffer where the callback should
 *                           read the next received frame from The callback should
 *                           use this as the input to its filter.
 * \param i2s_frame_size     The number of samples that should be read from
 *                           \p i2s_frame.
 * \param receive_buf        A pointer to the next frame in the driver's send
 *                           buffer. The callback should use this as the input
 *                           to its filter.
 * \param sample_spaces_free The number of sample spaces free in \p receive_buf.
 *
 * \returns the number of samples written to \p receive_buf.
 */
typedef size_t (*rtos_i2s_receive_filter_cb_t)(rtos_i2s_t *ctx, void *app_data, int32_t *i2s_frame, size_t i2s_frame_size, int32_t *receive_buf, size_t sample_spaces_free);




struct SendBuffer{
    int32_t *buf;
    size_t buf_size;
    size_t write_index;
    size_t read_index;
    volatile size_t total_written;
    volatile size_t total_read;
    volatile size_t required_free_count;
};


/**
 * Struct representing an RTOS I2S driver instance.
 *
 * The members in this struct should not be accessed directly.
 */
struct rtos_i2s_struct{
    rtos_driver_rpc_t *rpc_config;

    __attribute__((fptrgroup("rtos_i2s_rx_fptr_grp")))
    size_t (*rx)(rtos_i2s_t *, int32_t *, size_t, unsigned);

    __attribute__((fptrgroup("rtos_i2s_tx_fptr_grp")))
    size_t (*tx)(rtos_i2s_t *, int32_t *, size_t, unsigned);

    __attribute__((fptrgroup("rtos_i2s_tx_fptr_grp")))
    size_t (*tx_1)(rtos_i2s_t *, int32_t *, size_t, unsigned);

    unsigned mclk_bclk_ratio;
    i2s_mode_t mode;
    port_t p_dout[I2S_MAX_DATALINES];
    size_t num_out;
    port_t p_din[I2S_MAX_DATALINES];
    size_t num_in;
    port_t p_bclk;
    port_t p_lrclk;
    port_t p_mclk;
    xclock_t bclk;

    void *send_filter_app_data;
    RTOS_I2S_APP_SEND_FILTER_CALLBACK_ATTR rtos_i2s_send_filter_cb_t send_filter_cb;

    void *receive_filter_app_data;
    RTOS_I2S_APP_RECEIVE_FILTER_CALLBACK_ATTR rtos_i2s_receive_filter_cb_t receive_filter_cb;

    rtos_osal_mutex_t mutex;
    streaming_channel_t c_i2s_isr;

    rtos_osal_thread_t hil_thread;
    rtos_osal_semaphore_t send_sem;
    rtos_osal_semaphore_t send_1_sem;
    rtos_osal_semaphore_t recv_sem;
    int send_blocked;
    int send_1_blocked;
    int recv_blocked;
    struct SendBuffer send_buffer;
    struct SendBuffer send_buffer_1;
    struct {
        int32_t *buf;
        size_t buf_size;
        size_t write_index;
        size_t read_index;
        volatile size_t total_written;
        volatile size_t total_read;
        volatile size_t required_available_count;
    } recv_buffer;
    uint8_t isr_cmd;
    bool is_slave;
};

#include "rtos_i2s_rpc.h"

/**
 * Helper function to calculate the MCLK/BCLK ratio given the
 * audio clock frequency at the master clock pin and the
 * desired sample rate.
 *
 * \param audio_clock_frequency The frequency of the audio clock at the port p_mclk.
 * \param sample_rate           The desired sample rate.
 *
 * \returns the MCLK/BCLK ratio that should be provided to rtos_i2s_start().
 */
inline int rtos_i2s_mclk_bclk_ratio(
        const unsigned audio_clock_frequency,
        const unsigned sample_rate)
{
    return audio_clock_frequency / (sample_rate * (8 * sizeof(int32_t)) * I2S_CHANS_PER_FRAME);
}

inline void rtos_i2s_send_filter_cb_set(
        rtos_i2s_t *ctx,
        rtos_i2s_send_filter_cb_t send_filter_cb,
        void *send_filter_app_data)
{
    ctx->send_filter_app_data = send_filter_app_data;
    ctx->send_filter_cb = send_filter_cb;
}

inline void rtos_i2s_receive_filter_cb_set(
        rtos_i2s_t *ctx,
        rtos_i2s_receive_filter_cb_t receive_filter_cb,
        void *receive_filter_app_data)
{
    ctx->receive_filter_app_data = receive_filter_app_data;
    ctx->receive_filter_cb = receive_filter_cb;
}

/**
 * Starts an RTOS I2S driver instance. This must only be called by the tile that
 * owns the driver instance. It must be called after starting the RTOS from an RTOS thread,
 * and must be called before any of the core I2S driver functions are called with this instance.
 *
 * One of rtos_i2s_master_init(), rtos_i2s_master_ext_clock_init, or rtos_i2s_slave_init()
 * must be called on this I2S driver instance prior to calling this.
 *
 * \param i2s_ctx           A pointer to the I2S driver instance to start.
 * \param mclk_bclk_ratio   The master clock to bit clock ratio. This may be computed
 *                          by the helper function rtos_i2s_mclk_bclk_ratio().
 *                          This is only used if the I2S instance was initialized with
 *                          rtos_i2s_master_init(). Otherwise it is ignored.
 * \param mode              The mode of the LR clock. See i2s_mode_t.
 * \param recv_buffer_size  The size in frames of the input buffer. Each frame is two samples
 *                          (left and right channels) per input port. For example, a size of two
 *                          here when num_in is three would create a buffer that holds up to
 *                          12 samples.
 * \param send_buffer_size  The size in frames of the output buffer. Each frame is two samples
 *                          (left and right channels) per output port. For example, a size of two
 *                          here when num_out is three would create a buffer that holds up to
 *                          12 samples.
 *                          Frames transmitted by rtos_i2s_tx() are stored in this
 *                          buffers before they are sent out to the I2S interface.
 * \param interrupt_core_id The ID of the core on which to enable the I2S interrupt.
 */
void rtos_i2s_start(
        rtos_i2s_t *i2s_ctx,
        unsigned mclk_bclk_ratio,
        i2s_mode_t mode,
        size_t recv_buffer_size,
        size_t send_buffer_size,
        unsigned interrupt_core_id);

/**
 * \addtogroup rtos_i2s_driver_core rtos_i2s_driver_core
 *
 * The core functions for using an RTOS I2S driver instance after
 * it has been initialized and started. These functions may be used
 * by both the host and any client tiles that RPC has been enabled for.
 * @{
 */

/**
 * Receives sample frames from the I2S interface.
 *
 * This function will block until new frames are available.
 *
 * \param ctx            A pointer to the I2S driver instance to use.
 * \param i2s_sample_buf A buffer to copy the received sample frames into.
 * \param frame_count    The number of frames to receive from the buffer.
 *                       This must be less than or equal to the size of the
 *                       input buffer specified to rtos_i2s_start().
 * \param timeout        The amount of time to wait before the requested number
 *                       of frames becomes available.
 *
 * \returns              The number of frames actually received into \p i2s_sample_buf.
 */
inline size_t rtos_i2s_rx(
        rtos_i2s_t *ctx,
        int32_t *i2s_sample_buf,
        size_t frame_count,
        unsigned timeout)
{
    return ctx->rx(ctx, i2s_sample_buf, frame_count, timeout);
}

/**
 * Transmits sample frames out to the I2S interface.
 *
 * The samples are stored into a buffer and are not necessarily sent out to the
 * I2S interface before this function returns.
 *
 * \param ctx            A pointer to the I2S driver instance to use.
 * \param i2s_sample_buf A buffer containing the sample frames to transmit out
 *                       to the I2S interface.
 * \param frame_count    The number of frames to transmit out from the buffer.
 *                       This must be less than or equal to the size of the
 *                       output buffer specified to rtos_i2s_start().
 * \param timeout        The amount of time to wait before there is enough
 *                       space in the send buffer to accept the frames to be
 *                       transmitted.
 *
 * \returns              The number of frames actually stored into the buffer.
 */
inline size_t rtos_i2s_tx(
        rtos_i2s_t *ctx,
        int32_t *i2s_sample_buf,
        size_t frame_count,
        unsigned timeout)
{
    return ctx->tx(ctx, i2s_sample_buf, frame_count, timeout);
}


/**
 * Transmits sample frames out to the I2S interface.
 *
 * The samples are stored into a buffer and are not necessarily sent out to the
 * I2S interface before this function returns.
 *
 * \param ctx            A pointer to the I2S driver instance to use.
 * \param i2s_sample_buf A buffer containing the sample frames to transmit out
 *                       to the I2S interface.
 * \param frame_count    The number of frames to transmit out from the buffer.
 *                       This must be less than or equal to the size of the
 *                       output buffer specified to rtos_i2s_start().
 * \param timeout        The amount of time to wait before there is enough
 *                       space in the send buffer to accept the frames to be
 *                       transmitted.
 *
 * \returns              The number of frames actually stored into the buffer.
 */
inline size_t rtos_i2s_tx_1(
        rtos_i2s_t *ctx,
        int32_t *i2s_sample_buf,
        size_t frame_count,
        unsigned timeout)
{
    xassert(ctx->num_out==2);
    return ctx->tx_1(ctx, i2s_sample_buf, frame_count, timeout);
}


/**@}*/

/**
 * \addtogroup rtos_i2s_master_driver rtos_i2s_master_driver
 *
 * The public API for using the RTOS I2S slave driver.
 * @{
 */

/**
 * Initializes an RTOS I2S driver instance in master mode.
 * This must only be called by the tile that owns the driver instance. It should be
 * called before starting the RTOS, and must be called before calling rtos_i2s_start()
 * or any of the core I2S driver functions with this instance.
 *
 * \param i2s_ctx        A pointer to the I2S driver instance to initialize.
 * \param io_core_mask   A bitmask representing the cores on which the low level I2S I/O thread
 *                       created by the driver is allowed to run. Bit 0 is core 0, bit 1 is core 1,
 *                       etc.
 * \param p_dout         An array of data output ports.
 * \param num_out        The number of output data ports.
 * \param p_din          An array of data input ports.
 * \param num_in         The number of input data ports.
 * \param p_bclk         The bit clock output port.
 * \param p_lrclk        The word clock output port.
 * \param p_mclk         Input port which supplies the master clock.
 * \param bclk           A clock that will get configured for use with
 *                       the bit clock.
 */
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
        xclock_t bclk);

/**
 * Initializes an RTOS I2S driver instance in master mode but that uses an externally
 * generated bit clock.
 * This must only be called by the tile that owns the driver instance. It should be
 * called before starting the RTOS, and must be called before calling rtos_i2s_start()
 * or any of the core I2S driver functions with this instance.
 *
 * \param i2s_ctx        A pointer to the I2S driver instance to initialize.
 * \param io_core_mask   A bitmask representing the cores on which the low level I2S I/O thread
 *                       created by the driver is allowed to run. Bit 0 is core 0, bit 1 is core 1,
 *                       etc.
 * \param p_dout         An array of data output ports.
 * \param num_out        The number of output data ports.
 * \param p_din          An array of data input ports.
 * \param num_in         The number of input data ports.
 * \param p_bclk         The bit clock output port.
 * \param p_lrclk        The word clock output port.
 * \param bclk           A clock that is configured externally to be used as
 *                       the bit clock
 */
void rtos_i2s_master_ext_clock_init(
        rtos_i2s_t *i2s_ctx,
        uint32_t io_core_mask,
        port_t p_dout[],
        size_t num_out,
        port_t p_din[],
        size_t num_in,
        port_t p_bclk,
        port_t p_lrclk,
        xclock_t bclk);

/**@}*/


/**
 * \addtogroup rtos_i2s_slave_driver rtos_i2s_slave_driver
 *
 * The public API for using the RTOS I2S slave driver.
 * @{
 */

/**
 * Initializes an RTOS I2S driver instance in slave mode.
 * This must only be called by the tile that owns the driver instance. It should be
 * called before starting the RTOS, and must be called before calling rtos_i2s_start()
 * or any of the core I2S driver functions with this instance.
 *
 * \param i2s_ctx        A pointer to the I2S driver instance to initialize.
 * \param io_core_mask   A bitmask representing the cores on which the low level I2S I/O thread
 *                       created by the driver is allowed to run. Bit 0 is core 0, bit 1 is core 1,
 *                       etc.
 * \param p_dout         An array of data output ports.
 * \param num_out        The number of output data ports.
 * \param p_din          An array of data input ports.
 * \param num_in         The number of input data ports.
 * \param p_bclk         The bit clock input port.
 * \param p_lrclk        The word clock input port.
 * \param bclk           A clock that will get configured for use with
 *                       the bit clock.
 */
void rtos_i2s_slave_init(
        rtos_i2s_t *i2s_ctx,
        uint32_t io_core_mask,
        port_t p_dout[],
        size_t num_out,
        port_t p_din[],
        size_t num_in,
        port_t p_bclk,
        port_t p_lrclk,
        xclock_t bclk);

/**@}*/

/**@}*/

#endif /* RTOS_I2S_H_ */
