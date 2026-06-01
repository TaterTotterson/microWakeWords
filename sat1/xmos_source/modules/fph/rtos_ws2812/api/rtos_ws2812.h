#ifndef RTOS_ws2812_H_
#define RTOS_ws2812_H_

#include "rtos_osal.h"
#include "rtos_driver_rpc.h"


typedef struct {
    port_t ws2812_port;
    uint32_t hl_value;
    rtos_osal_mutex_t lock;
    uint8_t num_leds;

    rtos_driver_rpc_t *rpc_config;
    
} rtos_ws2812_t;



/**
 * Writes data to an initialized and started ws2812 driver instance.
 * An xcore logical core is not reserved. The transmission
 * is a function call and the function blocks until the bit of the last
 * byte to be transmittted has completed. Interrupts are masked during this time
 * to avoid stretching of the waveform. Consequently, the function consumes cycles from
 * the caller thread.
 *
 * \param ctx             A pointer to the ws2812 driver instance to use.
 * \param buf             The buffer containing data to write.
 */
void rtos_ws2812_write( rtos_ws2812_t *ctx, const uint8_t buf[] );



/**
 * Starts an RTOS ws2812 driver instance. This must only be called by the tile that
 * owns the driver instance. It may be called either before or after starting
 * the RTOS, but must be called before any of the core ws2812 driver functions are
 * called with this instance.
 *
 * rtos_ws2812_init() must be called on this ws2812 driver instance prior to calling this.
 *
 * \param ctx       A pointer to the ws2812 driver instance to start.
 */
void rtos_ws2812_start(rtos_ws2812_t *ctx);



/**
 * Initialises an RTOS ws2812 driver instance.
 * This must only be called by the tile that owns the driver instance. It may be
 * called either before or after starting the RTOS, but must be called before calling
 * rtos_ws2812_start() or any of the core ws2812 driver functions with this instance.
 *
 * \param ctx        A pointer to the ws2812 driver instance to initialise.
 * \param port       The port containing the ws2812 pin
 * \param pin        The ws2812 pin number.
 * \param num_leds   The number of leds on the ws2812 bus.
 */
void rtos_ws2812_init(
        rtos_ws2812_t *ctx,
        const port_t port,
        const uint8_t pin,
        const uint8_t num_leds );

#endif
