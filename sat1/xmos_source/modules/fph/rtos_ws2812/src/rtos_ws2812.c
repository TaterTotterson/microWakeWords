
#include <xcore/port.h>
#include <xcore/channel.h>

#include "rtos_ws2812.h"

#define T1H  900    // Width of a 1 bit in ns
#define T1L  500    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

#define RES 250000    // Width of the low gap between bits to cause a frame to latch

#define NS_PER_SEC (1000000000L)

#define NS_PER_CYCLE ( NS_PER_SEC / XS1_TIMER_HZ )
#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )



static void ws2812_local_write(
        rtos_ws2812_t *ctx,
        const uint8_t buff[],
        size_t n)
{
    port_t p = ctx->ws2812_port;
    const uint32_t hl_value = ctx->hl_value;
    port_enable(p);
    port_out(p, 0);
    int t;
    t = port_get_trigger_time(p);

    //In case two threads try to access at the same time
    rtos_osal_mutex_get(&ctx->lock, RTOS_OSAL_WAIT_FOREVER);

    //To prevent interruption of Tx frame
    rtos_interrupt_mask_all();
    
    for(int i = 0; i < n; i++){
        uint8_t b = buff[i];
        for (int j = 0; j < 8; j++) {
            if ( b & (1 << 7) ){
                port_out_at_time(p, t, hl_value);
                t +=  NS_TO_CYCLES( T1H );
                port_out_at_time(p, t, 0);
                t +=  NS_TO_CYCLES( T1L );    
            } else {
                port_out_at_time(p, t, hl_value);
                t +=  NS_TO_CYCLES( T0H );
                port_out_at_time(p, t, 0);
                t +=  NS_TO_CYCLES( T0L );    
            }
            b <<= 1;
        }
    }
    t += NS_TO_CYCLES(RES);
    port_out_at_time(p, t, 0);
    rtos_interrupt_unmask_all();
    rtos_osal_mutex_put(&ctx->lock);
}


void rtos_ws2812_write( rtos_ws2812_t *ctx, const uint8_t buf[] ){
    ws2812_local_write( ctx, buf, ctx->num_leds * 3);
}

void rtos_ws2812_start( rtos_ws2812_t *ctx ){
    rtos_osal_mutex_create(&ctx->lock, "ws2812_lock", RTOS_OSAL_RECURSIVE);

    //if (ctx->rpc_config != NULL && ctx->rpc_config->rpc_host_start != NULL) {
    //    ctx->rpc_config->rpc_host_start(ctx->rpc_config);
    //}
}


void rtos_ws2812_init(
        rtos_ws2812_t *ctx,
        port_t ws2812port,
        const uint8_t pin,
        const uint8_t num_leds
)
{
    ctx->ws2812_port = ws2812port;
    ctx->hl_value = (1 << pin);
    ctx->num_leds = num_leds;
}
