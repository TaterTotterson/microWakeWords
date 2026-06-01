#pragma once

#include "servicer.h"

#define LED_RING_SERVICER_RESID           (200)
#define NUM_RESOURCES_LED_RING            (  1)

typedef struct {
    servicer_t    *servicer;
    rtos_ws2812_t *ws2812_ctx;    
} led_ring_servicer_ctx_t;


/**
 * @brief LED-Ring servicer task.
 *
 * This task handles LED-Ring commands from the device control interface and relays
 * them to the internal ws2812 cntrl.
 *
 * \param args   Pointer to the Servicer's state data structure
 */
void led_ring_servicer(void *args);

// Servicer initialization functions
/**
 * @brief LED-Ring servicer initialisation function.
 * \param servicer      Pointer to the Servicer's state data structure
 */
void led_ring_servicer_init(servicer_t *servicer );

