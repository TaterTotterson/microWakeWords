#pragma once

#include "servicer.h"

#define GPIO_CONTROLLER_SERVICER_RESID   (210) //not used for now
#define GPIO_CONTROLLER_RESOURCE_IN_A    (211)
#define GPIO_CONTROLLER_RESOURCE_IN_B    (212)
#define GPIO_CONTROLLER_RESOURCE_OUT_A   (221)
#define GPIO_CONTROLLER_MAX_RESOURCES    (  3)

typedef enum gpio_resource_id
{
  RESOURCE_IN_A = 0,
  RESOURCE_IN_B = 1,
  RESOURCE_OUT_A = 2
} gpio_resource_id_t;


typedef struct {
  size_t  port_id;
  uint8_t bit_mask;
  uint8_t bit_shift;
  bool writeable;
  gpio_resource_id_t resource_idx;
  uint8_t status_register;
} device_control_gpio_ports_t;


typedef struct {
    servicer_t  *servicer;
    rtos_gpio_t *gpio_ctx;
    device_control_t **device_control_ctx;
    size_t device_control_ctx_count;
    uint8_t num_of_ports;
    device_control_gpio_ports_t *port_defs;
} device_control_gpio_ctx_t;


void gpio_servicer_init(device_control_gpio_ctx_t *ctx, rtos_gpio_t *gpio_ctx, device_control_gpio_ports_t* port_defs, uint8_t num_of_ports );


void gpio_servicer_start(device_control_gpio_ctx_t *ctx, device_control_t **device_control_ctx, size_t device_control_ctx_count );