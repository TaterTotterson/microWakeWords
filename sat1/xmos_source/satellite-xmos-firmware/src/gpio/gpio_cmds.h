#pragma once

#include <stdint.h>

// GPIO_CONTROLLER_SERVICER_RESID commands
enum e_gpio_controller_servicer_resid_cmds
{
  GPIO_CONTROLLER_SERVICER_CMD_READ_PORT  = 0,
  GPIO_CONTROLLER_SERVICER_CMD_WRITE_PORT,
  GPIO_CONTROLLER_SERVICER_CMD_SET_PIN,

  NUM_GPIO_CONTROLLER_SERVICER_RESID_CMDS
};

// GPIO_CONTROLLER_SERVICER_RESID number of elements
// number of values of type gpio_controller_servicer_resid_write_port_t expected by GPIO_CONTROLLER_SERVICER_RESID_WRITE_PORT
#define GPIO_CONTROLLER_SERVICER_RESID_SET_PIN_NUM_VALUES (3)
// number of values of type gpio_controller_servicer_resid_read_port_t expected by GPIO_CONTROLLER_SERVICER_RESID_READ_PORT
#define GPIO_CONTROLLER_SERVICER_RESID_GET_PIN_NUM_VALUES (2)

// GPIO_CONTROLLER_SERVICER_RESID types
// type expected by GPIO_CONTROLLER_SERVICER_RESID_WRITE_PORT 
typedef uint8_t gpio_controller_servicer_resid_set_pin_t;
// type expected by GPIO_CONTROLLER_SERVICER_RESID_READ_PORT 
typedef uint8_t gpio_controller_servicer_resid_get_pin_t;
