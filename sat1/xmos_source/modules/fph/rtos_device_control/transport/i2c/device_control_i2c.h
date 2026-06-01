// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef I2C_DEVICE_CONTROL_H_
#define I2C_DEVICE_CONTROL_H_

#include "rtos_i2c_slave.h"
#include "device_control.h"

void device_control_i2c_start_cb(rtos_i2c_slave_t *ctx,
                                 device_control_t *device_control_ctx);

void device_control_i2c_rx_cb(rtos_i2c_slave_t *ctx,
                              device_control_t *device_control_ctx,
                              uint8_t *data,
                              size_t len);

size_t device_control_i2c_tx_start_cb(rtos_i2c_slave_t *ctx,
                                      device_control_t *device_control_ctx,
                                      uint8_t **data);


#endif /* I2C_DEVICE_CONTROL_H_ */
