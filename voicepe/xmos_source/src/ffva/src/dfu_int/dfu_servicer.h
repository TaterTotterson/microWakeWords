// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#pragma once

#include "servicer.h"

#define DFU_CONTROLLER_SERVICER_RESID   (240)
#define NUM_RESOURCES_DFU_SERVICER      (1) // DFU servicer

/**
 * @brief DFU servicer task.
 *
 * This task handles DFU commands from the device control interface and relays
 * them to the internal DFU INT state machine.
 *
 * \param args      Pointer to the Servicer's state data structure
 */
void dfu_servicer(void *args);

// Servicer initialization functions
/**
 * @brief DFU servicer initialisation function.
 * \param servicer      Pointer to the Servicer's state data structure
 */
void dfu_servicer_init(servicer_t *servicer);

/**
 * @brief DFU servicer read command handler
 *
 * Handles read commands dedicated to the DFU servicer resource
 *
 * @param res_info          Resource info of the current command
 * @param cmd               Command ID of this command
 * @param payload           Pointer to the payload that contains the write data
 * @param payload_len       Length in bytes of the write command payload
 * @return control_ret_t    CONTROL_SUCCESS if command handled successfully,
 *                          otherwise control_ret_t error status indicating the error.
 */
control_ret_t dfu_servicer_read_cmd(control_resource_info_t *res_info, control_cmd_t cmd, uint8_t *payload, size_t payload_len);

/**
 * @brief DFU servicer write command handler
 *
 * Handles write commands dedicated to the DFU servicer resource
 *
 * @param res_info          Resource info of the current command
 * @param cmd               Command ID of this command
 * @param payload           Pointer to the payload that contains the write data
 * @param payload_len       Length in bytes of the write command payload
 * @return control_ret_t    CONTROL_SUCCESS if command handled successfully,
 *                          otherwise control_ret_t error status indicating the error.
 */
control_ret_t dfu_servicer_write_cmd(control_resource_info_t *res_info, control_cmd_t cmd, const uint8_t *payload, size_t payload_len);