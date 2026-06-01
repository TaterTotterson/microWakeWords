// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#pragma once
#include "device_control.h"
#include "cmd_map.h"

#define NUM_TILE_0_SERVICERS            (1) // only DFU servicer is used
#define NUM_TILE_1_SERVICERS            (0) // no control servicer


/**
 * Clears the read bit on a command code
 *
 * \param[in,out] c The command code to clear the read bit on.
 */
#define CONTROL_CMD_CLEAR_READ(c) ((c) & ~0x80)

// Structure encapsulating all the information about a resource
typedef struct
{
    control_resid_t resource; //device_control lib
    command_map_t command_map;
}control_resource_info_t;

typedef struct {
    uint32_t start_io; // set to 1 on one servicer per tile to make it responsible for starting the IO tasks on that tile.
    int32_t id; // Unique ID for the servicer. Used for debugging.
    // Num resources
    int32_t num_resources;
    // Resource ID and command map for every resource
    control_resource_info_t *res_info;
}servicer_t;

typedef struct {
    servicer_t *servicer;
    device_control_t **device_control_ctx;
    size_t device_control_ctx_count;    
    void *app_data;
} servicer_register_ctx_t;

// Servicer device_control callback functions
/**
 * @brief Device control callback function to handle a read command.
 *
 * @param resid         Resource ID of the command
 * @param cmd           Command ID of the command
 * @param payload       Pointer to the payload buffer that needs to be updated with the read command response.
 * @param payload_len   Length of the payload buffer
 * @param app_data      Application specific data.
 * @return              CONTROL_SUCCESS is command is handled successfully,
 *                      otherwise control_ret_t error status indicating the error.
 */
extern DEVICE_CONTROL_CALLBACK_ATTR
control_ret_t read_cmd(control_resid_t resid, control_cmd_t cmd, uint8_t *payload, size_t payload_len, void *app_data);

/**
 * @brief Device control callback function to handle a write command
 *
 * @param resid         Resource ID of the command
 * @param cmd           Command ID of the command
 * @param payload       Pointer to the payload buffer containing the payload the host wants to write to the device.
 * @param payload_len   Length of the payload buffer
 * @param app_data      Application specific data
 * @return              CONTROL_SUCCESS is command is handled successfully,
 *                      otherwise control_ret_t error status indicating the error.
 */
extern DEVICE_CONTROL_CALLBACK_ATTR
control_ret_t write_cmd(control_resid_t resid, control_cmd_t cmd, const uint8_t *payload, size_t payload_len, void *app_data);

// Servicer helper functions
/**
 * @brief Check if a command exists in the command map for a given resource and return the pointer to the cmd info object for the given command
 *
 * @param cmd_id        Command ID for which the control_cmd_info_t object is required.
 * @param res_info      Pointer to the resource info which contains all the command info objects for a given resource.
 * @return              Pointer to a control_cmd_info_t object when one is found for the given Command ID, NULL otherwise.
 */
control_cmd_info_t* get_cmd_info(uint8_t cmd_id, const control_resource_info_t *res_info);

/**
 * @brief Check if a resource ID is one of the resources supported by a servicer and return a pointer to the resource info object.
 *
 * @param resource      Resource ID that needs to be checked.
 * @param servicer      Pointer to the servicer state structure that holds all the resource info objects for the servicer.
 * @return              Pointer to the resource_info_t object when one is found for a given resource ID, NULL otherwise.
 */
control_resource_info_t* get_res_info(control_resid_t resource, const servicer_t *servicer);

/**
 * @brief Validate the command received in the servicer and return a pointer to the command info if the command is valid.
 *
 * All checks related to the command such as command ID being valid, correctness of the payload length,
 * validation of the actual payload for write commands are done in this function.
 *
 * @param cmd_info      Pointer in which the address of the command info is returned if this is a valid command/
 * @param res_info      Resource info of the resource the command is meant for.
 * @param cmd           Command ID of the command that needs validating
 * @param payload       Payload buffer of the command that needs validating.
 * @param payload_len   Payload length of the payload.
 * @return              CONTROL_SUCCESS if the command is found to be valid for the given resource. control_ret_t error otherwise
 *                      indicating the error code of the specific validation check that failed.
 */
control_ret_t validate_cmd(control_cmd_info_t **cmd_info,
                            control_resource_info_t *res_info,
                            control_cmd_t cmd,
                            const uint8_t *payload,
                            size_t payload_len);

/**
 * @brief Function for handling write commands directed to the servicer resource itself.
 *
 * @param resid         Command Resource ID
 * @param cmd           Command Command ID
 * @param payload       Command write payload buffer
 * @param payload_len   Length of the payload buffer
 * @return              CONTROL_SUCCESS if write command processed successfully. contro_ret_t error status otherwise.
 */
control_ret_t servicer_write_cmd(control_resource_info_t *res_info, control_cmd_t cmd, const uint8_t *payload, size_t payload_len);

/**
 * @brief               Function for handling read commands directed to the servicer resource itself.
 *
 * @param resid         Command Resource ID
 * @param cmd           Command Command ID
 * @param payload       Payload buffer to populate with the read response
 * @param payload_len   Length of the payload buffer
 * @return              CONTROL_SUCCESS if read command processed successfully. contro_ret_t error status otherwise.
 */
control_ret_t servicer_read_cmd(control_resource_info_t *res_info, control_cmd_t cmd, uint8_t *payload, size_t payload_len);
