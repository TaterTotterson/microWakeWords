// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#define DEBUG_UNIT SERVICER_TASK
#ifndef DEBUG_PRINT_ENABLE_SERVICER_TASK
    #define DEBUG_PRINT_ENABLE_SERVICER_TASK 0
#endif
#include "debug_print.h"
#include <stdio.h>
#include <platform.h>
#include "platform/platform_conf.h"

#include "servicer.h"
#include "dfu_servicer.h"


//-----------------Servicer read write callback functions-----------------------//
DEVICE_CONTROL_CALLBACK_ATTR
control_ret_t read_cmd(control_resid_t resid, control_cmd_t cmd, uint8_t *payload, size_t payload_len, void *app_data)
{
    control_ret_t ret = CONTROL_SUCCESS;
    servicer_t *servicer = (servicer_t*)app_data;

    // For read commands, payload[0] is reserved from status. So payload_len is one more than the payload_len stored in the resource command map
    payload_len -= 1;
    uint8_t *payload_ptr = &payload[1]; //Excluding the status byte, which is updated later.

    debug_printf("Servicer ID %d on tile %d received READ command %02x for resid %02x\n\t",servicer->id, THIS_XCORE_TILE, cmd, resid);
    debug_printf("The command is requesting %d bytes\n\t", payload_len);


    control_resource_info_t *current_res_info = get_res_info(resid, servicer);
    xassert(current_res_info != NULL); // This should never happen
    control_cmd_info_t *current_cmd_info;
    ret = validate_cmd(&current_cmd_info, current_res_info, cmd, payload_ptr, payload_len);
    if(ret != CONTROL_SUCCESS)
    {
        payload[0] = ret; // Update status in byte 0
        return ret;
    }
    // Check if command is for the servicer itself
    if(current_res_info->resource == servicer->res_info[0].resource)
    {
        ret = servicer_read_cmd(current_res_info, cmd, payload_ptr, payload_len);
        payload[0] = ret;
        return ret;
    }
    return CONTROL_ERROR;
}

DEVICE_CONTROL_CALLBACK_ATTR
control_ret_t write_cmd(control_resid_t resid, control_cmd_t cmd, const uint8_t *payload, size_t payload_len, void *app_data)
{
    control_ret_t ret = CONTROL_SUCCESS;
    servicer_t *servicer = (servicer_t*)app_data;
    //debug_printf("Device control WRITE. Servicer ID %d\n\t", servicer->id);

    debug_printf("Servicer ID %d on tile %d received WRITE command %02x for resid %02x\n\t", servicer->id, THIS_XCORE_TILE, cmd, resid);
    debug_printf("The command has %d bytes\n\t", payload_len);

    control_resource_info_t *current_res_info = get_res_info(resid, servicer);
    xassert(current_res_info != NULL);
    control_cmd_info_t *current_cmd_info;
    ret = validate_cmd(&current_cmd_info, current_res_info, cmd, payload, payload_len);
    if(ret != CONTROL_SUCCESS)
    {
        return ret;
    }
    // Check if command is for the servicer itself
    if(current_res_info->resource == servicer->res_info[0].resource)
    {
        ret = servicer_write_cmd(current_res_info, cmd, payload, payload_len);
        return ret;
    }
    return CONTROL_ERROR;
}

// Initialise packet payload pointers to point to valid memory.


//-----------------Servicer helper functions-----------------------//
// Return a pointer to the control_cmd_info_t structure for a given command ID. Return NULL if command not found in the
// command map for the resource.
control_cmd_info_t* get_cmd_info(uint8_t cmd_id, const control_resource_info_t *res_info)
{
    for(int i=0; i<res_info->command_map.num_commands; i++)
    {
        if(res_info->command_map.commands[i].cmd_id == cmd_id)
        {
            return &res_info->command_map.commands[i];
        }
    }
    return NULL;
}

// Return a pointer to the servicer's control_resource_info_t structure for a given resource ID.
// Return NULL if the resource ID is not found in the list of resources serviced by the servicer.
control_resource_info_t* get_res_info(control_resid_t resource, const servicer_t *servicer)
{
    for(int res=0; res<servicer->num_resources; res++)
    {
        // Get the cmd_info for the current command
        if(servicer->res_info[res].resource == resource)
        {
            return &servicer->res_info[res];
        }
    }
    return NULL;
}

// Validate the command from the host against that commands information in the stored command_map
control_ret_t validate_cmd(control_cmd_info_t **cmd_info,
                            control_resource_info_t *res_info,
                            control_cmd_t cmd,
                            const uint8_t *payload,
                            size_t payload_len)
{
    control_ret_t ret = CONTROL_SUCCESS;
    *cmd_info = get_cmd_info(CONTROL_CMD_CLEAR_READ(cmd), res_info);
    if(*cmd_info == NULL)
    {
        return SERVICER_WRONG_COMMAND_ID;
    }

    // Validate non special command length
    // Don't do payload check for special commands since for the last filter chunk, host might request less than the payload length specified in the cmd_map

    if(payload_len != (*cmd_info)->bytes_per_val * (*cmd_info)->num_vals)
    {
        return SERVICER_WRONG_COMMAND_LEN;
    }
    // Payload validation happens either on the host or in the specific command handlers. No generic payload validation is done in the servicer.

    return ret;
}

// Process write commands directed to the servicer resource
control_ret_t servicer_write_cmd(control_resource_info_t *res_info, control_cmd_t cmd, const uint8_t *payload, size_t payload_len)
{
    // Handle write commands directed to a servicer resource
    switch(res_info->resource)
    {
        case DFU_CONTROLLER_SERVICER_RESID:
            return dfu_servicer_write_cmd(res_info, cmd, payload, payload_len);
        break;
    }
    return CONTROL_SUCCESS;
}

// Process read commands directed to the servicer resource
control_ret_t servicer_read_cmd(control_resource_info_t *res_info, control_cmd_t cmd, uint8_t *payload, size_t payload_len)
{
    control_ret_t ret = CONTROL_SUCCESS;
     // Handle read commands directed to a servicer resource
    switch(res_info->resource)
    {
        case DFU_CONTROLLER_SERVICER_RESID:
            ret = dfu_servicer_read_cmd(res_info, cmd, payload, payload_len);
            break;
    }
    return ret;
}
