// Copyright 2022-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#pragma once

#include <stdint.h>
#include "device_control_shared.h"

// Types of commands in the Command ID enum space -
// - Dedicated: Commands dedicated only for the resource
//      - For SHF this is further spilt into dedicated SHF commands and dedicated custom commands
// - Shared: Commands shared between a resource and its servicer. These are present in the servicer's command map only though respources process them as well. (Used for special commands protocol)
// - External: space reserved for customers to add commands.
#define DEDICATED_COMMANDS_START_OFFSET (0)
#define SHARED_COMMANDS_START_OFFSET (90)
#define EXTERNAL_COMMANDS_START_OFFSET (110)
// Servicers will have commands in the above 4 categories.
// Resources will have commands in the dedicated, reserved and broadcast categories.


typedef enum
{
    CMD_READ_ONLY,
    CMD_READ_WRITE,
    CMD_WRITE_ONLY
}cmd_rw_type_t;

typedef struct
{
    uint8_t cmd_id;
    uint8_t num_vals;
    uint8_t bytes_per_val;
    uint8_t cmd_rw_type;
}control_cmd_info_t;

typedef struct {
    int32_t num_commands;
    control_cmd_info_t *commands;
}command_map_t;
