// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"

// DFU_CONTROLLER_SERVICER_RESID command map
// This array may be unused as servicers can be moved between tiles
// Unused variable warnings are suppressed in this header file
static control_cmd_info_t dfu_controller_servicer_resid_cmd_map[] =
{
    { DFU_CONTROLLER_SERVICER_RESID_DFU_DETACH, 1, sizeof(uint8_t), CMD_WRITE_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_DNLOAD, 130, sizeof(uint8_t), CMD_WRITE_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_UPLOAD, 130, sizeof(uint8_t), CMD_READ_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATUS, 5, sizeof(uint8_t), CMD_READ_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_CLRSTATUS, 1, sizeof(uint8_t), CMD_WRITE_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATE, 1, sizeof(uint8_t), CMD_READ_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_ABORT, 1, sizeof(uint8_t), CMD_WRITE_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_SETALTERNATE, 1, sizeof(uint8_t), CMD_WRITE_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_TRANSFERBLOCK, 2, sizeof(uint8_t), CMD_READ_WRITE },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_GETVERSION, 3, sizeof(uint8_t), CMD_READ_ONLY },
    { DFU_CONTROLLER_SERVICER_RESID_DFU_REBOOT, 1, sizeof(uint8_t), CMD_WRITE_ONLY },
};
#pragma clang diagnostic pop
