// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#pragma once

#include <stdint.h>

// Note: The enums are wrapped around a #ifndef block to support autogenerating cmd_map files from the
// BeClearSuperHandsFree.h defines. The defines in BeClearSuperHandsFree.h are visible in the device
// but not in the host, so the #ifndef is needed to keep the cmd_map files common between device and host.
 
// DFU_CONTROLLER_SERVICER_RESID commands
enum e_dfu_controller_servicer_resid_cmds
{
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_DETACH
    DFU_CONTROLLER_SERVICER_RESID_DFU_DETACH = 0,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_DNLOAD
    DFU_CONTROLLER_SERVICER_RESID_DFU_DNLOAD = 1,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_UPLOAD
    DFU_CONTROLLER_SERVICER_RESID_DFU_UPLOAD = 2,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATUS
    DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATUS = 3,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_CLRSTATUS
    DFU_CONTROLLER_SERVICER_RESID_DFU_CLRSTATUS = 4,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATE
    DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATE = 5,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_ABORT
    DFU_CONTROLLER_SERVICER_RESID_DFU_ABORT = 6,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_SETALTERNATE
    DFU_CONTROLLER_SERVICER_RESID_DFU_SETALTERNATE = 64,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_TRANSFERBLOCK
    DFU_CONTROLLER_SERVICER_RESID_DFU_TRANSFERBLOCK = 65,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_GETVERSION
    DFU_CONTROLLER_SERVICER_RESID_DFU_GETVERSION = 88,
#endif
#ifndef DFU_CONTROLLER_SERVICER_RESID_DFU_REBOOT
    DFU_CONTROLLER_SERVICER_RESID_DFU_REBOOT = 89,
#endif
    NUM_DFU_CONTROLLER_SERVICER_RESID_CMDS = 11
};

// DFU_CONTROLLER_SERVICER_RESID number of elements
// number of values of type dfu_controller_servicer_resid_dfu_detach_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_DETACH
#define DFU_CONTROLLER_SERVICER_RESID_DFU_DETACH_NUM_VALUES (1)
// number of values of type dfu_controller_servicer_resid_dfu_dnload_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_DNLOAD
#define DFU_CONTROLLER_SERVICER_RESID_DFU_DNLOAD_NUM_VALUES (130)
// number of values of type dfu_controller_servicer_resid_dfu_upload_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_UPLOAD
#define DFU_CONTROLLER_SERVICER_RESID_DFU_UPLOAD_NUM_VALUES (130)
// number of values of type dfu_controller_servicer_resid_dfu_getstatus_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATUS
#define DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATUS_NUM_VALUES (5)
// number of values of type dfu_controller_servicer_resid_dfu_clrstatus_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_CLRSTATUS
#define DFU_CONTROLLER_SERVICER_RESID_DFU_CLRSTATUS_NUM_VALUES (1)
// number of values of type dfu_controller_servicer_resid_dfu_getstate_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATE
#define DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATE_NUM_VALUES (1)
// number of values of type dfu_controller_servicer_resid_dfu_abort_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_ABORT
#define DFU_CONTROLLER_SERVICER_RESID_DFU_ABORT_NUM_VALUES (1)
// number of values of type dfu_controller_servicer_resid_dfu_setalternate_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_SETALTERNATE
#define DFU_CONTROLLER_SERVICER_RESID_DFU_SETALTERNATE_NUM_VALUES (1)
// number of values of type dfu_controller_servicer_resid_dfu_transferblock_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_TRANSFERBLOCK
#define DFU_CONTROLLER_SERVICER_RESID_DFU_TRANSFERBLOCK_NUM_VALUES (2)
// number of values of type dfu_controller_servicer_resid_dfu_getversion_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_GETVERSION
#define DFU_CONTROLLER_SERVICER_RESID_DFU_GETVERSION_NUM_VALUES (3)
// number of values of type dfu_controller_servicer_resid_dfu_reboot_t expected by DFU_CONTROLLER_SERVICER_RESID_DFU_REBOOT
#define DFU_CONTROLLER_SERVICER_RESID_DFU_REBOOT_NUM_VALUES (1)

// DFU_CONTROLLER_SERVICER_RESID types
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_DETACH
typedef uint8_t dfu_controller_servicer_resid_dfu_detach_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_DNLOAD
typedef uint8_t dfu_controller_servicer_resid_dfu_dnload_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_UPLOAD
typedef uint8_t dfu_controller_servicer_resid_dfu_upload_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATUS
typedef uint8_t dfu_controller_servicer_resid_dfu_getstatus_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_CLRSTATUS
typedef uint8_t dfu_controller_servicer_resid_dfu_clrstatus_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_GETSTATE
typedef uint8_t dfu_controller_servicer_resid_dfu_getstate_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_ABORT
typedef uint8_t dfu_controller_servicer_resid_dfu_abort_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_SETALTERNATE
typedef uint8_t dfu_controller_servicer_resid_dfu_setalternate_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_TRANSFERBLOCK
typedef uint8_t dfu_controller_servicer_resid_dfu_transferblock_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_GETVERSION
typedef uint8_t dfu_controller_servicer_resid_dfu_getversion_t;
// type expected by DFU_CONTROLLER_SERVICER_RESID_DFU_REBOOT
typedef uint8_t dfu_controller_servicer_resid_dfu_reboot_t;
