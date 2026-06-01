// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef DEVICE_CONTROL_SHARED_H_
#define DEVICE_CONTROL_SHARED_H_

#include <stdint.h>
#include <stddef.h>

/**
 * \defgroup device_control_shared
 *
 * The shared API for using the device control library on the device and host
 * @{
 */

/**
 * This is the version of control protocol. Used to check compatibility
 */
#define CONTROL_VERSION 0x10

/**
 * @{
 * These types are used in control functions to identify the resource id,
 * command, version, and status.
 */
typedef uint8_t control_resid_t;
typedef uint8_t control_cmd_t;
typedef uint8_t control_version_t;
typedef uint8_t control_status_t;
/**@}*/

/**
 * This type enumerates the possible outcomes from a control transaction.
 */
typedef enum {
    CONTROL_SUCCESS = 0,
    CONTROL_REGISTRATION_FAILED,
    CONTROL_BAD_COMMAND,
    CONTROL_DATA_LENGTH_ERROR,
    CONTROL_OTHER_TRANSPORT_ERROR,
    CONTROL_BAD_RESOURCE,
    CONTROL_MALFORMED_PACKET,
    CONTROL_COMMAND_IGNORED_IN_DEVICE,
    CONTROL_ERROR,

    SERVICER_COMMAND_RETRY = 64,
    SERVICER_WRONG_COMMAND_ID,
    SERVICER_WRONG_COMMAND_LEN,
    SERVICER_WRONG_PAYLOAD,
    SERVICER_QUEUE_FULL,
    SERVICER_SPECIAL_COMMAND_ALREADY_ONGOING,
    SERVICER_SPECIAL_COMMAND_BUFFER_OVERFLOW,
    SERVICER_RESOURCE_ERROR,
    SERVICER_SPECIAL_COMMAND_WRONG_ORDER,
    SERVICER_SPECIAL_COMMAND_BUF_SIZE_ERROR,
    
} control_ret_t;

/**
 * This type is used to inform the control library the direction of
 * a control transfer from the transport layer.
 */
typedef enum {
    CONTROL_HOST_TO_DEVICE,
    CONTROL_DEVICE_TO_HOST
} control_direction_t;

/**
 * Checks if the read bit is set in a command code.
 *
 * \param[in] c The command code to check
 *
 * \returns true if the read bit in the command is set
 * \returns false if the read bit is not set
 */
#define IS_CONTROL_CMD_READ(c) ((c) & 0x80)

/**
 * Sets the read bit on a command code
 *
 * \param[in,out] c The command code to set the read bit on.
 */
#define CONTROL_CMD_SET_READ(c) ((c) | 0x80)

/**
 * Clears the read bit on a command code
 *
 * \param[in,out] c The command code to clear the read bit on.
 */
#define CONTROL_CMD_SET_WRITE(c) ((c) & ~0x80)

/**
 * This is the special resource ID owned by the control library.
 * It can be used to check the version of the control protocol.
 * Servicers may not register this resource ID.
 */
#define CONTROL_SPECIAL_RESID 0

/**
 * The maximum resource ID. IDs greater than this cannot
 * be registered.
 */
#define CONTROL_MAX_RESOURCE_ID 255

/**
 * The command to read the version of the control protocol.
 * It must be sent to resource ID CONTROL_SPECIAL_RESID.
 */
#define CONTROL_GET_VERSION CONTROL_CMD_SET_READ(0)

/**
 * The command to read the return status of the last command.
 * It must be sent to resource ID CONTROL_SPECIAL_RESID.
 */
#define CONTROL_GET_LAST_COMMAND_STATUS CONTROL_CMD_SET_READ(1)

/**
 * The mode value to use when initializing a device control instance
 * that is on the same tile as its associated transport layer. These
 * may be connected to device control instances on other tiles that
 * have been initialized with DEVICE_CONTROL_CLIENT_MODE.
 */
#define DEVICE_CONTROL_HOST_MODE   0

/**
 * The mode value to use when initializing a device control instance
 * that is not on the same tile as its associated transport layer.
 * These must be connected to a device control instance on another tile
 * that has been initialized with DEVICE_CONTROL_HOST_MODE.
 */
#define DEVICE_CONTROL_CLIENT_MODE 1

/**
 * This attribute must be specified on all device control command handler callback functions
 * provided by the application.
 */
#define DEVICE_CONTROL_CALLBACK_ATTR __attribute__((fptrgroup("device_control_cb_fptr_grp")))

/**@}*/

#endif /* DEVICE_CONTROL_SHARED_H_ */
