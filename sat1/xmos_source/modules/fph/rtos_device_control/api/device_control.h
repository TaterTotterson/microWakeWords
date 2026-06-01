// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef DEVICE_CONTROL_H_
#define DEVICE_CONTROL_H_

#include <stdint.h>
#include <stddef.h>

#include "device_control_shared.h"

#include "rtos_osal.h"
#include "rtos_intertile.h"

/**
 * \defgroup device_control_xcore
 *
 * The public API for using the device control library
 * @{
 */

/**
 * Struct representing a device control instance.
 *
 * The members in this struct should not be accessed directly.
 */
typedef struct {
    uint8_t *resource_table; /* NULL on client tiles */
    int intertile_port;
    union {
        rtos_intertile_t *host_intertile;

        /*
         * Everything past this point is only used by the host tile.
         * The device_control_client_t struct includes only the above
         * members and may be used to save some memory when initialized
         * with DEVICE_CONTROL_CLIENT_MODE.
         */
        struct {
            rtos_intertile_t *client_intertile[3];

            size_t servicer_count;
            size_t intertile_count;
            rtos_osal_queue_t gateway_queue;

            struct {
                rtos_intertile_t *intertile_ctx;
                rtos_osal_queue_t *queue;
            } *servicer_table;

            size_t requested_payload_len;
            control_resid_t requested_resid;
            control_cmd_t requested_cmd;
            control_status_t last_status;
            uint8_t* status_buffer;
            uint8_t status_buffer_len;
        };
    };
} device_control_t;

/**
 * A device_control_t pointer may be cast to a pointer to this structure type and
 * used with the device control API, provided it is initialized with DEVICE_CONTROL_CLIENT_MODE.
 * This is not necessary to do, but will save a small amount of memory.
 */
typedef struct {
    uint8_t *resource_table; /* NULL on client tiles */
    int intertile_port;
    rtos_intertile_t *host_intertile;
} device_control_client_t;

/**
 * Struct representing a device control servicer instance.
 *
 * The members in this struct should not be accessed directly.
 */
typedef struct {
    rtos_osal_queue_t queue;
} device_control_servicer_t;

/**
 * Function pointer type for application provided device control read command handler callback functions.
 *
 * Called by device_control_servicer_cmd_recv() when a read command is received from the transport layer.
 * The command consists of a resource ID, command value, and a payload_len. This handler must respond with
 * a payload of the requested length.
 *
 * \param[in]  resid       Resource ID. Indicates which resource the command is intended for.
 * \param[in]  cmd         Command code. Note that this will be in the range 0x80 to 0xFF
 *                         because bit 7 set indicates a read command.
 * \param[out] payload     Payload bytes of length \p payload_len that will be sent back over
 *                         the transport layer in response to this read command.
 * \param[in]  payload_len Requested size of the payload in bytes.
 * \param[in,out] app_data A pointer to application specific data provided to device_control_servicer_cmd_recv().
 *                         How and if this is used is entirely up to the application.
 *
 * \returns                CONTROL_SUCCESS if the handling of the read data by the device was successful. An
 *                         error code otherwise.
 */
typedef control_ret_t (*device_control_read_cmd_cb_t)(control_resid_t resid, control_cmd_t cmd, uint8_t *payload, size_t payload_len, void *app_data);

/**
 * Function pointer type for application provided device control write command handler callback functions.
 *
 * Called by device_control_servicer_cmd_recv() when a write command is received from the transport layer.
 * The command consists of a resource ID, command value, payload, and the payload's length.
 *
 * \param[in]  resid       Resource ID. Indicates which resource the command is intended for.
 * \param[in]  cmd         Command code. Note that this will be in the range 0x80 to 0xFF
 *                         because bit 7 set indicates a read command.
 * \param[in]  payload     Payload bytes of length \p payload_len.
 * \param[in]  payload_len The number of bytes in \p payload.
 * \param[in,out] app_data A pointer to application specific data provided to device_control_servicer_cmd_recv().
 *                         How and if this is used is entirely up to the application.
 *
 * \returns                CONTROL_SUCCESS if the handling of the read data by the device was successful. An
 *                         error code otherwise.
 */
typedef control_ret_t (*device_control_write_cmd_cb_t)(control_resid_t resid, control_cmd_t cmd, const uint8_t *payload, size_t payload_len, void *app_data);


/**
 * Must be called by the transport layer when a new request is received.
 *
 * Precisely how each of the three command parameters resid, cmd, and payload_len
 * are received is specific to the transport layer and not defined by this library.
 *
 * \param ctx         A pointer to the associated device control instance.
 * \param resid       The received resource ID.
 * \param cmd         The received command value.
 * \param payload_len The length in bytes of the payload that will follow.
 *
 * \retval CONTROL_SUCCESS if \p resid has been registered by a servicer.
 * \retval CONTROL_BAD_COMMAND if \p resid has not been registered by a servicer.
 */
control_ret_t device_control_request(device_control_t *ctx,
                                     control_resid_t resid,
                                     control_cmd_t cmd,
                                     size_t payload_len);

/**
 * Must be called by the transport layer either when it receives a payload, or
 * when it requires a payload to transmit.
 *
 * \param ctx         A pointer to the associated device control instance.
 * \param payload_buf A pointer to the payload buffer.
 * \param buf_size    A pointer to a variable containing the size of \p payload_buf.
 *
 *                    When \p direction is CONTROL_HOST_TO_DEVICE, no more than this
 *                    number of bytes will be read from it.
 *
 *                    When \p direction is CONTROL_DEVICE_TO_HOST, this will be updated
 *                    to the number of bytes actually written to \p payload_buf.
 * \param direction   The direction of the payload transfer.
 *
 *                    This must be CONTROL_HOST_TO_DEVICE when a payload has already
 *                    been received and is inside \p payload_buf.
 *
 *                    This must be CONTROL_DEVICE_TO_HOST when a payload needs to be
 *                    written into \p payload_buf by device_control_payload_transfer()
 *                    before sending it.
 *
 * \returns           CONTROL_SUCCESS if everything works and the command is successfully
 *                    handled by a registered servicer. An error code otherwise.
 *
 */
control_ret_t device_control_payload_transfer(device_control_t *ctx,
                                              uint8_t *payload_buf,
                                              size_t *buf_size,
                                              control_direction_t direction);

/**
 * Must be called by the transport layer when it receives a payload and requires a
 * payload to transmit, for example, in a SPI transfer. The error status returned by the servicer
 * handling the command is updated in the first byte of the tx_buf.
 *
 * \param ctx         A pointer to the associated device control instance.
 * \param rx_buf      A pointer to the receive payload buffer.
 * \param rx_size     A variable containing the size of \p rx_buf.
 *
 *                    No more than this
 *                    number of bytes will be read from it.
 * \param tx_buf      A pointer to the transmitr payload buffer.
 * \param tx_size     A pointer variable containing the size of \p tx_buf.
 *
 *                    This will be updated
 *                    to the number of bytes actually written to \p tx_buf.
 *
 */
void device_control_payload_transfer_bidir(device_control_t *ctx,
                                              uint8_t *rx_buf,
                                              const size_t rx_size,
                                              uint8_t *tx_buf,
                                              size_t *tx_size);

/**
 * This is called by servicers to wait for and receive any commands received by the transport layer
 * contain one of the resource IDs registered by the servicer. This is also responsible for responding
 * to read commands.
 *
 * \param ctx          A pointer to the device control servicer context to receive commands for.
 * \param read_cmd_cb  The callback function to handle read commands for all resource IDs associated
 *                     with the given servicer.
 * \param write_cmd_cb The callback function to handle write commands for all resource IDs associated
 *                     with the given servicer.
 * \param app_data     A pointer to application specific data to pass along to the provided callback
 *                     functions. How and if this is used is entirely up to the application.
 * \param timeout      The number of RTOS ticks to wait before returning if no command is received.
 *
 * \retval             CONTROL_SUCCESS if a command successfully received and responded to.
 * \retval             CONTROL_ERROR if no command is received before the function times out,
 *                     or if there was a problem communicating back to the transport layer thread.
 */
control_ret_t device_control_servicer_cmd_recv(device_control_servicer_t *ctx,
                                               device_control_read_cmd_cb_t read_cmd_cb,
                                               device_control_write_cmd_cb_t write_cmd_cb,
                                               void *app_data,
                                               unsigned timeout);

/**
 * Set a status for a given resource.
 *
 * \param ctx        A pointer to the associated device control instance.
 * \param resid      The received resource ID.
 * \param status     The status to set.
 * 
 * \retval CONTROL_SUCCESS if \p resid has been registered by a servicer.
 * 
 */
control_ret_t device_control_set_resource_status( device_control_t *ctx,
                                                  uint8_t status_id,
                                                  uint8_t resource_status );


/**
 * This must be called on the tile that runs the transport layer for the device
 * control instance, and has initialized it with DEVICE_CONTROL_HOST_MODE. This
 * must be called after calling device_control_start() and before the transport
 * layer is started. It is to be run simultaneously with device_control_servicer_register()
 * from other threads on any tiles associated with the device control instance.
 * The number of servicers that must register is specified by the servicer_count parameter
 * of device_control_init().
 *
 * \param ctx     A pointer to the device control instance to register resources for.
 * \param timeout The amount of time in RTOS ticks to wait before all servicers register
 *                their resource IDs with device_control_servicer_register().
 *
 * \retval        CONTROL_SUCCESS if all servicers successfully register their resource
 *                IDs before the timeout.
 * \retval        CONTROL_REGISTRATION_FAILED otherwise.
 */
control_ret_t device_control_resources_register(device_control_t *ctx,
                                                unsigned timeout);

/**
 * Registers a servicer for a device control instance. Each servicer is responsible
 * for handling any number of resource IDs. All commands received from the transport
 * layer will be forwarded to the servicer that has registered the resource ID that
 * is found in the command.
 *
 * Servicers may be registered on any tile that has initialized a device control
 * instance. This must be called after calling device_control_start().
 *
 * \param ctx                      A pointer to the device control servicer context to initialize.
 * \param device_control_ctx       An array of pointers to the device control instance to register
 *                                 the servicer with.
 * \param device_control_ctx_count The number of device control instances to register the servicer
 *                                 with.
 * \param resources                Array of resource IDs to associate with this servicer.
 * \param num_resources            The number of resource IDs within \p resources.
 */
control_ret_t device_control_servicer_register(device_control_servicer_t *ctx,
                                               device_control_t *device_control_ctx[],
                                               size_t device_control_ctx_count,
                                               const control_resid_t resources[],
                                               size_t num_resources);

/**
 * Starts a device control instance. This must be called by all tiles that have called
 * device_control_init(). It may be called either before or after starting the RTOS, but
 * must be called before registering the resources and servicers for this instance.
 *
 * device_control_init() must be called on this device control instance prior to calling
 * this.
 *
 * \param ctx            A pointer to the device control instance to start.
 * \param intertile_port The port to use with any and all associated intertile
 *                       instances associated with this device control instance.
 *                       If this device control instance is only used by one tile
 *                       then this is unused.
 * \param priority       The priority of the task that will be created if the device
 *                       control instance was initialized with DEVICE_CONTROL_CLIENT_MODE.
 *                       This is unused on the tiles where this has been initialized
 *                       with DEVICE_CONTROL_HOST_MODE. This task is used to listen for
 *                       commands for a resource ID registered by a servicer running on
 *                       this tile, but received by the transport layer that is running on
 *                       another.
 */
control_ret_t device_control_start(device_control_t *ctx,
                                   uint8_t intertile_port,
                                   unsigned priority);

/**
 * Initializes a device control instance.
 *
 * This must be called by the tile that runs the transport layer (I2C, USB, etc) for the device
 * control instance, as well as all tiles that will register device control servicers for it.
 * It may be called either before or after starting the RTOS, but must be called before calling
 * device_control_start().
 *
 * \param ctx             A pointer to the device control context to initialize.
 * \param mode            Set to DEVICE_CONTROL_HOST_MODE if the command transport layer is on the
 *                        same tile. Set to DEVICE_CONTROL_CLIENT_MODE if the command transport layer
 *                        is on another tile.
 * \param servicer_count  The number of servicers that will be associated with this device
 *                        control instance.
 * \param intertile_ctx   An array of intertile contexts used to communicate with other tiles.
 * \param intertile_count The number of intertile contexts in the \p intertile_ctx array.
 *
 *                        When \p mode is DEVICE_CONTROL_HOST_MODE, this may be 0 if there are
 *                        no servicers on other tiles, up to one per device control instance that
 *                        has been initialized with DEVICE_CONTROL_CLIENT_MODE on other tiles.
 *
 *                        When \p mode is DEVICE_CONTROL_CLIENT_MODE then this must be 1,
 *                        and the intertile context must connect to a device control instance
 *                        on another tile that has been initialized with DEVICE_CONTROL_HOST_MODE.
 *
 * \returns               CONTROL_SUCCESS if the initialization was successful. An error status
 *                        otherwise.
 */
control_ret_t device_control_init(device_control_t *ctx,
                                  int mode,
                                  size_t servicer_count,
                                  rtos_intertile_t *intertile_ctx[],
                                  size_t intertile_count);

/**@}*/

#endif /* DEVICE_CONTROL_H_ */
