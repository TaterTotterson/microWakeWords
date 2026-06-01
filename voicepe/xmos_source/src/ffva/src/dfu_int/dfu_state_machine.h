// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

// Compiler includes
#include <stdint.h>
#include <stdbool.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "dfu_common.h"

/**
 * \brief Defines the size of the internal data buffer. Should match the size
 *   of the payload transmitted over device control.
 */
#define DFU_DATA_XFER_SIZE 128
/**
 * \brief Defines the size of a flash sector; used to assemble multiple payloads
 *   into one write operation.
 */
#define DFU_SECTOR_SIZE 4096
/**
 * \brief Defines the number of payloads required to fill one flash sector. It is
 *   assumed that #DFU_DATA_XFER_SIZE is an integer factor of #DFU_SECTOR_SIZE.
 */
#define DFU_NUM_FRAGMENTS (DFU_SECTOR_SIZE / DFU_DATA_XFER_SIZE)

/**
 * \enum dfu_int_alt_setting_t
 * \brief Sets up identifiers for whether to target the factory or upgrade alts.
 *
 * Host application users will specify 0 for factory, 1 for upgrade.
 *
 * \var dfu_int_alt_setting_t::DFU_INT_ALTERNATE_FACTORY
 *   Sets to target the "factory" alt. This alt is read-only.
 * \var dfu_int_alt_setting_t::DFU_INT_ALTERNATE_UPGRADE
 *   Sets to target the "upgrade" alt. This alt can be both read and written.
 *
 */
typedef enum dfu_int_alt_setting_t
{
    DFU_INT_ALTERNATE_FACTORY,
    DFU_INT_ALTERNATE_UPGRADE
} dfu_int_alt_setting_t;

/**
 * \enum dfu_int_state_t
 * \brief Sets up identifiers for the different states in the DFU state machine.
 *
 * The names of these states and their values are derived from the USB Device
 *   Class Specification for Device Firmware Upgrade Version 1.1. This enum
 *   should not be changed, unless a specification update is issued.
 *
 */
typedef enum dfu_int_state_t
{
    DFU_INT_APP_IDLE,   // unused
    DFU_INT_APP_DETACH, // unused
    DFU_INT_DFU_IDLE,
    DFU_INT_DFU_DNLOAD_SYNC,
    DFU_INT_DFU_DNBUSY,
    DFU_INT_DFU_DNLOAD_IDLE,
    DFU_INT_DFU_MANIFEST_SYNC,
    DFU_INT_DFU_MANIFEST,
    DFU_INT_DFU_MANIFEST_WAIT_RESET,
    DFU_INT_DFU_UPLOAD_IDLE,
    DFU_INT_DFU_ERROR
} dfu_int_state_t;

/**
 * \enum dfu_int_status_t
 * \brief Sets up identifiers for statuses reported by the state machine.
 *
 * The names of these statuses and their values are derived from the USB Device
 *   Class Specification for Device Firmware Upgrade Version 1.1. This enum
 *   should not be changed, unless a specification update is issued. The meaning
 *   of each of these statuses may be found in the third table in section 6.1.2
 *   of the above specification.
 *
 */
typedef enum dfu_int_status_t
{
    DFU_INT_DFU_STATUS_OK,
    DFU_INT_DFU_STATUS_ERR_TARGET,
    DFU_INT_DFU_STATUS_ERR_FILE,
    DFU_INT_DFU_STATUS_ERR_WRITE,
    DFU_INT_DFU_STATUS_ERR_ERASE,
    DFU_INT_DFU_STATUS_ERR_CHECK_ERASED,
    DFU_INT_DFU_STATUS_ERR_PROG,
    DFU_INT_DFU_STATUS_ERR_VERIFY,
    DFU_INT_DFU_STATUS_ERR_ADDRESS,
    DFU_INT_DFU_STATUS_ERR_NOTDONE,
    DFU_INT_DFU_STATUS_ERR_FIRMWARE,
    DFU_INT_DFU_STATUS_ERR_VENDOR,
    DFU_INT_DFU_STATUS_ERR_USBR,
    DFU_INT_DFU_STATUS_ERR_POR,
    DFU_INT_DFU_STATUS_ERR_UNKNOWN,
    DFU_INT_DFU_STATUS_ERR_STALLEDPKT
} dfu_int_status_t;

/**
 * \struct dfu_int_get_status_packet_t
 * \brief Contains necessary fields to facilitate the GETSTATUS request.
 *
 * \var dfu_int_get_status_packet_t::next_state
 *   Indicates the next state that the state machine will move into after this
 *   request is processed.
 * \var dfu_int_get_status_packet_t::current_status
 *   Indicates the current status of the state machine, _before_ this request is
 *   processed.
 * \var dfu_int_get_status_packet_t::timeout_ms
 *   Indicates the number of milliseconds the host should wait before attempting
 *   another request.
 *
 */
typedef struct dfu_int_get_status_packet_t
{
    dfu_int_state_t next_state;
    dfu_int_status_t current_status;
    uint32_t timeout_ms;
} dfu_int_get_status_packet_t;

/**
 * \brief Sends a DFU_DETACH request to the DFU state machine.
 *
 * This is implemented as a device reboot after #DFU_REBOOT_DELAY_MS ms, set by
 *   default as 100 ms.
 */
void dfu_int_detach();

/**
 * \brief Sends a DFU_DNLOAD request to the DFU state machine.
 *
 * This function will copy \p length bytes of the \p download_data buffer
 *   into a buffer internal to dfu_state_machine.c, before notifying the state
 *   machine and returning. If \p length is given as 0, the state machine will
 *   regard this as the end of the download process and will move to the
 *   manifestation phase.
 *
 * \param[in] length         Number of valid bytes of data in \p download_data.
 * \param[in] download_data  Buffer containing \p length valid bytes of data.
 */
void dfu_int_download(uint16_t length, const uint8_t *download_data);

/**
 * \brief Sends a DFU_UPLOAD request to the DFU state machine.
 *
 * The state machine will fill \p upload_buffer with the next transfer block of
 *   data, and this function will then return the number of bytes written.
 *   Should the function return less than \p upload_buffer_length , then the
 *   device is signalling the end of the upload process.
 *
 * \param[out] upload_buffer         Buffer into which to place the upload data
 * \param[in]  upload_buffer_length  Length of \p upload_buffer in bytes
 * \return                           size_t Number of bytes written to buffer
 */
size_t dfu_int_upload(uint8_t *upload_buffer, size_t upload_buffer_length);

/**
 * \brief Sends a DFU_GETSTATUS request to the DFU state machine.
 *
 * This function will populate \p get_status_packet with required information.
 *   This will be handled by the calling task, rather than by the asynchronous
 *   state machine task. The calling task will predict the next state that the
 *   state machine will move into. The calling task will then usually signal the
 *   asynchronous task to action this request. This allows an instantaneous
 *   resonse to this request, rather than waiting for the state machine to
 *   process the request - which may be a non-trivial time if the state machine
 *   is currently writing to flash.
 *
 * \param[out] get_status_packet Pointer to the structure to be populated
 */
void dfu_int_get_status(dfu_int_get_status_packet_t *get_status_packet);

/**
 * \brief Sends a DFU_CLRSTATUS request to the DFU state machine.
 */
void dfu_int_clear_status();

/**
 * \brief Reads and returns the current state of the state machine.
 *
 * This request is serviced instantaneously, so long as the calling task has
 *   access to the current state of the state machine (which in the current
 *   implementation it does).
 *
 * \return dfu_int_state_t The current state of the state machine.
 */
dfu_int_state_t dfu_int_get_state();

/**
 * \brief Sends a DFU_ABORT request to the DFU state machine.
 */
void dfu_int_abort();

/**
 * \brief Sets the alternate interface used in UPLOAD and DNLOAD operations.
 *
 * This request will also always reset the state machine.
 *
 * \param[in] alt Alternate setting to change to.
 */
void dfu_int_set_alternate(dfu_int_alt_setting_t alt);

/**
 * \brief Sets the transfer block number for use in an UPLOAD operation.
 *
 * The DFU_UPLOAD request will return the data found at this transfer block.
 *   The transfer block size for this implementation is 64 bytes.
 *   This number will automatically increment on every successful DFU_UPLOAD.
 *   This number will default to 0. Therefore, to read the entire image, it is
 *   not necessary to first set this value.
 *
 * \param[in] transferblock Transfer block to use in an UPLOAD operation.
 */
void dfu_int_set_transfer_block(uint16_t transferblock);

/**
 * \brief Retrieves the current transfer block number.
 *
 * Strictly, this number represents the next transfer block that the UPLOAD
 *   operation will return. If called after issuing a DFU_UPLOAD request, it
 *   will represent the transfer block of the previous request plus one.
 *
 * \return uint16_t Current transfer block setting.
 */
uint16_t dfu_int_get_transfer_block();

/**
 * \brief RTOS task running the DFU state machine.
 *
 * Task should be created before attempting to use any of the above functions.
 *   Initialises the state machine and then proceeds into a loop, waiting for
 *   notifications on index 0 to determine which request has been issued. Will
 *   read a value in notification index 1 to determine how many requests have
 *   been issued since the last time the task awoke; must be exactly 1 or the
 *   state machine will move to the error state. Notification index 2 is
 *   currently unused.
 *
 * \param[in] args Unused
 */
void dfu_int_state_machine(void *args);
