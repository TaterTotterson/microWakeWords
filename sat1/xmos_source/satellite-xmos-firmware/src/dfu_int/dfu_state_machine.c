// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#define DEBUG_UNIT DFU_STATE_MACHINE
#ifndef DEBUG_PRINT_ENABLE_DFU_STATE_MACHINE
#define DEBUG_PRINT_ENABLE_DFU_STATE_MACHINE 0
#endif
#include "debug_print.h"

// Compiler includes
#include <stddef.h>
#include <string.h>
#include "xassert.h"
#include "xcore/hwtimer.h"

// Application includes
#include "dfu_state_machine.h"
#include "dfu_common.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "rtos_osal.h"
#include "task.h"

#define XCORE_MS_TO_TICKS(ms) (XS1_TIMER_KHZ * ms);

#define CLEAR_ALL_BITS 0xFFFFFFFF
#define REQUEST_COUNTER_INDEX 1

// Each notification bit matches the value given to the command by Table 3.2
// in USB-DFU v1.1 e.g. DNLOAD uses bit 1 since it has value 1.

// DFU_DETACH is value 0, handled by servicer.
#define DFU_INT_TASK_BIT_DNLOAD 0b00000010
#define DFU_INT_TASK_BIT_UPLOAD 0b00000100
#define DFU_INT_TASK_BIT_GETSTATUS 0b00001000
#define DFU_INT_TASK_BIT_CLRSTATUS 0b00010000
// DFU_GETSTATE is value 5, handled by servicer.
#define DFU_INT_TASK_BIT_ABORT 0b00100000
#define DFU_INT_TASK_BIT_SETALTERNATE 0b10000000 // not a DFU spec. command

typedef struct dfu_int_dfu_data_t
{
    TaskHandle_t task_handle;
    SemaphoreHandle_t upload_semaphore;
    dfu_int_alt_setting_t alt_setting;
    dfu_int_state_t current_state;
    dfu_int_status_t current_status;
    bool download_or_manifest_in_progress;
    bool move_to_error;
    dfu_int_status_t move_to_error_status;
    uint16_t transfer_block;
    uint16_t data_xfer_length;
    uint16_t download_block_number;
    uint8_t frag_number;
    uint8_t dfu_data_buffer[DFU_SECTOR_SIZE];
    uint32_t previous_timeout_ms;
    uint32_t timeout_start;
} dfu_int_dfu_data_t;

static dfu_int_dfu_data_t dfu_data;

/* DFU INT functions. These are called by the DFU servicer, and run on its RTOS
 * task and thread of control.*/

void dfu_int_detach()
{
    /*
     * This request is not valid in any state.
     * It is non-compliant to issue a DFU_DETACH request at any point
     * other than in the appIDLE state. Since we do not implement that
     * state, we should move to the dfuERROR state on receipt. However,
     * we will allow the user to force a reboot using this command. It
     * is preferred that the user use DFU_REBOOT for this purpose. Since
     * use of this command always forces a reboot, which resets the DFU
     * state machine, we do not bother pushing the state machine into
     * the dfuERROR state.
     *
     *                                   ┌───────┐
     *              DFU_DETACH           │   2   │
     *   Any state ─────────────REBOOT──►│dfuIDLE│
     *                                   └───────┘
     *
     */
    debug_printf("Detach\n");
    reboot();
}

void dfu_int_download(uint16_t length, const uint8_t *download_data)
{
    debug_printf("Download %d bytes\n", length);
    xassert(length <= DFU_DATA_XFER_SIZE);

    // frag_addr takes values [0, 128]
    uint16_t frag_addr = dfu_data.frag_number * DFU_DATA_XFER_SIZE;

    // Buffer starts empty and is cleared by state machine on write

    dfu_data.data_xfer_length = length;
    if (length > 0)
    {
        memcpy(&(dfu_data.dfu_data_buffer[frag_addr]), download_data, length);
    }
    // else ZLP, so end of download. Buffer is cleared by state machine on reset

    xTaskNotifyGiveIndexed(dfu_data.task_handle, REQUEST_COUNTER_INDEX);
    xTaskNotify(dfu_data.task_handle, DFU_INT_TASK_BIT_DNLOAD, eSetBits);
}

size_t dfu_int_upload(uint8_t *upload_buffer, size_t upload_buffer_length)
{
    debug_printf("Upload Start\n");
    // Immediately signal the state machine to populate the data buffer.
    xTaskNotifyGiveIndexed(dfu_data.task_handle, REQUEST_COUNTER_INDEX);
    xTaskNotify(dfu_data.task_handle, DFU_INT_TASK_BIT_UPLOAD, eSetBits);
    // Now we wait for the state machine to populate the buffer and tell us.
    xSemaphoreTake(dfu_data.upload_semaphore, RTOS_OSAL_WAIT_FOREVER);
    // Eat the data. This will be padded to the length of dfu_data_buffer.
    // Note - we are only using the first 64 bytes of the 256-wide data_buffer.
    memcpy(upload_buffer, dfu_data.dfu_data_buffer, upload_buffer_length);
    // And return the number of bytes that were actually read.
    debug_printf("Upload %d bytes\n", dfu_data.data_xfer_length);
    return dfu_data.data_xfer_length;
}

void dfu_int_get_status(dfu_int_get_status_packet_t *get_status_packet)
{
    /*
     * This request drives a reasonable amount of the state machine in
     * download and manifestation. See the below diagram for state
     * transitions.
     * This function is quite verbose/explicit and that is intentional.
     * Remember - this function runs on the servicer and tells the host what the
     * state machine is going to do next. It doesn't actually drive the state
     * machine, other than to handle the transitions to the error state by
     * setting a relevant flag. The state machine handles the actual transitions
     * and behaviours of those states.
     *
     *                                                              x
     *                                                      ┌───┐   │
     *                                         DFU_GETSTATUS│   │   ▼
     *              ┌───┐                                   │ ┌─┴──────┐
     * DFU_GETSTATUS│   │                                   └►│   10   │
     *              │ ┌─┴────────────┐                        │dfuERROR│
     *              └►│      9       │                        └────────┘
     *                │dfuUPLOAD-IDLE│                                ▲
     *                └──────────────┘                   DFU_GETSTATUS│
     *                                                                │
     *              ┌───┐                 DFU_GETSTATUS ┌──────┐      │
     * DFU_GETSTATUS│   │             (block incomplete)│      ▼      │
     *              │ ┌─┴─────┐            ┌────────────┴─┐  ┌────────┴┐
     *              └►│   2   │            │      3       │  │    4    │
     *                │dfuIDLE│            │dfuDNLOAD-SYNC│  │dfuDNBUSY│
     *                └───────┘            └──────┬───────┘  └─────────┘
     *                  ▲                         │
     *        ┌─────────┘                         │ DFU_GETSTATUS
     *        │ DFU_GETSTATUS                     │(block complete)
     *        │(manifestation complete)           ▼
     *       ┌┴───────────────┐            ┌──────────────┐
     *       │       6        │            │      5       │
     *       │dfuMANIFEST-SYNC├─┐          │dfuDNLOAD-IDLE├─┐
     *       └────────────────┘ │          └──────────────┘ │
     *                          │                        ▲  │
     *            ┌───────────┐ │                        └──┘
     *            │     7     │◄┘                       DFU_GETSTATUS
     *            │dfuMANIFEST│  DFU_GETSTATUS
     *            └─────────┬─┘ (manifestation incomplete)
     *                      │
     *           x──────────┘
     *           DFU_GETSTATUS
     *
     */
    bool send_notification = true;
    switch (dfu_data.current_state)
    {
    case DFU_INT_DFU_UPLOAD_IDLE:
    case DFU_INT_DFU_IDLE:
    case DFU_INT_DFU_ERROR:
    case DFU_INT_DFU_DNLOAD_IDLE:
    {
        get_status_packet->next_state = dfu_data.current_state;
        get_status_packet->current_status = dfu_data.current_status;
        get_status_packet->timeout_ms = 0;
        break;
    }
    case DFU_INT_DFU_DNBUSY:
    case DFU_INT_DFU_MANIFEST:
    {
        /*
         * If we are here, then we will have previously communicated a timeout
         * to the host. Has the host respected that (in which case the elapsed
         * time will be more than the previously sent timeout) or not?
         */
        uint32_t current_time = get_reference_time();
        uint32_t timeout_tks = XCORE_MS_TO_TICKS(dfu_data.previous_timeout_ms);
        uint32_t timeout_end = dfu_data.timeout_start + timeout_tks;
        if ((timeout_end - current_time) < timeout_tks)
        {
            get_status_packet->next_state = DFU_INT_DFU_ERROR;
            get_status_packet->current_status = dfu_data.current_status;
            get_status_packet->timeout_ms = 0;
            /*
             * If we're in these states, then the state machine task is busy
             * doing these things, and the host has bothered us too early - we
             * should tell the state machine to make the transition to the error
             * state. This is checked after it finishes being busy. This handles
             * this transition without sending a notification - by the time the
             * task reads the notification it will already be out of these
             * states so will not behave the way we want it to.
             */
            dfu_data.move_to_error = true;
            dfu_data.move_to_error_status = DFU_INT_DFU_STATUS_ERR_STALLEDPKT;
        }
        else
        {
            /*
             * The timeout has expired but we're still in these states, which
             * means that we're still busy (and underestimated the time we'd
             * need). Send the same timeout again, and tell the host we're going
             * to remain in these states. (Very strictly, what the standard says
             * we should be doing is moving back into the _SYNC states and then
             * moving into the DNBUSY/MANIFEST states, but our replies to this
             * will in theory look the same. The only way the host can tell the
             * difference is if they issue a GETSTATE before a GETSTATUS, which
             * they're not likely to do, and even then it'll just tell them
             * we're still busy.)
             */
            get_status_packet->next_state = dfu_data.current_state;
            get_status_packet->current_status = dfu_data.current_status;
            get_status_packet->timeout_ms = dfu_data.previous_timeout_ms;
        }
        send_notification = false;
        break;
    }
    case DFU_INT_DFU_DNLOAD_SYNC:
    {
        if (dfu_data.download_or_manifest_in_progress)
        {
            get_status_packet->next_state = DFU_INT_DFU_DNBUSY;
            get_status_packet->current_status = dfu_data.current_status;
            if (dfu_data.frag_number == (DFU_NUM_FRAGMENTS - 1))
            {
                if (dfu_data.download_block_number % 16 == 0) // SECTOR / PAGE
                {
                    // On this download, we will be erasing a sector and writing
                    get_status_packet->timeout_ms = DOWNLOAD_TIMEOUT_ERASE_MS;
                }
                else
                {
                    // On this download, we will be writing
                    get_status_packet->timeout_ms = DOWNLOAD_TIMEOUT_WRITE_MS;
                }
            }
            else
            {
                // On this download, we will just be storing in a buffer
                get_status_packet->timeout_ms = DOWNLOAD_TIMEOUT_BUFFER_MS;
            }
        }
        else
        {
            get_status_packet->next_state = DFU_INT_DFU_DNLOAD_IDLE;
            get_status_packet->current_status = dfu_data.current_status;
            get_status_packet->timeout_ms = 0;
        }
        break;
    }
    case DFU_INT_DFU_MANIFEST_SYNC:
    {
        if (dfu_data.download_or_manifest_in_progress)
        {
            get_status_packet->next_state = DFU_INT_DFU_MANIFEST;
            get_status_packet->current_status = dfu_data.current_status;
            get_status_packet->timeout_ms = 0;
        }
        else
        {
            get_status_packet->next_state = DFU_INT_DFU_IDLE;
            get_status_packet->current_status = dfu_data.current_status;
            get_status_packet->timeout_ms = 0;
        }
        break;
    }
    default:
    {
        // We have no idea where we are, but we shouldn't be there. Panic.
        get_status_packet->next_state = DFU_INT_DFU_ERROR;
        get_status_packet->current_status = dfu_data.current_status;
        get_status_packet->timeout_ms = 0;
        break;
    }
    }
    // Record the time just before we send the notification - latest we can set
    dfu_data.previous_timeout_ms = get_status_packet->timeout_ms;
    dfu_data.timeout_start = get_reference_time();
    if (send_notification)
    {
        xTaskNotifyGiveIndexed(dfu_data.task_handle, REQUEST_COUNTER_INDEX);
        xTaskNotify(dfu_data.task_handle, DFU_INT_TASK_BIT_GETSTATUS, eSetBits);
    }
    debug_printf("Get Status: Status %d, Timeout %d, Time %d, Next State %d\n",
                 get_status_packet->current_status,
                 get_status_packet->timeout_ms,
                 dfu_data.timeout_start,
                 get_status_packet->next_state);
}

void dfu_int_clear_status()
{
    debug_printf("Clear Status\n");
    xTaskNotifyGiveIndexed(dfu_data.task_handle, REQUEST_COUNTER_INDEX);
    xTaskNotify(dfu_data.task_handle, DFU_INT_TASK_BIT_CLRSTATUS, eSetBits);
}

dfu_int_state_t dfu_int_get_state()
{
    debug_printf("Get State: %d\n", dfu_data.current_state);
    return dfu_data.current_state;
}

void dfu_int_abort()
{
    debug_printf("Abort\n");
    xTaskNotifyGiveIndexed(dfu_data.task_handle, REQUEST_COUNTER_INDEX);
    xTaskNotify(dfu_data.task_handle, DFU_INT_TASK_BIT_ABORT, eSetBits);
}

void dfu_int_set_alternate(dfu_int_alt_setting_t alt)
{
    debug_printf("Set Alternate: %d\n", alt);
    dfu_data.alt_setting = alt;
    xTaskNotifyGiveIndexed(dfu_data.task_handle, REQUEST_COUNTER_INDEX);
    xTaskNotify(dfu_data.task_handle, DFU_INT_TASK_BIT_SETALTERNATE, eSetBits);
}

void dfu_int_set_transfer_block(uint16_t transferblock)
{
    debug_printf("Set Transfer Block: %d\n", transferblock);
    dfu_data.transfer_block = transferblock;
}

uint16_t dfu_int_get_transfer_block()
{
    debug_printf("Get Transfer Block: %d\n", dfu_data.transfer_block);
    return dfu_data.transfer_block;
}

/* Some readability functions */

static void dfu_int_reset_download_buffer()
{
    dfu_data.data_xfer_length = 0;
    dfu_data.frag_number = 0;
    dfu_data.download_block_number = 0;
    memset(dfu_data.dfu_data_buffer, 0, DFU_SECTOR_SIZE);
}

static void dfu_int_reset_state()
{
    dfu_data.current_state = DFU_INT_DFU_IDLE;
    dfu_data.current_status = DFU_INT_DFU_STATUS_OK;
    dfu_data.download_or_manifest_in_progress = false;
    dfu_data.move_to_error = false;
    dfu_data.move_to_error_status = DFU_INT_DFU_STATUS_OK;
    dfu_int_reset_download_buffer();
}

static void dfu_int_error(dfu_int_status_t status)
{
    dfu_int_reset_state();
    dfu_data.current_state = DFU_INT_DFU_ERROR;
    dfu_data.current_status = status;
}

/*
 * State machine function. This is a separate RTOS task.
 * Has three notification boxes: "what event has occured", "how many events have
 * occurred", and "extra data". "Extra data" is currently unused in favour of
 * the dfu_data structure being accessed directly by the servicer task, but if
 * this ever gets ported to a system where they don't have shared memory (i.e.
 * all RTOS notification calls are going via an intertile context) then this
 * could be used for the TRANSFERBLOCK and SETALTERNATE commands etc. - but the
 * implementor would still need to work out how to handle the 64 byte download
 * packet!
 */
void dfu_int_state_machine(void *args)
{
    /* Initialise semaphore and timer */
    dfu_data.upload_semaphore = xSemaphoreCreateBinary(); // Sem.s init empty
    /* Initialise the state machine */
    dfu_data.task_handle = xTaskGetCurrentTaskHandle();
    dfu_data.alt_setting = DFU_INT_ALTERNATE_FACTORY;
    dfu_int_reset_state();
    /* Sit in a loop and wait for mail */
    while (1)
    {
        uint32_t notification_value;
        xTaskNotifyWait(
            CLEAR_ALL_BITS,
            CLEAR_ALL_BITS,
            &notification_value,
            RTOS_OSAL_WAIT_FOREVER);

        /*
         * At this point, we've woken up from a notification.
         * notification_value should have exactly 1 bit set, which corresponds
         * to a specific request. Therefore, we can do a switch case.
         * If multiple bits are set, then we've had multiple requests without
         * the time to process them. Because we will have lost information about
         * the order of these requests, this is bad - go to an error state.
         *
         * However - we should never actually need to check this! Because:
         * We still can't know if we've had multiple of the same request issued
         * since the last time we awoke. Therefore, we should use one of our
         * notification indices as a counting semaphore - we decrement when we
         * start processing, the servicer increments when we've got mail. If
         * this value is ever 2, bad things happen - so we go to an error state.
         *
         * If this is actually a problem we can implement a message queue
         * instead, but I'm trying to keep the implementation lightweight for
         * now.
         */

        uint32_t counter_value = ulTaskNotifyTakeIndexed(
            REQUEST_COUNTER_INDEX,
            pdFALSE,
            RTOS_OSAL_NO_WAIT);

        if (counter_value != 1) // If it's >1, bad. If it's 0... also bad.
        {
            dfu_int_error(DFU_INT_DFU_STATUS_ERR_STALLEDPKT);
            continue; // Restart from top of loop
        }

        switch (notification_value)
        {
        case DFU_INT_TASK_BIT_DNLOAD:
        {
            /*
             * This request is valid in states dfuIDLE and dfuDNLOAD-IDLE.
             * If in any other state, pushes to dfuERROR with errSTALLED-PKT.
             * If in dfuIDLE and length is 0, pushes to dfuERROR.
             * If in dfuIDLE and length > 0, pushes to dfuDNLOAD-SYNC.
             * If in dfuDNLOAD-IDLE and length > 0, pushes to dfuDNLOAD-SYNC.
             * If in dfuDNLOAD-IDLE and length is 0, pushes to dfuMANIFEST-SYNC.
             *
             * TODO: There is provision in the specification that if in
             *  dfuDNLOAD-IDLE and length is 0, but the device does not think
             *  that it has enough data, then we can push to dfuERROR with
             *  status errNOTDONE. We do not currently implement this behaviour
             *  in INT or in USB.
             *
             *                   Any other state
             *                         or X
             *                            │    ┌────────┐
             *                 DFU_DNLOAD │    │   10   │
             *                            └───►│dfuERROR│
             *                                 └────────┘
             *              X                               X
             *              │                               │
             *   DFU_DNLOAD │                    DFU_DNLOAD │
             *   (len = 0)  │                               │
             *            ┌─┴─────┐ DFU_DNLOAD ┌────────────┴─┐
             *            │   2   │ (len > 0)  │      3       │
             *            │dfuIDLE├───────────►│dfuDNLOAD-SYNC│
             *            └───────┘            └──────────────┘
             *     X                                  ▲
             *     │                                  │ DFU_DNLOAD
             *     │ DFU_DNLOAD                       │ (len > 0)
             *     │                                  │
             *   ┌─┴──────────────┐ DFU_DNLOAD ┌──────┴───────┐
             *   │       6        │ (len = 0)  │      5       │
             *   │dfuMANIFEST-SYNC│◄───────────┤dfuDNLOAD-IDLE│
             *   └────────────────┘            └──────────────┘
             *
             */
            if (dfu_data.current_state == DFU_INT_DFU_IDLE &&
                dfu_data.data_xfer_length != 0)
            {
                // Starting a download. Set the "in progress" flag.
                dfu_data.download_or_manifest_in_progress = true;
                // Then move to the sync state
                dfu_data.current_state = DFU_INT_DFU_DNLOAD_SYNC;
                // We don't do anything else until we get a GETSTATUS
            }
            else if (dfu_data.current_state == DFU_INT_DFU_DNLOAD_IDLE)
            {
                if (dfu_data.data_xfer_length != 0)
                {
                    // Continuing a download. Set the "in progress" flag.
                    dfu_data.download_or_manifest_in_progress = true;
                    // Then move to the sync state
                    dfu_data.current_state = DFU_INT_DFU_DNLOAD_SYNC;
                    // We don't do anything else until we get a GETSTATUS
                }
                else // fill_level == 0, so end of download phase
                {
                    // Time to manifest. Set the "in progress" flag.
                    dfu_data.download_or_manifest_in_progress = true;
                    // Fully reset the download buffer, just to be neat.
                    dfu_int_reset_download_buffer();
                    // Then move to the sync state
                    dfu_data.current_state = DFU_INT_DFU_MANIFEST_SYNC;
                    // We don't do anything else until we get a GETSTATUS
                }
            }
            else // IDLE and len == 0, or not in IDLE or DNLOAD_IDLE
            {
                dfu_int_error(DFU_INT_DFU_STATUS_ERR_STALLEDPKT);
            }
            break;
        } // end of case DFU_INT_TASK_BIT_DNLOAD - returns to top of loop
        case DFU_INT_TASK_BIT_UPLOAD:
        {
            /*
             * This request is valid in states dfuIDLE and dfuUPLOAD-IDLE.
             * If in any other state, pushes to dfuERROR with errSTALLED-PKT.
             * First and foremost, send a packet.
             * If there is a full fragment to send, send
             *     it and then push to dfuUPLOAD-IDLE.
             * If there is less than a full fragment to
             *     send, send it and then push to dfuIDLE.
             *
             *                                             Any other state
             *          DFU_UPLOAD                                  │
             *         (len < fragment)                             │ DFU_UPLOAD
             *   ┌────────────────────────────┐                     ▼
             *   │                            │                ┌────────┐
             *   │      DFU_UPLOAD            │                │   10   │
             *   │     (len = fragment)       │                │dfuERROR│
             *   │          ┌────┐            │ ┌────┐         └────────┘
             *   │          │    │            ▼ │    │
             * ┌─┴──────────┴─┐  │        ┌─────┴─┐  │
             * │      9       │◄─┘        │   2   │◄─┘
             * │dfuUPLOAD-IDLE│           │dfuIDLE│   DFU_UPLOAD
             * └──────────────┘           └───┬───┘  (len < fragment)
             *        ▲                       │
             *        └───────────────────────┘
             *          DFU_UPLOAD
             *         (len = fragment)
             *
             */
            // First, clear the buffer
            memset(dfu_data.dfu_data_buffer, 0, DFU_SECTOR_SIZE);
            dfu_data.data_xfer_length = 0;

            if (dfu_data.current_state == DFU_INT_DFU_IDLE ||
                dfu_data.current_state == DFU_INT_DFU_UPLOAD_IDLE)
            {
                dfu_data.data_xfer_length = dfu_common_read_from_flash(
                    dfu_data.alt_setting,
                    dfu_data.transfer_block,
                    dfu_data.dfu_data_buffer,
                    DFU_DATA_XFER_SIZE);

                if (dfu_data.data_xfer_length < DFU_DATA_XFER_SIZE)
                {
                    debug_printf("Fill level %d, resetting!\n",
                                 dfu_data.data_xfer_length);
                    dfu_data.transfer_block = 0;
                    dfu_data.current_state = DFU_INT_DFU_IDLE;
                }
                else
                {
                    dfu_data.transfer_block += 1;
                    dfu_data.current_state = DFU_INT_DFU_UPLOAD_IDLE;
                }
            }
            else
            {
                dfu_int_error(DFU_INT_DFU_STATUS_ERR_STALLEDPKT);
            }
            /*
             * The servicer is yielding until we give this, so we must give
             * it back regardless of what happened. If an error occured, the
             * data_buffer_fill_level will be 0, so the host _should_ stop
             * trying to continue the upload process. That's the only way we can
             * signal the conclusion of the upload process.
             */
            xSemaphoreGive(dfu_data.upload_semaphore);
        }
        case DFU_INT_TASK_BIT_GETSTATUS:
        {
            /*
             * This request drives a reasonable amount of the state machine in
             * download and manifestation. See the below diagram for state
             * transitions.
             * This function is quite verbose/explicit and that is intentional.
             * This function runs on the state machine itself, and drives
             * behaviour and transitions thereof.
             *
             *                                                              x
             *                                                      ┌───┐   │
             *                                         DFU_GETSTATUS│   │   ▼
             *              ┌───┐                                   │ ┌─┴──────┐
             * DFU_GETSTATUS│   │                                   └►│   10   │
             *              │ ┌─┴────────────┐                        │dfuERROR│
             *              └►│      9       │                        └────────┘
             *                │dfuUPLOAD-IDLE│                                ▲
             *                └──────────────┘                   DFU_GETSTATUS│
             *                                                                │
             *              ┌───┐                 DFU_GETSTATUS ┌──────┐      │
             * DFU_GETSTATUS│   │             (block incomplete)│      ▼      │
             *              │ ┌─┴─────┐            ┌────────────┴─┐  ┌────────┴┐
             *              └►│   2   │            │      3       │  │    4    │
             *                │dfuIDLE│            │dfuDNLOAD-SYNC│  │dfuDNBUSY│
             *                └───────┘            └──────┬───────┘  └─────────┘
             *                  ▲                         │
             *        ┌─────────┘                         │ DFU_GETSTATUS
             *        │ DFU_GETSTATUS                     │(block complete)
             *        │(manifestation complete)           ▼
             *       ┌┴───────────────┐            ┌──────────────┐
             *       │       6        │            │      5       │
             *       │dfuMANIFEST-SYNC├─┐          │dfuDNLOAD-IDLE├─┐
             *       └────────────────┘ │          └──────────────┘ │
             *                          │                        ▲  │
             *            ┌───────────┐ │                        └──┘
             *            │     7     │◄┘                       DFU_GETSTATUS
             *            │dfuMANIFEST│  DFU_GETSTATUS
             *            └─────────┬─┘ (manifestation incomplete)
             *                      │
             *           x──────────┘
             *           DFU_GETSTATUS
             *
             */
            switch (dfu_data.current_state)
            {
            case DFU_INT_DFU_UPLOAD_IDLE:
            case DFU_INT_DFU_IDLE:
            case DFU_INT_DFU_ERROR:
            case DFU_INT_DFU_DNLOAD_IDLE:
            {
                // Do nothing - stay in the current state
                break;
            }
            case DFU_INT_DFU_DNLOAD_SYNC:
            {
                if (dfu_data.download_or_manifest_in_progress)
                {
                    /*
                     * We're in DNLOAD_SYNC and there is a download in progress.
                     * Therefore, we should write what's in the buffer to flash.
                     * While we're busy, if the host tries to talk to us again
                     * the servicer will handle it - if it should e.g. push us
                     * into an error state, the servicer will let us know. When
                     * we're done, we move back into DNLOAD_SYNC and clear the
                     * "download in progress" flag, ready to either receive
                     * another block or to move to manifestation.
                     */
                    dfu_data.current_state = DFU_INT_DFU_DNBUSY;
                    /*
                     * From here on, we assume that DFU_DATA_XFER_SIZE is an
                     * integer factor of DFU_SECTOR_SIZE. I'm not going to
                     * waste cycles asserting on this, so reader beware, please
                     * make sure this is always the case (unless you want to do
                     * some horrible maths!)
                     */
                    dfu_int_status_t retval = DFU_INT_DFU_STATUS_OK;

                    if (dfu_data.frag_number == (DFU_NUM_FRAGMENTS - 1))
                    {
                        // We've assembled a full download buffer. Write it.
                        retval = dfu_common_write_to_flash(
                            dfu_data.alt_setting,
                            dfu_data.download_block_number,
                            dfu_data.dfu_data_buffer,
                            DFU_SECTOR_SIZE);
                        memset(dfu_data.dfu_data_buffer, 0, DFU_SECTOR_SIZE);
                        dfu_data.frag_number = 0;
                        dfu_data.download_block_number += 1;
                    }
                    else
                    {
                        dfu_data.frag_number += 1;
                    }
                    dfu_data.download_or_manifest_in_progress = false;
                    if (dfu_data.move_to_error)
                    {
                        /*
                         * If this is set, then while we've been busy doing a
                         * download the servicer has decided that something bad
                         * has happened and asked us to move to an error state
                         * once we're done being busy. This is practically a
                         * deferment of the state transition from DNBUSY to
                         * ERROR.
                         */

                        dfu_int_error(dfu_data.move_to_error_status);
                    }
                    else if (retval != DFU_INT_DFU_STATUS_OK)
                    {
                        dfu_int_error(retval);
                    }
                    else
                    {
                        dfu_data.current_state = DFU_INT_DFU_DNLOAD_SYNC;
                    }
                }
                else
                {
                    /*
                     * We're in DNLOAD_SYNC but a download is not in progress.
                     * This means that we've finished downloading this block and
                     * should move to DNLOAD_IDLE, ready for either more blocks
                     * or a zero-length-packet to continue.
                     */
                    dfu_data.current_state = DFU_INT_DFU_DNLOAD_IDLE;
                }
                break;
            }
            case DFU_INT_DFU_MANIFEST_SYNC:
            {
                if (dfu_data.download_or_manifest_in_progress)
                {
                    /*
                     * We're in MANIFEST_SYNC and there's a manifest in
                     * progress. Therefore we've just moved from DNLOAD_IDLE,
                     * and should manifest what we've just downloaded. Do that.
                     * If the host tries to talk to us, the servicer will handle
                     * it, and will tell us if we need to move to an error state
                     * as a result of that.
                     */
                    dfu_data.current_state = DFU_INT_DFU_MANIFEST;
                    dfu_int_status_t retval = dfu_common_make_manifest();
                    dfu_data.download_or_manifest_in_progress = false;
                    if (dfu_data.move_to_error)
                    {
                        /*
                         * If this is set, then while we've been busy doing a
                         * manifest the servicer has decided that something bad
                         * has happened and asked us to move to an error state
                         * once we're done being busy. This is practically a
                         * deferment of the state transition from MANIFEST to
                         * ERROR.
                         */
                        dfu_int_error(dfu_data.move_to_error_status);
                    }
                    else if (retval != DFU_INT_DFU_STATUS_OK)
                    {
                        dfu_int_error(retval);
                    }
                    else
                    {
                        dfu_data.current_state = DFU_INT_DFU_MANIFEST_SYNC;
                    }
                }
                else
                {
                    /*
                     * We're in MANIFEST_SYNC but a manifest is not in progress.
                     * This means that we've finished manifesting and are done!
                     * Should move to DFU_IDLE.
                     */
                    dfu_data.current_state = DFU_INT_DFU_IDLE;
                }
                break;
            }
            case DFU_INT_DFU_DNBUSY:
            case DFU_INT_DFU_MANIFEST:
            default:
            {
                /*
                 * I don't think we can actually ever get here in these states,
                 * because we never yield while we're in them, but here for
                 * completeness. This transition is actually catered for by
                 * deferment using the dfu_data.move_to_error flag.
                 */

                dfu_int_error(DFU_INT_DFU_STATUS_ERR_STALLEDPKT);
                break;
            }
            } // end of switch current_state
            break;
        } // end of case DFU_INT_TASK_BIT_GETSTATUS - returns to top of loop
        case DFU_INT_TASK_BIT_CLRSTATUS:
        {
            /*
             * This request is valid in state dfuERROR only.
             * If in any other states, pushes to dfuERROR with errSTALLED-PKT.
             * If in dfuERROR, sets status to OK and pushes to dfuIDLE.
             *
             *                          Any other state
             *                                   │
             *                                   │ DFU_CLRSTATUS
             *                                   ▼
             *                                 ┌────────┐
             *                ┌────────────────┤   10   │
             *                │                │dfuERROR│
             *  DFU_CLRSTATUS │                └────────┘
             *                ▼
             *            ┌───────┐
             *            │   2   │
             *            │dfuIDLE│
             *            └───────┘
             *
             */
            if (dfu_data.current_state == DFU_INT_DFU_ERROR)
            {
                dfu_int_reset_state();
            }
            else
            {
                dfu_int_error(DFU_INT_DFU_STATUS_ERR_STALLEDPKT);
            }
            break;
        } // end of case DFU_INT_TASK_BIT_CLRSTATUS - returns to top of loop
        case DFU_INT_TASK_BIT_ABORT:
        {
            /*
             * This request is valid in dfuIDLE, dfuDNLOAD-IDLE, dfuUPLOAD-IDLE.
             * If in any other states, pushes to dfuERROR with errSTALLED-PKT.
             * If in dfuIDLE, do nothing.
             * If in dfuDNLOAD-IDLE, push to dfuIDLE.
             * If in dfuUPLOAD-IDLE, push to dfuIDLE.
             *
             *                                             Any other state
             *                                                      │
             *                                                      │ DFU_ABORT
             *                                                      ▼
             *                                                 ┌────────┐
             *                                                 │   10   │
             *                                                 │dfuERROR│
             *                                ┌──────┐         └────────┘
             *                                │      │
             * ┌──────────────┐           ┌───┴───┐  │
             * │      9       │ DFU_ABORT │   2   │  │ DFU_ABORT
             * │dfuUPLOAD-IDLE├──────────►│dfuIDLE│◄─┘
             * └──────────────┘           └───────┘
             *                                ▲
             *                                │
             *                                │
             *                                │
             *                                │                ┌──────────────┐
             *                                │    DFU_ABORT   │      5       │
             *                                └────────────────┤dfuDNLOAD-IDLE│
             *                                                 └──────────────┘
             *
             */
            if (dfu_data.current_state == DFU_INT_DFU_UPLOAD_IDLE ||
                dfu_data.current_state == DFU_INT_DFU_DNLOAD_IDLE)
            {
                dfu_int_reset_state();
            }
            else
            {
                dfu_int_error(DFU_INT_DFU_STATUS_ERR_STALLEDPKT);
            }
            break;
        } // end of case DFU_INT_TASK_BIT_ABORT
        case DFU_INT_TASK_BIT_SETALTERNATE:
        {
            /*
             * dfu_int_reset_state() purposefully doesn't change the alternate
             * setting, so we use it here, regardless of the fact the servicer
             * has updated this already
             *
             * To match the USB implementation, this request resets the state
             * machine. Therefore, set the alternate setting, and then we call
             * reset_state(). Moving to error resets everything else about the
             * state machine, so no need to do anything else here in that
             * instance.
             */
            dfu_int_reset_state();
            break;
        } // end of case DFU_INT_TASK_BIT_SETALTERNATE
        } // end of switch
    }     // end of while loop
} // end of dfu_state_machine()
