// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <platform.h>
#include <stdlib.h>
#include <stdbool.h>

#include "rtos_printf.h"
#include "device_control_usb.h"

static device_control_t *device_control_ctx;

#if CFG_TUSB_DEBUG >= 2
  #define DRIVER_NAME(_name)    .name = _name,
#else
  #define DRIVER_NAME(_name)
#endif

static void device_control_usb_init(void)
{
    if (device_control_usb_get_ctrl_ctx_cb) {
        device_control_ctx = device_control_usb_get_ctrl_ctx_cb();
        rtos_printf("USB Device Control Driver Initialized!\n");
    } else {
        rtos_printf("tud_device_control_get_ctrl_ctx_cb required to use USB device control\n");
    }
}

static void device_control_usb_reset(uint8_t rhport)
{
  (void) rhport;

  rtos_printf("USB Device Control Driver Reset!\n");
}

static uint16_t device_control_usb_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass);

    TU_VERIFY(itf_desc->bNumEndpoints == 0);

    TU_VERIFY(device_control_ctx != NULL);

    uint16_t const drv_len = sizeof(tusb_desc_interface_t);
    TU_VERIFY(max_len >= drv_len);

    control_ret_t dc_ret;

    dc_ret = device_control_resources_register(device_control_ctx,
                                               pdMS_TO_TICKS(5000));

    if (dc_ret != CONTROL_SUCCESS) {
        rtos_printf("Device control resources failed to register for USB on tile %d\n", THIS_XCORE_TILE);
    } else {
        rtos_printf("Device control resources registered for USB on tile %d\n", THIS_XCORE_TILE);
    }
    TU_VERIFY(dc_ret == CONTROL_SUCCESS);

    rtos_printf("Device control USB interface #%d opened\n", itf_desc->bInterfaceNumber);

    return drv_len;
}

#define REQ_TYPE(direction, type, recipient) ((uint8_t) ((direction) << 7)) | ((uint8_t) ((type) << 5)) | (recipient)

bool device_control_usb_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    static uint8_t request_data[CFG_TUD_ENDPOINT0_SIZE];
    control_ret_t ret;
    size_t len;

    if (stage == CONTROL_STAGE_SETUP) {
        ret = device_control_request(device_control_ctx,
                                     request->wIndex,
                                     request->wValue,
                                     request->wLength);
        if (ret != CONTROL_SUCCESS) {
            rtos_printf("Bad command received: %02x, %02x, %d\n", request->wIndex, request->wValue, request->wLength);
            return false;
        }
    }

    switch (request->bmRequestType) {
    case REQ_TYPE(TUSB_DIR_OUT, TUSB_REQ_TYPE_VENDOR, TUSB_REQ_RCPT_DEVICE):

        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_xfer(rhport, request, request_data, request->wLength);
        } else if (stage == CONTROL_STAGE_DATA) {
            len = request->wLength;
            ret = device_control_payload_transfer(device_control_ctx,
                                                  request_data, &len,
                                                  CONTROL_HOST_TO_DEVICE);
            if (ret == CONTROL_SUCCESS) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }

    case REQ_TYPE(TUSB_DIR_IN, TUSB_REQ_TYPE_VENDOR, TUSB_REQ_RCPT_DEVICE):

        if (stage == CONTROL_STAGE_SETUP) {
            len = CFG_TUD_ENDPOINT0_SIZE;
            ret = device_control_payload_transfer(device_control_ctx,
                                                  request_data, &len,
                                                  CONTROL_DEVICE_TO_HOST);

            if (ret == CONTROL_SUCCESS) {
                return tud_control_xfer(rhport, request, request_data, len);
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    return false;
}

const usbd_class_driver_t device_control_usb_app_driver = {
    DRIVER_NAME("XMOS-DEVICE-CONTROL")
    .init = device_control_usb_init,
    .reset = device_control_usb_reset,
    .open = device_control_usb_open,
    .control_xfer_cb = NULL,
    .xfer_cb = NULL,
    .sof = NULL,
};
