/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2021 XMOS LIMITED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <quadflashlib.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "tusb.h"
#include "dfu_common.h"

//--------------------------------------------------------------------+
// DFU callbacks
// Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
//--------------------------------------------------------------------+

// Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
// Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
// During this period, USB host won't try to communicate with us.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
    if ( state == DFU_DNBUSY ) {
        return 10;  // 10 ms
    } else if (state == DFU_MANIFEST) {
        // since we don't buffer entire image and do any flashing in manifest stage
        return 0;
    }
  
    return 0;
}

// Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
// This callback could be returned before flashing op is complete (async).
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length)
{
    dfu_status_t error_code = dfu_common_write_to_flash(alt,
                                                        block_num,
                                                        data,
                                                        length);
    tud_dfu_finish_flashing(error_code);
}

// Invoked when download process is complete, received DFU_DNLOAD (wLength=0) following by DFU_GETSTATUS (state=Manifest)
// Application can do checksum, or actual flashing if buffered entire image previously.
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_manifest_cb(uint8_t alt)
{
    (void)alt;

    dfu_state_t error_code = dfu_common_make_manifest();
    tud_dfu_finish_flashing(error_code);
}

// Invoked when received DFU_UPLOAD request
// Application must populate data with up to length bytes and
// Return the number of written bytes
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t* data, uint16_t length)
{
    return dfu_common_read_from_flash(alt,
                                      block_num,
                                      data,
                                      length);
}

// Invoked when the Host has terminated a download or upload transfer
void tud_dfu_abort_cb(uint8_t alt)
{
    (void) alt;
    rtos_printf("Host aborted transfer\n");
}

// Invoked when a DFU_DETACH request is received
void tud_dfu_detach_cb(void)
{
    rtos_printf("Host detach, we should probably reboot\n");
    reboot();
}
