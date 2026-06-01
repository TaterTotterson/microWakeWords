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
// #include "platform/driver_instances.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
// static void reboot(void);
#include "dfu_common.h"

//--------------------------------------------------------------------+
// DFU callbacks
// Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
//--------------------------------------------------------------------+

// static size_t total_len = 0;
// static size_t bytes_avail = 0;
// static uint32_t dn_base_addr = 0;

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
    // rtos_printf("Received Alt %d BlockNum %d of length %d\n", alt, block_num, length);

    // unsigned data_partition_base_addr = rtos_dfu_image_get_data_partition_addr(dfu_image_ctx);
    // switch(alt) {
    //     default:
    //     case 0:
    //         tud_dfu_finish_flashing(DFU_STATUS_ERR_WRITE);
    //         break;
    //     case 1:
    //         if (dn_base_addr == 0) {
    //             total_len = 0;
    //             dn_base_addr = rtos_dfu_image_get_upgrade_addr(dfu_image_ctx);
    //             bytes_avail = data_partition_base_addr - dn_base_addr;    
    //         }
    //         /* fallthrough */
    //     case 2:
    //         if (dn_base_addr == 0) {
    //             total_len = 0;
    //             dn_base_addr = data_partition_base_addr;
    //             bytes_avail = rtos_qspi_flash_size_get(qspi_flash_ctx) - dn_base_addr;    
    //         }
    //         rtos_printf("Using addr 0x%x\nsize %u\n", dn_base_addr, bytes_avail);
    //         if(length > 0) {
    //             unsigned cur_addr = dn_base_addr + (block_num * CFG_TUD_DFU_XFER_BUFSIZE);
    //             if((bytes_avail - total_len) >= length) {
    //                 rtos_printf("write %d at 0x%x\n", length, cur_addr);

    //                 size_t sector_size = rtos_qspi_flash_sector_size_get(qspi_flash_ctx);
    //                 xassert(CFG_TUD_DFU_XFER_BUFSIZE == sector_size);

    //                 uint8_t *tmp_buf = rtos_osal_malloc( sizeof(uint8_t) * sector_size);
    //                 rtos_qspi_flash_lock(qspi_flash_ctx);
    //                 {
    //                     rtos_qspi_flash_read(
    //                             qspi_flash_ctx,
    //                             tmp_buf,
    //                             cur_addr,
    //                             sector_size);
    //                     memcpy(tmp_buf, data, length);
    //                     rtos_qspi_flash_erase(
    //                             qspi_flash_ctx,
    //                             cur_addr,
    //                             sector_size);
    //                     rtos_qspi_flash_write(
    //                             qspi_flash_ctx,
    //                             (uint8_t *) tmp_buf,
    //                             cur_addr,
    //                             sector_size);
    //                 }
    //                 rtos_qspi_flash_unlock(qspi_flash_ctx);
    //                 rtos_osal_free(tmp_buf);
    //                 total_len += length;
    //             } else {
    //                 rtos_printf("Insufficient space\n");
    //                 tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);
    //             }
    //         }

    //         tud_dfu_finish_flashing(DFU_STATUS_OK);
    //         break;
    // }
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
    // (void) alt;
    // rtos_printf("Download completed, enter manifestation\n");

    // /* Perform a read to ensure all writes have been flushed */
    // uint32_t dummy = 0;
    // rtos_qspi_flash_read(
    //         qspi_flash_ctx,
    //         (uint8_t *)&dummy,
    //         0,
    //         sizeof(dummy));
    
    // /* Reset download */
    // dn_base_addr = 0;
    (void)alt;

    // // flashing op for manifest is complete without error
    // // Application can perform checksum, should it fail, use appropriate status such as errVERIFY.
    // tud_dfu_finish_flashing(DFU_STATUS_OK);
    dfu_state_t error_code = dfu_common_make_manifest();
    tud_dfu_finish_flashing(error_code);
}

// Invoked when received DFU_UPLOAD request
// Application must populate data with up to length bytes and
// Return the number of written bytes
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t* data, uint16_t length)
{
    // uint32_t endaddr = 0;
    // uint16_t retval = 0;
    // uint32_t addr = block_num * CFG_TUD_DFU_XFER_BUFSIZE;
  
    // rtos_printf("Upload Alt %d BlockNum %d of length %d\n", alt, block_num, length);

    // switch(alt) {
    //     default:
    //         break;
    //     case 0:
    //         if (rtos_dfu_image_get_factory_size(dfu_image_ctx) > 0) {
    //             addr += rtos_dfu_image_get_factory_addr(dfu_image_ctx);
    //             endaddr = rtos_dfu_image_get_factory_addr(dfu_image_ctx) + rtos_dfu_image_get_factory_size(dfu_image_ctx);
    //         }
    //         break;
    //     case 1:
    //         if (rtos_dfu_image_get_upgrade_size(dfu_image_ctx) > 0) {
    //             addr += rtos_dfu_image_get_upgrade_addr(dfu_image_ctx);
    //             endaddr = rtos_dfu_image_get_upgrade_addr(dfu_image_ctx) + rtos_dfu_image_get_upgrade_size(dfu_image_ctx);
    //         }
    //         break;
    //     case 2:
    //         if ((rtos_qspi_flash_size_get(qspi_flash_ctx) - rtos_dfu_image_get_data_partition_addr(dfu_image_ctx)) > 0) {
    //             addr += rtos_dfu_image_get_data_partition_addr(dfu_image_ctx);
    //             endaddr = rtos_qspi_flash_size_get(qspi_flash_ctx);  /* End of flash */
    //         }
    //         break;
    // }

    // if (addr < endaddr) {
    //     rtos_qspi_flash_read(qspi_flash_ctx, data, addr, length);
    //     retval = length;
    // }
    // return retval;
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

// static void reboot(void)
// {
//     rtos_printf("Reboot initiated by tile:0x%x\n", get_local_tile_id());
//     write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_COUNT_NUM, 0x10000);
//     write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_CFG_NUM, (1 << XS1_WATCHDOG_COUNT_ENABLE_SHIFT) | (1 << XS1_WATCHDOG_TRIGGER_ENABLE_SHIFT) );
//     while(1) {;}
// }
