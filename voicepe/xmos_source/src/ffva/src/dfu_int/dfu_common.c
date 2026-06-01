// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#define DEBUG_UNIT DFU_COMMON
#ifndef DEBUG_PRINT_ENABLE_DFU_COMMON
#define DEBUG_PRINT_ENABLE_DFU_COMMON 0
#endif
#include "debug_print.h"

#include <stddef.h>
#include <stdint.h>
#include "quadflashlib.h"

#include "dfu_common.h"
#include "rtos_dfu_image.h"
#include "rtos_qspi_flash.h"
#include "platform/driver_instances.h"
#include "platform/platform_conf.h" // needed for appconfI2C_DFU_ENABLED

static size_t bytes_avail = 0;
static uint32_t dn_base_addr = 0;
static size_t total_len = 0;

uint32_t dfu_common_write_to_flash(uint8_t alt,
                                   uint16_t block_num,
                                   uint8_t const *data,
                                   uint16_t length)
{
    rtos_printf("Received Alt %d BlockNum %d of length %d\n", alt, block_num, length);
    uint32_t return_value = 0; // DFU_STATUS_OK

    unsigned data_partition_base_addr = rtos_dfu_image_get_data_partition_addr(dfu_image_ctx);
    switch(alt) {
        default:
        case 0:
            return_value = 3; //DFU_STATUS_ERR_WRITE
            break;
        case 1:
            if (dn_base_addr == 0) {
                total_len = 0;
                dn_base_addr = rtos_dfu_image_get_upgrade_addr(dfu_image_ctx);
                bytes_avail = data_partition_base_addr - dn_base_addr;
            }
            /* fallthrough */
        case 2:
            if (dn_base_addr == 0) {
                total_len = 0;
                dn_base_addr = data_partition_base_addr;
                bytes_avail = rtos_qspi_flash_size_get(qspi_flash_ctx) - dn_base_addr;
            }
            rtos_printf("Using addr 0x%x\nsize %u\n", dn_base_addr, bytes_avail);
            if(length > 0) {
                unsigned cur_addr = dn_base_addr + (block_num * length);
                if((bytes_avail - total_len) >= length) {
                    rtos_printf("write %d at 0x%x\n", length, cur_addr);

                    size_t sector_size = rtos_qspi_flash_sector_size_get(qspi_flash_ctx);
                    xassert(length == sector_size);

                    uint8_t *tmp_buf = rtos_osal_malloc( sizeof(uint8_t) * sector_size);
                    rtos_qspi_flash_lock(qspi_flash_ctx);
                    {
                        rtos_qspi_flash_read(
                                qspi_flash_ctx,
                                tmp_buf,
                                cur_addr,
                                sector_size);
                        memcpy(tmp_buf, data, length);
                        rtos_qspi_flash_erase(
                                qspi_flash_ctx,
                                cur_addr,
                                sector_size);
                        rtos_qspi_flash_write(
                                qspi_flash_ctx,
                                (uint8_t *) tmp_buf,
                                cur_addr,
                                sector_size);
                    }
                    rtos_qspi_flash_unlock(qspi_flash_ctx);
                    rtos_osal_free(tmp_buf);
                    total_len += length;
                } else {
                    rtos_printf("Insufficient space\n");
                    return_value = 8; //DFU_STATUS_ERR_ADDRESS;
                }
            }

            break;
    }

    return return_value;
}

uint32_t dfu_common_make_manifest()
{
    debug_printf("Download completed, enter manifestation\n");

    /* Perform a read to ensure all writes have been flushed */
    uint32_t dummy = 0;
    rtos_qspi_flash_read(
        qspi_flash_ctx,
        (uint8_t *)&dummy,
        0,
        sizeof(dummy));

    /* Reset download */
    dn_base_addr = 0;

    // flashing op for manifest is complete without error
    // Application can perform checksum.
    // Should it fail, return appropriate status such as errVERIFY.
    return 0; // DFU_STATUS_OK
}

uint16_t dfu_common_read_from_flash(uint8_t alt,
                                    uint16_t block_num,
                                    uint8_t *data,
                                    uint16_t length)
{
    uint32_t endaddr = 0;
    uint16_t retval = 0;
    uint32_t addr = block_num * length;
  
    rtos_printf("Upload Alt %d BlockNum %d of length %d\n", alt, block_num, length);

    switch(alt) {
        default:
            break;
        case 0:
            if (rtos_dfu_image_get_factory_size(dfu_image_ctx) > 0) {
                addr += rtos_dfu_image_get_factory_addr(dfu_image_ctx);
                endaddr = rtos_dfu_image_get_factory_addr(dfu_image_ctx) + rtos_dfu_image_get_factory_size(dfu_image_ctx);
            }
            break;
        case 1:
            if (rtos_dfu_image_get_upgrade_size(dfu_image_ctx) > 0) {
                addr += rtos_dfu_image_get_upgrade_addr(dfu_image_ctx);
                endaddr = rtos_dfu_image_get_upgrade_addr(dfu_image_ctx) + rtos_dfu_image_get_upgrade_size(dfu_image_ctx);
            }
            break;
        case 2:
            if ((rtos_qspi_flash_size_get(qspi_flash_ctx) - rtos_dfu_image_get_data_partition_addr(dfu_image_ctx)) > 0) {
                addr += rtos_dfu_image_get_data_partition_addr(dfu_image_ctx);
                endaddr = rtos_qspi_flash_size_get(qspi_flash_ctx);  /* End of flash */
            }
            break;
    }

    if (addr < endaddr) {
        rtos_qspi_flash_read(qspi_flash_ctx, data, addr, length);
        retval = length;
    }
    return retval;
}

void reboot(void)
{
    rtos_printf("Reboot initiated by tile:0x%x\n", get_local_tile_id());
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_PRESCALER_WRAP_NUM, (24000 - 1));
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_COUNT_NUM, 100);
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_PRESCALER_NUM, 0); // Reset counter
    write_sswitch_reg_no_ack(get_local_tile_id(), XS1_SSWITCH_WATCHDOG_CFG_NUM, (1 << XS1_WATCHDOG_COUNT_ENABLE_SHIFT) | (1 << XS1_WATCHDOG_TRIGGER_ENABLE_SHIFT) );
    // If running DFU over I2C this function returns to the application, so that the I2C connection can be closed
 #if ! appconfI2C_DFU_ENABLED
    while(1) {;}
#endif
}