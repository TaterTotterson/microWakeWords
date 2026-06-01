
#include <stddef.h>
#include <stdint.h>
#include "quadflashlib.h"

#include "configuration_common.h"
#include "rtos_qspi_flash.h"
#include "rtos_dfu_image.h"
#include "platform/driver_instances.h"
#include "platform/platform_conf.h"


static uint32_t data_partition_base_addr = 0;
static uint32_t flash_size = 0;
static uint32_t sector_size = 0;

static uint32_t conriguration_check_flash()
{
    if (sector_size == 0) return 1;
    if (data_partition_base_addr == flash_size) return 1;
    return 0;
}

uint32_t configuration_init()
{
    data_partition_base_addr = rtos_dfu_image_get_data_partition_addr(dfu_image_ctx);
    flash_size = rtos_qspi_flash_size_get(qspi_flash_ctx);
    sector_size = rtos_qspi_flash_sector_size_get(qspi_flash_ctx);
    return 0;
}

uint32_t configuration_write_to_flash(uint8_t zone,
                                        uint8_t const *data,
                                        uint16_t length)
{
    uint32_t cur_addr = 0;
    if (conriguration_check_flash() != 0) return 1; // flash config not right
    cur_addr = flash_size - (zone + 1) * sector_size;
    if (cur_addr < data_partition_base_addr) return 1; // flash config not right

    switch(zone) {
        case CONFIGURATION_FACTORY_ZONE:
        {
            return 2; // can't rewrite factory zone
        }
        break;

        case CONFIGURATION_CUSTOMER_ZONE:
        {
            if (length > sector_size) return 3; // write length too long
            rtos_printf("write %d at 0x%x\n", length, cur_addr);

            uint8_t *tmp_buf = rtos_osal_malloc( sizeof(uint8_t) * sector_size);
            rtos_printf("alloc heap at 0x%x\n", tmp_buf);
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
            return 0;
        }
        break;

        default:
        {
            return 4; // unknown cmd
        }
        break;
    }

    return 0;
}

uint32_t configuration_flush()
{
    debug_printf("configuration_flush\n");

    /* Perform a read to ensure all writes have been flushed */
    uint32_t dummy = 0;
    rtos_qspi_flash_read(
        qspi_flash_ctx,
        (uint8_t *)&dummy,
        data_partition_base_addr,
        sizeof(dummy));
    
    return 0;
}

uint16_t configuration_read_from_flash(uint8_t zone,
                                    uint8_t *data,
                                    uint16_t length)
{
    uint32_t cur_addr;
    if (conriguration_check_flash() != 0) return 1; // flash config not right
    cur_addr = flash_size - (zone + 1) * sector_size;
    if (cur_addr < data_partition_base_addr) return 1; // flash config not right

    if (length > sector_size) return 3; // read length too long
    rtos_printf("read %d at 0x%x\n", length, cur_addr);

    rtos_qspi_flash_read(qspi_flash_ctx, data, cur_addr, length);

    return 0;
}
