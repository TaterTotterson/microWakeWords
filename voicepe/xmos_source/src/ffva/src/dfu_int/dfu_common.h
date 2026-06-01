// Copyright 2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <stdint.h>

// Define the delay to wait before rebooting the device after a successful download
#define DFU_REBOOT_DELAY_MS 100
// Define the timeout for the download operation for UA
#define DOWNLOAD_TIMEOUT_MS 10

// Define variable timeouts for INT to try to bring execution time down
#define DOWNLOAD_TIMEOUT_ERASE_MS 10
#define DOWNLOAD_TIMEOUT_WRITE_MS 3
#define DOWNLOAD_TIMEOUT_BUFFER_MS 1

/**
 * \brief Handle a DFU request to write some data to the flash memory.
 *
 * This function will write \p length bytes of \p data
 *   to the flash memory. The correct memory partition is selected based on
 *   the value of \p alt. The data is written to the flash memory at the
 *   address specified by \p block_num.
 *
 * \param[in] alt           Interface to identify the memory partition to write to.
 * \param[in] block_num     The block number used to calculate the address to write to.
 * \param[in] data          Buffer containing \p length valid bytes of data.
 * \param[in] length        The number of bytes present in \p data.
 *
 * \return                  0 if the write operation was successful, a non-zero error value otherwise.
 */
uint32_t dfu_common_write_to_flash(uint8_t alt,
                                   uint16_t block_num,
                                   uint8_t const *data,
                                   uint16_t length);

/**
 * \brief Handle a DFU request to perform a manifestation phase.
 *
 * This function will ensure that all the data to be written to the flash memory
 *   are flushed, and it resets the necessary variables to prepare for the next
 *   download operation.
 *
 * \return                  0 if the write operation was successful, a non-zero error value otherwise.
*/
uint32_t dfu_common_make_manifest();

/**
 * \brief Handle a DFU request to read some data from the flash memory.
 *
 * This function will read \p length bytes of \p data
 *   from the flash memory. The correct memory partition is selected based on
 *   the value of \p alt.  The data is read from the flash memory at the
 *   address specified by \p block_num.
 *
 * \param[in] alt           Interface to identify the memory partition to read from.
 * \param[in] block_num     The block number used to calculate the address to read from.
 * \param[in] data          Buffer to store the data read from the memory.
 * \param[in] length        The number of bytes to copy to \p data.
 *
 * \return                  0 if the write operation was successful, a non-zero error value otherwise.
 */
uint16_t dfu_common_read_from_flash(uint8_t alt,
                                    uint16_t block_num,
                                    uint8_t *data,
                                    uint16_t length);

/**
 * \brief Reboot the device.
 */
void reboot(void);
