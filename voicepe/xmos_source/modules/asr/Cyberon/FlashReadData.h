// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#ifndef FLASH_READ_EXT_H_
#define FLASH_READ_EXT_H_

#include <stddef.h>
#include <xs1.h>

/**
 * Load model data.
 *
 * @param[out] dest Pointer to the memory location to copy to
 * @param[in]  src  Pointer to the memory location to copy from
 * @param[in]  size Number of bytes to copy
 */
size_t FlashReadData(void *dest, const void *src, size_t size);

#endif  // FLASH_READ_EXT_H_
