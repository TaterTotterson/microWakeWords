// Copyright 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#ifndef XCORE_DEVICE_MEMORY_H
#define XCORE_DEVICE_MEMORY_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <xcore/assert.h>

/**
 * Typedef to the device memory manager context.
 * Allows an application to define how memory allocation and
 * data loading is implemented.
 */
typedef struct devmem_manager_struct
{
    __attribute__((fptrgroup("devmem_malloc_fptr_grp")))
    void * (*malloc)(size_t size);
    
    __attribute__((fptrgroup("devmem_free_fptr_grp")))
    void (*free)(void *ptr);
    
    __attribute__((fptrgroup("devmem_read_ext_fptr_grp")))
    void (*read_ext)(void *dest, const void *src, size_t n);

    __attribute__((fptrgroup("devmem_read_ext_async_fptr_grp")))
    int (*read_ext_async)(void *dest, const void * src, size_t n);

    __attribute__((fptrgroup("devmem_read_ext_wait_fptr_grp")))
    void (*read_ext_wait)(int handle);
} devmem_manager_t;


/**
 * \addtogroup devmem_api devmem_api
 *
 * The public API for XCORE device memory management.
 * @{
 */

#define IS_SRAM(a)      ((uintptr_t)a < XS1_SWMEM_BASE)

#define IS_SWMEM(a)     (((uintptr_t)a >= XS1_SWMEM_BASE) && (((uintptr_t)a <= (XS1_SWMEM_BASE - 1 + XS1_SWMEM_SIZE))))

#define IS_FLASH(a)     IS_SWMEM(a)

/**
 * Memory allocation function that allows the application 
 * to provide an alternative implementation.
 * 
 * Call devmem_malloc instead of malloc
 * 
 * \param ctx      A pointer to the device memory context.
 * \param size     Number of bytes to allocate.
 *
 * \returns        A pointer to the beginning of newly allocated memory, or NULL on failure.
 */
void *devmem_malloc(devmem_manager_t *ctx, size_t size);

/**
 * Memory deallocation function that allows the application 
 * to provide an alternative implementation.
 * 
 * Call devmem_free instead of free
 * 
 * \param ctx      A pointer to the device memory context.
 * \param ptr      A pointer to the memory to deallocate.
 */
void devmem_free(devmem_manager_t *ctx, void *ptr);

/**
 * Synchronous extended memory read function that allows the application  
 * to provide an alternative implementation.  Blocks the callers thread 
 * until the read is completed.
 * 
 * Call devmem_read_ext instead of any other functions to read memory from 
 * flash, LPDDR or SDRAM. Modules are free to use memcpy if the dest and src 
 * are both SRAM addresses.
 * 
 * \param ctx      A pointer to the device memory context.
 * \param dest     A pointer to the destination array where the content is to be read.
 * \param src      A pointer to the word-aligned address of data to be read.
 * \param n        Number of bytes to read.
 */
void devmem_read_ext(devmem_manager_t *ctx, void *dest, const void * src, size_t n);

/**
 * Asynchronous extended memory read function that allows the application  
 * to provide an alternative implementation.
 * 
 * Call asr_read_ext_async instead of any other functions to read memory
 * from flash, LPDDR or SDRAM. 
 * 
 * \param ctx      A pointer to the device memory context.
 * \param dest     A pointer to the destination array where the content is to be read.
 * \param src      A pointer to the word-aligned address of data to be read.
 * \param n        Number of bytes to read.
 * 
 * \returns        A handle that can be used in a call to devmem_read_ext_wait.
 */
int devmem_read_ext_async(devmem_manager_t *ctx, void *dest, const void * src, size_t n);

/**
 * Wait in the caller's thread for an asynchronous extended memory read to finish.
 * 
 * \param ctx      A pointer to the device memory context.
 * \param handle   The devmem_read_ext_asyc handle to wait on.
 */
void devmem_read_ext_wait(devmem_manager_t *ctx, int handle);

/**@}*/

#endif // XCORE_DEVICE_MEMORY_H
