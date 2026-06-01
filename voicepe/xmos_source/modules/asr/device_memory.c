// Copyright 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <xcore/assert.h>

#include "device_memory.h"

void *devmem_malloc(devmem_manager_t *ctx, size_t size) {
    xassert(ctx);    
    xassert(ctx->malloc);    
    return ctx->malloc(size);
}

void devmem_free(devmem_manager_t *ctx, void *ptr) {
    xassert(ctx);    
    xassert(ctx->free);    
    ctx->free(ptr);
}

void devmem_read_ext(devmem_manager_t *ctx, void *dest, const void * src, size_t n) {
    xassert(ctx);    
    xassert(ctx->read_ext);
    xassert((intptr_t)src % 4 == 0);
    ctx->read_ext(dest, src, n);
}

int devmem_read_ext_async(devmem_manager_t *ctx, void *dest, const void * src, size_t n) {
    xassert(ctx);    
    xassert(ctx->read_ext);    
    xassert((intptr_t)src % 4 == 0);
    return ctx->read_ext_async(dest, src, n);
}

void devmem_read_ext_wait(devmem_manager_t *ctx, int handle) {
    xassert(ctx);    
    xassert(ctx->read_ext_wait);    
    ctx->read_ext_wait(handle);
}