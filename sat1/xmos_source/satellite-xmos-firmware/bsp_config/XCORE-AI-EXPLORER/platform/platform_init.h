// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef PLATFORM_INIT_H_
#define PLATFORM_INIT_H_

#include <xcore/chanend.h>

void platform_init(chanend_t other_tile_c);
void platform_start(void);

#endif /* PLATFORM_INIT_H_ */
