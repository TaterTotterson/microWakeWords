// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

// This file intentionally only includes pure generic C constructs to allow compilation and testing by an x86 processor.
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "tusb_config.h"

#if CFG_TUD_AUDIO
uint32_t determine_USB_audio_rate(uint32_t timestamp,
                                    uint32_t data_length,
                                    uint32_t direction,
                                    bool update);
void reset_state();
void sof_toggle();
#endif