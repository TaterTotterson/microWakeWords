// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#pragma once

#include "mic_array.h"


C_API_START

/**
 * @brief Initializes the mic array module. (Vanilla API only)
 * 
 * Initializes the contexts for the decimator thread and configures
 * the clocks and ports for PDM reception.
 * 
 * After calling this, the PDM clock is active and signaling, but the PDM rx
 * service (ISR) has not yet been activated, so received PDM samples are
 * ignored. The real-time condition is not yet active.
 * 
 * @param pdm_res   Hardware resources required by the mic array module.
 */
MA_C_API
void ma_vanilla_init();


/**
 * @brief Entry point for decimator thread and PDM rx. (Vanilla API only)
 * 
 * This function sets up and activates the PDM rx service in ISR mode, and then
 * immediately begins executing the decimator.
 * 
 * After calling this the real-time condition is active, meaning there must be
 * another thread waiting to pull frames from the other end of `c_frames_out`
 * as they become available.
 * 
 * @param c_frames_out  (Non-streaming) Channel over which to send processed
 *                      frames of audio.
 */
MA_C_API
void ma_vanilla_task(chanend_t c_frames_out);

    
C_API_END
