// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef USB_AUDIO_H_
#define USB_AUDIO_H_

#include "tusb_config.h"

#if CFG_TUD_AUDIO
#include "rtos_intertile.h"


/*
 * frame_buffers format assumes:
 *   processed_audio_frame
 *   reference_audio_frame
 *   raw_mic_audio_frame
 */
void usb_audio_send(rtos_intertile_t *intertile_ctx,
                    size_t frame_count,
                    int32_t **frame_buffers,
                    size_t num_chans);

/*
* frame_buffers format assumes:
*   reference_audio_frame
*   raw_mic_audio_frame
*/
void usb_audio_recv(rtos_intertile_t *intertile_ctx,
                    size_t frame_count,
                    int32_t **frame_buffers,
                    size_t num_chans);

void usb_audio_init(rtos_intertile_t *intertile_ctx, unsigned priority);


#endif

#endif /* USB_AUDIO_H_ */
