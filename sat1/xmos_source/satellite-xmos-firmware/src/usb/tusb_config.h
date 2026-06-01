/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2021 XMOS LIMITED
 * Copyright (c) 2025 FutureProofHomes Inc. (mischa.siekmann@futureproofhomes.net)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#include <stdint.h>
#include "app_conf.h"
#include "audio_pipeline.h"
#include "rtos_printf.h"

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUSB_RHPORT0_MODE      (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#define CFG_TUSB_OS                OPT_OS_CUSTOM

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG             0
#endif

#define CFG_TUSB_MEM_ALIGN         __attribute__ ((aligned(8)))

#ifndef CFG_TUSB_DEBUG_PRINTF
#ifdef rtos_printf
#define CFG_TUSB_DEBUG_PRINTF      rtos_printf
#endif
#endif

#ifndef appconfUSB_CDC_ENABLED
#define appconfUSB_CDC_ENABLED     0
#endif

#ifndef appconfUSB_AUDIO_ENABLED
#define appconfUSB_AUDIO_ENABLED   0
#endif


//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_EP_MAX            12
#define CFG_TUD_TASK_QUEUE_SZ     8
#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_XCORE_INTERRUPT_CORE     appconfUSB_INTERRUPT_CORE
#define CFG_TUD_XCORE_SOF_INTERRUPT_CORE appconfUSB_SOF_INTERRUPT_CORE
#define CFG_TUD_XCORE_IO_CORE_MASK       (1 << appconfXUD_IO_CORE)

//------------- CLASS -------------//
#define CFG_TUD_DFU               1
#define CFG_TUD_CDC               appconfUSB_CDC_ENABLED
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_AUDIO             appconfUSB_AUDIO_ENABLED
#define CFG_TUD_VENDOR            0


//--------------------------------------------------------------------
// DFU DRIVER CONFIGURATION
//--------------------------------------------------------------------
// DFU buffer size, it has to be set to the buffer size used in TUD_DFU_DESCRIPTOR
#define CFG_TUD_DFU_XFER_BUFSIZE    4096

#if appconfUSB_CDC_ENABLED
//--------------------------------------------------------------------
// Communication Device Class DRIVER CONFIGURATION
//--------------------------------------------------------------------

// CDC buffer sizes
#define CFG_TUD_CDC_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#define CFG_TUD_CDC_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

// CDC Endpoint transfer buffer size, more is faster
#define CFG_TUD_CDC_EP_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------
#if CFG_TUD_AUDIO
extern const uint16_t tud_audio_desc_lengths[CFG_TUD_AUDIO];

#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                       tud_audio_desc_lengths[0]
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT                       1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ                    64

/* TODO make these configurable in app_conf? */
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX          2
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX          2

#if appconfUSB_AUDIO_MODE == appconfUSB_AUDIO_RELEASE
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                  2
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX                  2
#else
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                  6
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX                  2
#endif

#if (appconfMIC_SRC_DEFAULT == appconfMIC_SRC_USB)
// In appconfMIC_SRC_USB, we wait forever for input mic and AEC reference channels
// will not overflow output
#define USB_AUDIO_RECV_DELAY                                portMAX_DELAY
#else
// In or any other mode, we do not wait for input AEC reference channels.
//  The reference will be all zeros if no AEC reference is received.
//  This is the typical mode.
#define USB_AUDIO_RECV_DELAY                                0
#endif

// EP and buffer sizes
#define appconfUSB_SPK_SAMPLE_RATE                    16000
#define AUDIO_FRAMES_PER_USB_SPK_FRAME                (appconfUSB_SPK_SAMPLE_RATE / 1000)

#define appconfUSB_MICS_SAMPLE_RATE                   16000
#define AUDIO_FRAMES_PER_USB_MICS_FRAME               (appconfUSB_MICS_SAMPLE_RATE / 1000)


#if appconfUSB_AUDIO_SAMPLE_RATE == 48000
#define USB_TASK_STACK_SIZE                          2000
#endif

// To support USB Adaptive/Asynchronous, maximum packet size must be large enough to accommodate an extra set of samples per frame.
// Adding 1 to AUDIO_SAMPLES_PER_USB_FRAME allows this.
#define CFG_TUD_AUDIO_ENABLE_EP_IN                  1
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ               ((AUDIO_FRAMES_PER_USB_MICS_FRAME + 1) * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX           (CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ)    // Maximum EP IN size for all AS alternate settings used
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ        CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ

#define CFG_TUD_AUDIO_ENABLE_EP_OUT                 1
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ              ((AUDIO_FRAMES_PER_USB_SPK_FRAME + 1) * CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX          (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ + 2)   // Maximum EP OUT size for all AS alternate settings used. Plus 2 for CRC
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ       CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ*3
#endif

#endif /* _TUSB_CONFIG_H_ */
