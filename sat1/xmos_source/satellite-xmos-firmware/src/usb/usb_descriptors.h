// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include "usb/tusb_config.h"


#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX > 0
#define AUDIO_INPUT_ENABLED 1
#else
#define AUDIO_INPUT_ENABLED 0
#endif
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX > 0
#define AUDIO_OUTPUT_ENABLED 1
#else
#define AUDIO_OUTPUT_ENABLED 0
#endif

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
#if AUDIO_OUTPUT_ENABLED
    ITF_NUM_AUDIO_STREAMING_SPK,
#endif
#if AUDIO_INPUT_ENABLED
    ITF_NUM_AUDIO_STREAMING_MIC,
#endif
    ITF_NUM_DFU_MODE,
#if CFG_TUD_CDC
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
#endif
    ITF_NUM_TOTAL
};


// Unit numbers are arbitrary selected
#define UAC2_ENTITY_CLOCK               0x01
// Speaker path
#define UAC2_ENTITY_SPK_INPUT_TERMINAL  0x11
#define UAC2_ENTITY_SPK_FEATURE_UNIT    0x12
#define UAC2_ENTITY_SPK_OUTPUT_TERMINAL 0x13

// Microphone path
#define UAC2_ENTITY_MIC_INPUT_TERMINAL  0x21
#define UAC2_ENTITY_MIC_FEATURE_UNIT    0x22
#define UAC2_ENTITY_MIC_OUTPUT_TERMINAL 0x23

#endif /* USB_DESCRIPTORS_H_ */
