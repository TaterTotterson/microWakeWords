/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2023 XMOS LIMITED
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

 #include "tusb_config.h"
 #include "usb/usb_descriptors.h"
#include "tusb.h"
 #include "class/dfu/dfu_device.h"

 #define XMOS_VID            0x20B1
 #define SATELLITE1_PID      0x4A05


//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
     .bcdUSB              = 0x0201,   
     // Use Interface Association Descriptor (IAD) for CDC
     // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
     .bDeviceClass       = 0, //TUSB_CLASS_MISC,
     .bDeviceSubClass    = 0, //MISC_SUBCLASS_COMMON,
     .bDeviceProtocol    = 0, //MISC_PROTOCOL_IAD,
 
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = XMOS_VID,
     .idProduct         = SATELLITE1_PID,
    .bcdDevice          = 0x0001,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const* tud_descriptor_device_cb(void)
{
    return (uint8_t const*) &desc_device;
}

// MSOS 2.0 descriptor copied from the examples in https://github.com/xmos/xcore_iot/blob/develop/test/usb/tinyusb_demos/webusb_serial/src/usb_descriptors.c#L152
// and https://github.com/pololu/libusbp/blob/master/test/firmware/wixel/main.c#L460
#define MS_OS_20_DESC_LEN  0xB2
#define REQUEST_GET_MS_DESCRIPTOR    0x20

// Microsoft OS 2.0 Descriptors, Table 8
#define MS_OS_20_DESCRIPTOR_INDEX 7

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

uint8_t const desc_bos[] =
{
  // total length, number of device caps
  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

  // Microsoft OS 2.0 descriptor
  TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, REQUEST_GET_MS_DESCRIPTOR)
};

uint8_t const desc_ms_os_20[] =
{
  // Set header: length, type, windows version, total length
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

  // Configuration subset header: length, type, configuration index, reserved, configuration total length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),

  // Function Subset header: length, type, first interface, reserved, subset length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_DFU_MODE, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),

  // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

  // MS OS 2.0 Registry property descriptor: length, type
  U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08-0x08-0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
  U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
  'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
  'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
  U16_TO_U8S_LE(0x0050), // wPropertyDataLength
	//bPropertyData: “{975F44D9-0D08-43FD-8B3E-127CA8AFFF9D}”.
  '{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00, '4', 0x00, 'D', 0x00, '9', 0x00, '-', 0x00,
  '0', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00, '-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00, 'D', 0x00, '-', 0x00,
  '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00, 'C', 0x00, 'A', 0x00,
  '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00, '9', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t const * tud_descriptor_bos_cb(void)
{
  return desc_bos;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    switch (request->bRequest)
    {
        case REQUEST_GET_MS_DESCRIPTOR:
            if ( request->wIndex == MS_OS_20_DESCRIPTOR_INDEX )
            {
                if (stage != CONTROL_STAGE_SETUP) return true; // nothing to with DATA & ACK stage
                // Send Microsoft OS 2.0 compatible descriptor
                return tud_control_xfer(rhport, request, (void*)(uintptr_t) desc_ms_os_20, MS_OS_20_DESC_LEN);
            }else
            {
                return false;
            }

        default:
            return false;
    }
}



//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
 // Number of DFU alt interfaces
#define DFU_ALT_COUNT                   3

// DFU functional attributes
#define DFU_FUNC_ATTRS (DFU_ATTR_CAN_UPLOAD | DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_WILL_DETACH | DFU_ATTR_MANIFESTATION_TOLERANT) 

#define EPNUM_AUDIO         0x01
#define EPNUM_CDC_NOTIF     0x83
#define EPNUM_CDC_OUT       0x04
#define EPNUM_CDC_IN        0x84
 
#if CFG_TUD_AUDIO
const size_t uac2_interface_descriptors_length =
        TUD_AUDIO_DESC_CLK_SRC_LEN
#if AUDIO_OUTPUT_ENABLED
        + TUD_AUDIO_DESC_INPUT_TERM_LEN
#if 0
        + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN
#endif
        + TUD_AUDIO_DESC_OUTPUT_TERM_LEN
#endif
#if AUDIO_INPUT_ENABLED
        + TUD_AUDIO_DESC_INPUT_TERM_LEN
#if 0
        + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN
#endif
        + TUD_AUDIO_DESC_OUTPUT_TERM_LEN
#endif
        ;


const size_t uac2_total_descriptors_length =
        TUD_AUDIO_DESC_IAD_LEN +
        TUD_AUDIO_DESC_STD_AC_LEN +
        TUD_AUDIO_DESC_CS_AC_LEN +
        uac2_interface_descriptors_length
#if AUDIO_OUTPUT_ENABLED
        + TUD_AUDIO_DESC_STD_AS_INT_LEN
        + TUD_AUDIO_DESC_STD_AS_INT_LEN
        + TUD_AUDIO_DESC_CS_AS_INT_LEN
        + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN
        + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN
        + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN
#endif
#if AUDIO_INPUT_ENABLED
        + TUD_AUDIO_DESC_STD_AS_INT_LEN
        + TUD_AUDIO_DESC_STD_AS_INT_LEN
        + TUD_AUDIO_DESC_CS_AS_INT_LEN
        + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN
        + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN
        + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN
#endif
        ;

// List of audio descriptor lengths which is required by audio driver - you need as many entries as CFG_TUD_AUDIO - unfortunately this is not possible to determine otherwise
const uint16_t tud_audio_desc_lengths[CFG_TUD_AUDIO] = {
        uac2_total_descriptors_length
};
#else
const size_t uac2_total_descriptors_length = 0;
#endif

#define CONFIG_TOTAL_LEN        (TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(DFU_ALT_COUNT) + (CFG_TUD_CDC * TUD_CDC_DESC_LEN) + (CFG_TUD_AUDIO * uac2_total_descriptors_length))

#define AUDIO_INTERFACE_STRING_INDEX 4
#define DFU_INTERFACE_STRING_INDEX   5
#define CDC_STRING_INDEX             8

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 400),

#if CFG_TUD_AUDIO
    /* Standard Interface Association Descriptor (IAD) */
    TUD_AUDIO_DESC_IAD(/*_firstitfs*/ ITF_NUM_AUDIO_CONTROL, /*_nitfs*/ 1+AUDIO_OUTPUT_ENABLED+AUDIO_INPUT_ENABLED, /*_stridx*/ 0x00),
    
    /* AudioControl (AC) */
    /* Standard AC Interface Descriptor(4.7.1) */
    TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ ITF_NUM_AUDIO_CONTROL, /*_nEPs*/ 0x00, /*_stridx*/ AUDIO_INTERFACE_STRING_INDEX),
    
    /* Class-Specific AC Interface Header Descriptor(4.7.2) */
    
    TUD_AUDIO_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_OTHER, /*_totallen*/ uac2_interface_descriptors_length, /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),
    /* Clock Source Descriptor(4.7.2.1) */
    /* _attr: internal clock with programmable frequency, internal clock is free running (D2 = 0b0) */
    TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_CLOCK, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK, /*_ctrl*/ (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_VAL_POS) | (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), /*_assocTerm*/ 0x00,  /*_stridx*/ 0x00),

#if AUDIO_OUTPUT_ENABLED
    /* Input Terminal Descriptor(4.7.2.4) */
    TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_clkid*/ UAC2_ENTITY_CLOCK, /*_nchannelslogical*/ CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ AUDIO_CTRL_NONE, /*_stridx*/ 0x00),
    /* Output Terminal Descriptor(4.7.2.5) */
    TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_SPK_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_OUT_GENERIC_SPEAKER, /*_assocTerm*/ 0x00, /*_srcid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_clkid*/ UAC2_ENTITY_CLOCK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_stridx*/ 0x00),
#endif
#if AUDIO_INPUT_ENABLED
    /* Input Terminal Descriptor(4.7.2.4) */
    TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_MIC_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm*/ 0x00, /*_clkid*/ UAC2_ENTITY_CLOCK, /*_nchannelslogical*/ CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ AUDIO_CTRL_NONE, /*_stridx*/ 0x00),
    #if 0
    /* Feature Unit Descriptor(4.7.2.8) */
    TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL(/*_unitid*/ UAC2_ENTITY_MIC_FEATURE_UNIT, /*_srcid*/ UAC2_ENTITY_MIC_INPUT_TERMINAL, /*_ctrlch0master*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_ctrlch1*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_stridx*/ 0x00),
    #endif

    /* Output Terminal Descriptor(4.7.2.5) */
    TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_srcid*/ UAC2_ENTITY_MIC_INPUT_TERMINAL, /*_clkid*/ UAC2_ENTITY_CLOCK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_stridx*/ 0x00),
#endif

#if AUDIO_OUTPUT_ENABLED
    /* AudioStreaming (AS) interface descriptors */
    /* Standard AS Interface Descriptor(4.9.1) */
    /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */
    TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ ITF_NUM_AUDIO_STREAMING_SPK, /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),
    /* Standard AS Interface Descriptor(4.9.1) */
    /* Interface 1, Alternate 1 - alternate interface for data streaming */
    TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ ITF_NUM_AUDIO_STREAMING_SPK, /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00),
    
    /* Class-Specific AS Interface Descriptor(4.9.2) */
    TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),
    /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */
    TUD_AUDIO_DESC_TYPE_I_FORMAT(CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX*8),
    /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */
    TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ EPNUM_AUDIO, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ADAPTIVE | TUSB_ISO_EP_ATT_IMPLICIT_FB /*| TUSB_ISO_EP_ATT_DATA*/), /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ, /*_interval*/ (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED) ? 0x04 : 0x01),
    /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */
    TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0003),
#endif

#if AUDIO_INPUT_ENABLED
    /* AudioStreaming (AS) interface descriptors */    
    /* Standard AS Interface Descriptor(4.9.1) */
    
    /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */
    TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ ITF_NUM_AUDIO_STREAMING_MIC, /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),
    /* Standard AS Interface Descriptor(4.9.1) */
    /* Interface 1, Alternate 1 - alternate interface for data streaming */
    TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ ITF_NUM_AUDIO_STREAMING_MIC, /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00),
    
    /* Class-Specific AS Interface Descriptor(4.9.2) */
    TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),
    /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */
    TUD_AUDIO_DESC_TYPE_I_FORMAT(CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX*8),
    /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */
    TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ 0x80 | EPNUM_AUDIO, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ADAPTIVE /*| TUSB_ISO_EP_ATT_IMPLICIT_FB */ | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ, /*_interval*/ (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED) ? 0x04 : 0x01),
    /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */
    TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0003),
#endif
#endif     
    // Interface number, Alternate count, starting string index, attributes, detach timeout, transfer size
    TUD_DFU_DESCRIPTOR(ITF_NUM_DFU_MODE, DFU_ALT_COUNT, DFU_INTERFACE_STRING_INDEX, DFU_FUNC_ATTRS, 1000, CFG_TUD_DFU_XFER_BUFSIZE),

#if CFG_TUD_CDC     
     //CDC Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, CDC_STRING_INDEX, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),
#endif
     
 };
 
 
 

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index; // for multiple configurations
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
 char const *string_desc_arr[] = {
    (const char[]) {0x09, 0x04}, // 0: is supported language is English (0x0409)
    "XMOS",                      // 1: Manufacturer
    "SATELLITE1",                // 2: Product
    "123456",                    // 3: Serials
    "SATELLITE1",                // 4: Audio Interface
    "DFU FACTORY",               // 5: DFU device
    "DFU UPGRADE",               // 6: DFU device
    "DFU DATAPARTITION",         // 7: DFU device
 #if appconfUSB_CDC_ENABLED
    "SATELLITE1 CDC",            // 8: CDC
 #endif       
        };

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index,
                                         uint16_t langid)
{
    (void) langid;

    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        // Convert ASCII string into UTF-16

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            return NULL;

        const char *str = string_desc_arr[index];

        // Cap at max char
        chr_count = strlen(str);
        if (chr_count > 31)
            chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}
