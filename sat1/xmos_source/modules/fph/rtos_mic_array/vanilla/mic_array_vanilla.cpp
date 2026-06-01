// Copyright 2022 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <stdint.h>
#include <xcore/channel_streaming.h>
#include <xcore/interrupt.h>
#include <platform.h>

#include "mic_array_vanilla.h"
#include "mic_array/cpp/Prefab.hpp"
#include "mic_array.h"
#include "mic_array/etc/filters_default.h"


////// Check that all the required config macros have been defined. 

#ifndef MIC_ARRAY_CONFIG_MCLK_FREQ
# error Application must specify the master clock frequency by defining MIC_ARRAY_CONFIG_MCLK_FREQ.
#endif

#ifndef MIC_ARRAY_CONFIG_PDM_FREQ
# error Application must specify the PDM clock frequency by defining MIC_ARRAY_CONFIG_PDM_FREQ.
#endif

#ifndef MIC_ARRAY_CONFIG_MIC_COUNT
# error Application must specify the microphone count by defining MIC_ARRAY_CONFIG_MIC_COUNT.
#endif


////// Provide default values for optional config macros

#ifndef MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME    
# define MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME    (1)
#else
# if ((MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME) < 1)
#  error MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME must be positive.
# endif
#endif

#ifndef MIC_ARRAY_CONFIG_USE_DC_ELIMINATION
# define MIC_ARRAY_CONFIG_USE_DC_ELIMINATION    (1)
#endif

#ifndef MIC_ARRAY_CONFIG_PORT_MCLK
# define MIC_ARRAY_CONFIG_PORT_MCLK   (PORT_MCLK_IN_OUT)
#endif

#ifndef MIC_ARRAY_CONFIG_PORT_PDM_CLK
# define MIC_ARRAY_CONFIG_PORT_PDM_CLK  (PORT_PDM_CLK)
#endif

#ifndef MIC_ARRAY_CONFIG_PORT_PDM_DATA
# define MIC_ARRAY_CONFIG_PORT_PDM_DATA   (PORT_PDM_DATA)
#endif

#ifndef MIC_ARRAY_CONFIG_CLOCK_BLOCK_A          
# define MIC_ARRAY_CONFIG_CLOCK_BLOCK_A   (XS1_CLKBLK_1)
#endif

#ifndef MIC_ARRAY_CONFIG_CLOCK_BLOCK_B
# define MIC_ARRAY_CONFIG_CLOCK_BLOCK_B   (XS1_CLKBLK_2)
#endif

#ifndef MIC_ARRAY_CONFIG_USE_DDR
# define MIC_ARRAY_CONFIG_USE_DDR         ((MIC_ARRAY_CONFIG_MIC_COUNT)==2)
#endif

#ifndef MIC_ARRAY_CONFIG_MIC_INPUT
# define MIC_ARRAY_CONFIG_MIC_INPUT        (MIC_ARRAY_CONFIG_MIC_COUNT)
#endif

#ifndef MIC_ARRAY_CONFIG_INPUT_MAPPING
# define MIC_ARRAY_CONFIG_INPUT_MAPPING {0, 1}
#endif

////// Additional macros derived from others

#define MIC_ARRAY_CONFIG_MCLK_DIVIDER     ((MIC_ARRAY_CONFIG_MCLK_FREQ)       \
                                              /(MIC_ARRAY_CONFIG_PDM_FREQ))
#define MIC_ARRAY_CONFIG_OUT_SAMPLE_RATE    ((MIC_ARRAY_CONFIG_PDM_FREQ)      \
                                              /(STAGE2_DEC_FACTOR))

////// Any Additional correctness checks



////// Allocate needed objects

#if (!(MIC_ARRAY_CONFIG_USE_DDR))
pdm_rx_resources_t pdm_res = PDM_RX_RESOURCES_SDR(
                                MIC_ARRAY_CONFIG_PORT_MCLK,
                                MIC_ARRAY_CONFIG_PORT_PDM_CLK,
                                MIC_ARRAY_CONFIG_PORT_PDM_DATA,
                                MIC_ARRAY_CONFIG_CLOCK_BLOCK_A);
#else
pdm_rx_resources_t pdm_res = PDM_RX_RESOURCES_DDR(
                                MIC_ARRAY_CONFIG_PORT_MCLK,
                                MIC_ARRAY_CONFIG_PORT_PDM_CLK,
                                MIC_ARRAY_CONFIG_PORT_PDM_DATA,
                                MIC_ARRAY_CONFIG_CLOCK_BLOCK_A,
                                MIC_ARRAY_CONFIG_CLOCK_BLOCK_B);
#endif



using TMicArray = mic_array::prefab::BasicMicArray<
                        MIC_ARRAY_CONFIG_MIC_COUNT,
                        MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME,
                        MIC_ARRAY_CONFIG_USE_DC_ELIMINATION,
                        MIC_ARRAY_CONFIG_MIC_INPUT>;

TMicArray mics;


void ma_vanilla_init()
{
  unsigned input_mic_mapping[] = MIC_ARRAY_CONFIG_INPUT_MAPPING;
  _Static_assert(
        (sizeof(input_mic_mapping) / sizeof(unsigned)) == MIC_ARRAY_CONFIG_MIC_COUNT,
        "The number of elements in MIC_ARRAY_CONFIG_INPUT does not match MIC_ARRAY_CONFIG_MIC_COUNT"
  );
  
  mics.Init();
  mics.SetPort(pdm_res.p_pdm_mics);
  mics.PdmRx.MapChannels(input_mic_mapping);
  mic_array_resources_configure(&pdm_res, MIC_ARRAY_CONFIG_MCLK_DIVIDER);
  mic_array_pdm_clock_start(&pdm_res);
}


void ma_vanilla_task(
    chanend_t c_frames_out)
{
  mics.SetOutputChannel(c_frames_out);

  mics.InstallPdmRxISR();
  mics.UnmaskPdmRxISR();

  mics.ThreadEntry();
}