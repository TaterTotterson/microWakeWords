
## Create custom board targets for application
add_library(fph_ffva_board_support_satellite1 INTERFACE)
target_sources(fph_ffva_board_support_satellite1
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/platform/app_pll_ctrl.c
        ${CMAKE_CURRENT_LIST_DIR}/platform/driver_instances.c
        ${CMAKE_CURRENT_LIST_DIR}/platform/platform_init.c
        ${CMAKE_CURRENT_LIST_DIR}/platform/platform_start.c
)
target_include_directories(fph_ffva_board_support_satellite1
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(fph_ffva_board_support_satellite1
    INTERFACE
        core::general
        rtos::freertos
        rtos::drivers::general
        rtos::drivers::usb
        rtos::drivers::dfu_image
        fph::rtos_mic_array
        fph::i2s_sync
        fph::rtos_ws2812
)
target_compile_options(fph_ffva_board_support_satellite1
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/SATELLITE1.xn
)
target_link_options(fph_ffva_board_support_satellite1
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/SATELLITE1.xn
)

# MICS: 
# North: 0 (pin 0 falling edge) 
# South: 1 (pin 1 falling edge) 
# East:  4 (pin 0 rising edge ) 
# West:  5 (pin 1 rising edge )
# pins 2 & 3 are not used

# Keep East/West as the first two mics so the existing 2-mic voice/AEC
# pipeline remains stable. Add North/South for local 4-mic DoA testing.
set(MIC_MAPPING "4, 5, 0, 1")

target_compile_definitions(fph_ffva_board_support_satellite1
    INTERFACE
        XCOREAI_EXPLORER=1
        PLATFORM_SUPPORTS_TILE_0=1
        PLATFORM_SUPPORTS_TILE_1=1
        PLATFORM_SUPPORTS_TILE_2=0
        PLATFORM_SUPPORTS_TILE_3=0
        USB_TILE_NO=0
        USB_TILE=tile[USB_TILE_NO]

        MIC_ARRAY_CONFIG_MCLK_FREQ=24576000
        MIC_ARRAY_CONFIG_PDM_FREQ=3072000
        MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME=240
        MIC_ARRAY_CONFIG_USE_DDR=1
        MIC_ARRAY_CONFIG_MIC_INPUT=8
        MIC_ARRAY_CONFIG_MIC_COUNT=4
        MIC_ARRAY_CONFIG_INPUT_MAPPING={${MIC_MAPPING}}
        appconfAUDIO_PIPELINE_CHANNELS=2
        
        MIC_ARRAY_CONFIG_CLOCK_BLOCK_A=XS1_CLKBLK_1
        MIC_ARRAY_CONFIG_CLOCK_BLOCK_B=XS1_CLKBLK_2
        MIC_ARRAY_CONFIG_PORT_MCLK=PORT_MCLK_IN
        MIC_ARRAY_CONFIG_PORT_PDM_CLK=PORT_PDM_CLK
        MIC_ARRAY_CONFIG_PORT_PDM_DATA=PORT_PDM_DATA
)

## Create an alias
add_library(fph::ffva::satellite1 ALIAS fph_ffva_board_support_satellite1)
