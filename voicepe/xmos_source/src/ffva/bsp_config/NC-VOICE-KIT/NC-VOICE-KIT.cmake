
## Create custom board targets for application
add_library(sln_voice_app_ffva_board_support_nc_voice_kit INTERFACE)
target_sources(sln_voice_app_ffva_board_support_nc_voice_kit
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/platform/app_pll_ctrl.c
        ${CMAKE_CURRENT_LIST_DIR}/platform/driver_instances.c
        ${CMAKE_CURRENT_LIST_DIR}/platform/platform_init.c
        ${CMAKE_CURRENT_LIST_DIR}/platform/platform_start.c
)
target_include_directories(sln_voice_app_ffva_board_support_nc_voice_kit
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(sln_voice_app_ffva_board_support_nc_voice_kit
    INTERFACE
        core::general
        rtos::freertos
        rtos::drivers::general
        rtos::drivers::audio
        rtos::drivers::usb
        rtos::drivers::dfu_image
)
target_compile_options(sln_voice_app_ffva_board_support_nc_voice_kit
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/NC-VOICE-KIT.xn
)
target_link_options(sln_voice_app_ffva_board_support_nc_voice_kit
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/NC-VOICE-KIT.xn
)
target_compile_definitions(sln_voice_app_ffva_board_support_nc_voice_kit
    INTERFACE
        XK_VOICE_L71=1
        PLATFORM_SUPPORTS_TILE_0=1
        PLATFORM_SUPPORTS_TILE_1=1
        PLATFORM_SUPPORTS_TILE_2=0
        PLATFORM_SUPPORTS_TILE_3=0
        USB_TILE_NO=0
        USB_TILE=tile[USB_TILE_NO]
        MIC_ARRAY_CONFIG_PDM_FREQ=3072000
        MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME=240
        MIC_ARRAY_CONFIG_MIC_COUNT=2
        MIC_ARRAY_CONFIG_CLOCK_BLOCK_A=XS1_CLKBLK_1
        MIC_ARRAY_CONFIG_CLOCK_BLOCK_B=XS1_CLKBLK_2
        MIC_ARRAY_CONFIG_PORT_MCLK=PORT_MCLK_IN_OUT
        MIC_ARRAY_CONFIG_PORT_PDM_CLK=PORT_PDM_CLK
        MIC_ARRAY_CONFIG_PORT_PDM_DATA=PORT_PDM_DATA
)

## Create an alias
add_library(sln_voice::app::ffva::nc_voice_kit ALIAS sln_voice_app_ffva_board_support_nc_voice_kit)
