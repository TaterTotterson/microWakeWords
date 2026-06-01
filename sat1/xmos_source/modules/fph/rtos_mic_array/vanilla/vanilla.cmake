
function( mic_array_vanilla_add
        TARGET_NAME
        MCLK_FREQ 
        PDM_FREQ
        MIC_COUNT
        SAMPLES_PER_FRAME )

  set( VANILLA_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}" )

  target_sources( ${TARGET_NAME} 
      PRIVATE ${VANILLA_DIR}/mic_array_vanilla.cpp )

  target_include_directories( ${TARGET_NAME}
      PRIVATE ${VANILLA_DIR} )

  target_compile_definitions( ${TARGET_NAME}
      PRIVATE MIC_ARRAY_CONFIG_MCLK_FREQ=${MCLK_FREQ} )
      
  target_compile_definitions( ${TARGET_NAME}
      PRIVATE MIC_ARRAY_CONFIG_PDM_FREQ=${PDM_FREQ} )
      
  target_compile_definitions( ${TARGET_NAME}
      PRIVATE MIC_ARRAY_CONFIG_MIC_COUNT=${MIC_COUNT} )

  target_compile_definitions( ${TARGET_NAME}
      PRIVATE MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME=${SAMPLES_PER_FRAME} )

endfunction()