# This file provides overrides to the Doxyfile configuration

PROJECT_NAME = XCORE-VOICE
PROJECT_BRIEF = "XCORE-VOICE User Guide"

EXTRACT_STATIC = YES

PREDEFINED = __DOXYGEN__=1 
PREDEFINED += DWORD_ALIGNED=
PREDEFINED += __attribute__((weak))=
PREDEFINED += C_API= MA_C_API= C_API_START= C_API_END= EXTERN_C=

# Core library APIs
INPUT += ../modules/core/modules/xcore_math/lib_xcore_math/lib_xcore_math/api ../modules/core/modules/xcore_math/lib_xcore_math/doc/programming_guide

# Voice DSP library APIs
INPUT += ../modules/voice/modules/lib_aec/api ../modules/voice/examples/bare-metal/shared_src/aec/
INPUT += ../modules/voice/modules/lib_ns/api
INPUT += ../modules/voice/modules/lib_agc/api
INPUT += ../modules/voice/modules/lib_adec/api
INPUT += ../modules/voice/modules/lib_ic/api
INPUT += ../modules/voice/modules/lib_vnr/api/common ../modules/voice/modules/lib_vnr/api/features ../modules/voice/modules/lib_vnr/api/inference

# RTOS driver APIs
INPUT += ../modules/rtos/modules/drivers 

# RTOS SW Services
INPUT += ../modules/rtos/modules/sw_services/device_control/host
INPUT += ../modules/rtos/modules/sw_services/device_control/api
INPUT += ../modules/rtos/modules/sw_services/concurrency_support/api
INPUT += ../modules/rtos/modules/sw_services/generic_pipeline/api

# IO library APIs
INPUT += ../modules/io/modules/i2c/api
INPUT += ../modules/io/modules/i2s/api
INPUT += ../modules/io/modules/spi/api
INPUT += ../modules/io/modules/uart/api
INPUT += ../modules/io/modules/mic_array/lib_mic_array/api ../modules/io/modules/mic_array/etc/vanilla ../modules/io/modules/mic_array/doc/programming_guide/src

# QSPI library APIs
INPUT += ../modules/lib_qspi_fast_read/lib_qspi_fast_read/api

# XCORE-VOICE APIs
INPUT += ../modules/asr/

USE_MATHJAX = YES
MATHJAX_FORMAT = HTML-CSS
MATHJAX_RELPATH = https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/

# Make short MATHJAX sequences prettier
ALIASES += math{1}="@f$\1@f$"
ALIASES += vector{1}="@f$\bar \1@f$"
ALIASES += operation{1}="@par Operation Performed^^@parblock^^@f{aligned}{ \1 @f}^^@endparblock"

ALIASES += "beginrst=^^\verbatim embed:rst^^"
ALIASES += "endrst=\endverbatim"