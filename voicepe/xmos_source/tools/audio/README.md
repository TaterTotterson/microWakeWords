# XCORE-VOICE Audio Testing Utilities

These utilities are currently supported on Linux or macOS.  

## Channel Order

The following channel orders are useful to keep in mind when running the utilities:

    The standard test vector channel order is: Mic 1, Mic 0, Ref L, Ref R
    
    XCORE-VOICE's input channel order is: Ref L, Ref R, Mic 0, Mic 1
    XCORE-VOICE's output channel order is: ASR, COMMS, Ref L, Ref R, Mic 0, Mic 1

## process_usb.sh

To build and flash the application run the following commands in the root of the repository:

    cmake -B build_test -DDEBUG_FFVA_USB_MIC_INPUT=1 -DCMAKE_TOOLCHAIN_FILE=xmos_cmake_toolchain/xs3a.cmake 
    cd build_test
    make example_ffva_ua_adec_altarch -j
    make flash_app_example_ffva_ua_adec_altarch -j

To process a list of wav files run the following command in the root of the repository:

    bash tools/audio/process_usb.sh build_test/example_ffva_ua_adec_altarch.xe <path-to-input-dir> <path-to-input-list> <path-to-output-dir> <path-to-amazon-wwe>

See `test/pipeline/README.rst` for info on the format of the input-list file.  

## process_wav.sh

Processes a wav file via USB and records the output wav file.  

Options:

- -c   Number of channels in input wav
- -r   Sample rate (default=16000)
- -a   Audio pipeline includes AEC (default=true)



