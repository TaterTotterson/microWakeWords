# voice-kit-xmos-firmware

This repository contains the source code for the [esphome/voice-kit](https://github.com/esphome/voice-kit) XU316 microcontroller.

## Building the firmware

_NOTE: This guide was written using Debian Linux 12 (bookworm)_

  

**Step 1:** download and install the [XMOS XTC tools](https://www.xmos.com/software-tools/) for your operating system.

You must create an XMOS developer account to download the tools.

On Linux:

*   Click the "Linux 64-bit" link
*   Extract the tools to a directory (example: `~/XMOS` )
*   In a terminal, source the environment script (example: `source ~/XMOS/XTC/15.2.1/SetEnv` )
*   If done correctly, run `xcc --version` and it should work

*Note:* This process is nearly identical for macOS -- just use the macOS version of the tools and install into `/Applications`.



**Step 2:** clone the source repository.

You must have an [SSH key associated with your Github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) because the submodules are all referenced using SSH. If your SSH key is working, `ssh -T git@github.com` should succeed.

```bash
git clone --recurse-submodules git@github.com:esphome/voice-kit-xmos-firmware
cd voice-kit-xmos-firmware/
git checkout <branch>
git submodule update --init --recursive
```

  

**Step 3:** build the firmware.

Ensure you have at least `cmake` version 3.21 or higher installed.

On Linux, you also need to ensure that `libtinfo5` is installed with `sudo apt-get install libtinfo5`

These commands must be run from the same terminal where you sourced the XTC environment script, and must be run within the `voice-kit-xmos-firmware` directory.

```bash
# Only run this once
cmake -B build --toolchain xmos_cmake_toolchain/xs3a.cmake
cd build/

# Run this every time (in the build directory)
make create_upgrade_img_example_ffva_int_fixed_delay
```

  

**Step 4:** flash the firmware.

Unplug the VoiceKit, change the switch on the VoiceKit to select the `XU316` and plug it back in.

Ensure that you can see it with `dfu-util` (install if needed):

```bash
sudo dfu-util --list
```

  

Now flash the firmware located in the `voice-kit-xmos-firmware/build` directory:

```bash
sudo dfu-util -e -a 1 -D example_ffva_int_fixed_delay_upgrade.bin
```

  

Unplug the VoiceKit, switch the jumper back to the ESP32, and plug it back in.

Profit.
