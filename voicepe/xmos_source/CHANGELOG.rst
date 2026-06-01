XCORE-VOICE change log
======================

2.2.0
-----

  * CHANGED: Updated submodule fwk_io to version 3.3.0 from 3.1.0.
  * CHANGED: Updated submodule fwk_core to version 1.0.2 from 1.0.0.
  * CHANGED: Updated submodule fwk_rtos to version 3.0.5 from 3.0.3.
  * CHANGED: Updated submodule fwk_voice to version 0.7.0 from 0.6.0.
  * CHANGED: Updated submodule lib_qspi_fast_read to version 1.0.2 from 1.0.1.
  * CHANGED: Updated submodule lib_src to version 2.4.0 from 2.2.0.
  * CHANGED: Updated submodule xscope_fileio to version 1.1.2 from 1.1.1.
  * ADDED: Support for Microsoft OS 2.0 Descriptors for FFVA and ASRC examples
    so the DFU interface automatically shows up as a WinUSB interface on
    Windows.
  * ADDED: FFD example with Cyberon speech recognition engine and model
    (DSpotter v2.2.18.0).

2.1.0
-----

  * ADDED: Mic aggregator app that bridges between 16 mics and TDM16 slave or
    USB Audio
  * ADDED: Asynchronous Sampling Rate Converter (ASRC) example application
  * ADDED: lib_xua 3.5.1 as submodule to support Mic aggregator app
  * CHANGED: Updated submodule fwk_io on to version 3.1.0 from version 3.0.1 to
    add support for TDM16 slave tx and 16ch mic_array

2.0.0
-----

  * ADDED: All speech recognition example deigns now use Sensory TrulyHandsfree
    speech recognition library.
  * ADDED: Low-power far-field dictionary (low-power FFD) example design using
    wakeword to exit standby mode.
  * ADDED: Mandarin model to far-field dictionary (low-power FFD) example design
    demonstration.
  * ADDED: Support for fast flash library which increases flash reading
    throughput by as much as 70%.
  * MOVED: Audio pipelines relocated to modules to allow easier re-use.
  * FIXED: Example designs now indicate when evaluation period has expired.
  * FIXED: Numerous minor bug fixes.

1.0.0
-----

  * ADDED: Improved documentation
  * ADDED: Speech recognition porting example design
  * FIXED: USB PIDs changed to 0x4000 for FFVA and 0x4001 for FFD
  * FIXED: Speech recognition model no longer in the filesystem
  * FIXED: Several Linux USB audio bugs
  * REMOVED: audiomux example design

0.21.0
------

  * ADDED: Improved Interference Cancellation using Voice-to-Noise Ratio
    estimator
  * ADDED: Low-power mode to far-field dictionary example design
  * ADDED: USB DFU to far-field voice assistant example design
  * ADDED: Optional AEC fixed-delay to far-field voice assistant example design

0.20.0
------

  * ADDED: Audio response playback
  * ADDED: Wanson 20220923 model update
  * ADDED: Documentation updates
  * FIXED: USB audio device is now recognized on Windows
  * FIXED: Applications moved to examples folder
  * FIXED: Sample rate conversion from 48k fixed

0.12.0
------

  * ADDED: Audio playback to FFD application
  * ADDED: First draft of documentation
  * KNOWN ISSUE: USB audio device is not recognized on Windows  (This does not
    effect FFD)

0.10.0
------

  * ADDED: FFD demo using OLED display

