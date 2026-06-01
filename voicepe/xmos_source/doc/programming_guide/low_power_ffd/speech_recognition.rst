
.. _sln_voice_low_power_ffd_speech_recognition:

##################
Speech Recognition
##################

License
=======

The Sensory TrulyHandsFree™ (THF) speech recognition library is `Copyright (C) 1995-2022 Sensory Inc., All Rights Reserved`.

Sensory THF software requires a commercial license granted by `Sensory Inc <https://www.sensory.com/>`_.
This software ships with an expiring development license. It will suspend recognition after 11.4 hours
or 107 recognition events.

Overview
========

The Sensory THF speech recognition engine runs proprietary models to identify keywords in an audio stream. Models can be generated using `VoiceHub <https://voicehub.sensory.com/>`__.

Two models are provided for the purpose of Low Power FFD. The small wake word model running on tile 1
is approximately 67KB. The command model running on tile 0 is approximately 289KB. On tile 1, the
Sensory runtime and application supporting code consumes approximately 239KB of SRAM. On tile 0, the
Sensory runtime and application supporting code consumes approximately 210KB of SRAM.

With the command model in flash, the Sensory engine requires a core frequency of at least 450 MHz to
keep up with real time. Additionally, the intent engine that is responsible for processing the
commands must be on the same tile as the flash.

To run with a different model, see the ``Set Sensory model variables`` section of the ``low_power_ffd.cmake`` file. There several variables are set pointing to files that are part of the VoiceHub generated model download. Change these variables to point to the files you downloaded. This can be done for both the wakeword and command models.  The command model "net.bin" file, because it is placed in flash memory, must first be nibble swapped.  A utility is provided that is part of the host applications built during install.  Run that application with the following command:

.. code-block:: console

  nibble_swap <your-model-prod-net.bin> <your-model-prod-net.bin.nibble_swapped>

Make sure run the following commands to rebuild and re-flash the data partition:

.. code-block:: console

    make clean
    make flash_app_example_low_power_ffd -j

You may also wish to modify the command ID-to-string lookup table which is located in the ``src/intent_engine/intent_engine_io.c`` source file.

.. TODO: Check if the line below can be removed or re-added
.. To replace the Sensory engine with a different engine, refer to the ASR documentation on :ref:`sln_voice_asr_programming_guide`

Wake Word Dictionary
====================

.. list-table:: English Language Wake Words
   :widths: 50 100
   :header-rows: 1
   :align: left

   * - Return code (decimal)
     - Utterance
   * - 1
     - Hello XMOS

Command Dictionary
==================

.. list-table:: English Language Commands
   :widths: 50 100
   :header-rows: 1
   :align: left

   * - Return code (decimal)
     - Utterance
   * - 1
     - Switch on the TV
   * - 2
     - Channel up
   * - 3
     - Channel down
   * - 4
     - Volume up
   * - 5
     - Volume down
   * - 6
     - Switch off the TV
   * - 7
     - Switch on the lights
   * - 8
     - Brightness up
   * - 9
     - Brightness down
   * - 10
     - Switch off the lights
   * - 11
     - Switch on the fan
   * - 12
     - Speed up the fan
   * - 13
     - Slow down the fan
   * - 14
     - Set higher temperature
   * - 15
     - Set lower temperature
   * - 16
     - Switch off the fan

|newpage|

Application Integration
=======================

In depth information on out of the box integration can be found here: :ref:`sln_voice_low_power_ffd_host_integration`

|newpage|
