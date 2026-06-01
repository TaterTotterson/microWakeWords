
.. _sln_voice_ffd_speech_recognition_sensory:

############################
Speech Recognition - Sensory
############################

License
=======

The Sensory TrulyHandsFree™ (THF) speech recognition library is `Copyright (C) 1995-2022 Sensory Inc., All Rights Reserved`.

Sensory THF software requires a commercial license granted by `Sensory Inc <https://www.sensory.com/>`_.
This software ships with an expiring development license. It will suspend recognition after 11.4 hours
or 107 recognition events.

Overview
========

The Sensory THF speech recognition engine runs proprietary models to identify keywords in an audio stream.  Models can be generated using `VoiceHub <https://voicehub.sensory.com/>`__.

Two models are provided - one in US English and one in Mainland Mandarin. The US English model is used by default.  To modify the software to use the Mandarin model, see the comment at the top of the ``ffd_sensory.cmake`` file. Make sure run the following commands to rebuild and re-flash the data partition:

.. code-block:: console

    make clean
    make flash_app_example_ffd_sensory -j

.. TODO: Check if the line below can be removed or re-added
.. To replace the Sensory engine with a different engine, refer to the ASR documentation on :ref:`sln_voice_asr_programming_guide`

Dictionary command table
========================

.. list-table:: English Language Demo
   :widths: 50 50 50
   :header-rows: 1
   :align: left

   * - Utterances
     - Type
     - Return code (decimal)
   * - Hello XMOS
     - keyword
     - 1
   * - Switch on the TV
     - command
     - 3
   * - Switch off the TV
     - command
     - 4
   * - Channel up
     - command
     - 5
   * - Channel down
     - command
     - 6
   * - Volume up
     - command
     - 7
   * - Volume down
     - command
     - 8
   * - Switch on the lights
     - command
     - 9
   * - Switch off the lights
     - command
     - 10
   * - Brightness up
     - command
     - 11
   * - Brightness down
     - command
     - 12
   * - Switch on the fan
     - command
     - 13
   * - Switch off the fan
     - command
     - 14
   * - Speed up the fan
     - command
     - 15
   * - Slow down the fan
     - command
     - 16
   * - Set higher temperature
     - command
     - 17
   * - Set lower temperature
     - command
     - 18

|newpage|

Application Integration
=======================

In depth information on out of the box integration can be found here: :ref:`sln_voice_ffd_host_integration`

|newpage|
