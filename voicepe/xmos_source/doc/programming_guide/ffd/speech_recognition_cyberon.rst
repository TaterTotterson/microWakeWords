.. include:: <isonum.txt>
.. include:: ../../substitutions.rst

.. _sln_voice_ffd_speech_recognition_cyberon:

############################
Speech Recognition - Cyberon
############################

License
=======

Cyberon DSpotter™ software requires a commercial license granted by `Cyberon Corporation <https://www.cyberon.com.tw/>`_.
This software ships with an expiring development license. It will suspend recognition after 100 recognition events.

Production versions of the DSpotter™ library are unrestricted when running on a specially licensed XMOS device. Please contact Cyberon or XMOS sales for further information.

Overview
========

The Cyberon DSpotter™ speech recognition engine runs proprietary models to identify keywords in an audio stream.

One model for US English is provided. For any technical questions or additional models please contact Cyberon.

.. TODO: Check if the line below can be removed or re-added
.. To replace the Cyberon engine with a different engine, refer to the ASR documentation on :ref:`sln_voice_asr_programming_guide`

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
   * - Hello Cyberon
     - keyword
     - 1
   * - Switch on the TV
     - command
     - 2
   * - Switch off the TV
     - command
     - 3
   * - Channel up
     - command
     - 4
   * - Channel down
     - command
     - 5
   * - Volume up
     - command
     - 6
   * - Volume down
     - command
     - 7
   * - Switch on the lights
     - command
     - 8
   * - Switch off the lights
     - command
     - 9
   * - Brightness up
     - command
     - 10
   * - Brightness down
     - command
     - 11
   * - Switch on the fan
     - command
     - 12
   * - Switch off the fan
     - command
     - 13
   * - Speed up the fan
     - command
     - 14
   * - Slow down the fan
     - command
     - 15
   * - Set higher temperature
     - command
     - 16
   * - Set lower temperature
     - command
     - 17

|newpage|

Application Integration
=======================

In depth information on out of the box integration can be found here: :ref:`sln_voice_ffd_host_integration`

|newpage|
