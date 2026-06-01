.. _sln_voice_low_power_ffd_software_desc_overview:

********
Overview
********

The approximate resource utilizations for Low Power FFD are shown in the table below.

.. list-table:: Low Power FFD Resources
   :widths: 30 10 30
   :header-rows: 1
   :align: left

   * - Resource
     - Tile 0
     - Tile 1
   * - Unused CPU Time (600MHz | 200MHz)
     - 50%
     - 10%
   * - Total Memory Free
     - 19.1k
     - 5.3k
   * - Runtime Heap Memory Free
     - 219k
     - 12.4k

The estimated (core) power usage for Low Power FFD are shown in the table below. Additional power
savings may be possible using Sensory's Low Power Sound Detect (LPSD) option which approaches sub-50mW
operation in Low Power mode. These measurements will vary based on component tolerances and any user
added code and/or user added compile options.

.. list-table:: Low Power FFD Power Usage
   :widths: 30 30
   :header-rows: 1
   :align: left

   * - Power State
     - Core Power (mW)
   * - Low Power
     - 54
   * - Full Power
     - 110

The description of the software is split up by folder:

.. list-table:: Low Power FFD Software Description
   :widths: 40 120
   :header-rows: 1
   :align: left

   * - Folder
     - Description

   * - :ref:`sln_voice_low_power_ffd_bsp_config`
     - Board support configuration setting up software based IO peripherals
   * - :ref:`sln_voice_low_power_ffd_filesystem_support`
     - Filesystem contents for application
   * - :ref:`sln_voice_low_power_ffd_model`
     - Wake word and command model files
   * - :ref:`sln_voice_low_power_ffd_src`
     - Main application
   * - :ref:`sln_voice_low_power_ffd_gpio_ctrl`
     - GPIO and LED related functions
   * - :ref:`sln_voice_low_power_ffd_intent_engine`
     - Intent engine integration
   * - :ref:`sln_voice_low_power_ffd_intent_handler`
     - Intent engine output integration
   * - :ref:`sln_voice_low_power_ffd_power`
     - Low power control logic
   * - :ref:`sln_voice_low_power_ffd_wakeword`
     - Wake word engine integration
