.. _sln_voice_ffva_software_desc_overview:

********
Overview
********

There are two main build configurations for this application.

.. list-table:: FFVA INT Fixed Delay Resources
   :widths: 30 10 30
   :header-rows: 1
   :align: left

   * - Resource
     - Tile 0
     - Tile 1
   * - Unused CPU Time (600 MHz)
     - 98%
     - 75%
   * - Total Memory Free
     - 166k
     - 82k
   * - Runtime Heap Memory Free
     - 75k
     - 82k

.. list-table:: FFVA UA ADEC Resources
   :widths: 30 10 30
   :header-rows: 1
   :align: left

   * - Resource
     - Tile 0
     - Tile 1
   * - Unused CPU Time (600 MHz)
     - 83%
     - 45%
   * - Total Memory Free
     - 123k
     - 58k
   * - Runtime Heap Memory Free
     - 54k
     - 83k

The description of the software is split up by folder:

.. list-table:: FFVA Software Description
   :widths: 40 120
   :header-rows: 1
   :align: left

   * - Folder
     - Description
   * - :ref:`sln_voice_ffva_audio_pipeline`
     - Preconfigured audio pipelines
   * - :ref:`sln_voice_ffva_bsp_config`
     - Board support configuration setting up software based IO peripherals
   * - :ref:`sln_voice_ffva_filesystem_support`
     - Filesystem contents for application
   * - :ref:`sln_voice_ffva_src`
     - Main application