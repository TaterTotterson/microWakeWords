
.. _sln_voice_memory_cpu:

###########################
Memory and CPU Requirements
###########################

******
Memory
******

The table below lists the approximate memory requirements for the larger software components.  All memory use estimates in the table below are based on the default configuration for the feature.  Alternate configurations will require more or less memory.  The estimates are provided as guideline to assist application developers judge the memory cost of extending the application or benefit of removing an existing feature.  It can be assumed that the memory requirement of components not listed in the table below are under 5 kB.

.. list-table:: Memory Requirements
    :widths: 50 50
    :header-rows: 1
    :align: left

    * - Component
      - Memory Use (kB)
    * - Stereo Adaptive Echo Canceler (AEC)
      - 275
    * - Sensory Speech Recognition Engine
      - 180
    * - Cyberon Speech Recognition Engine
      - 125
    * - Interference Canceler (IC) + Voice To Noise Ratio Estimator (VNR)
      - 130
    * - USB
      - 20
    * - Noise Suppressor (NS)
      - 15
    * - Adaptive Gain Control (AGC)
      - 11

***
CPU
***

The table below lists the approximate CPU requirements in MIPS for the larger software components.  All CPU use estimates in the table below are based on the default configuration for the feature.  Alternate configurations will require more or less MIPS.  The estimates are provided as guideline to assist application developers judge the MIP cost of extending the application or benefits of removing an existing feature.  It can be assumed that the memory requirement of components not listed in the table below are under 1%.

The following formula was used to convert CPU% to MIPS:

MIPS = (CPU% / 100%) * (600 MHz / 5 cores)

.. _table-CPU-sln-voice:

.. list-table:: CPU Requirements  (@ 600 MHz)
    :widths: 50 50 50
    :header-rows: 1
    :align: left

    * - Component
      - CPU Use (%)
      - MIPS Use
    * - USB XUD
      - 100
      - 120
    * - |I2S| (slave mode)
      - 80
      - 96
    * - Stereo Adaptive Echo Canceler (AEC)
      - 80
      - 96
    * - Sensory Speech Recognition Engine
      - 80
      - 96
    * - Cyberon Speech Recognition Engine
      - 72
      - 87
    * - Interference Canceler (IC) + Voice To Noise Ratio Estimator (VNR)
      - 25
      - 30
    * - Noise Suppressor (NS)
      - 10
      - 12
    * - Adaptive Gain Control (AGC)
      - 5
      - 6

