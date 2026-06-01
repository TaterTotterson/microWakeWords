.. _sln_voice_low_power_ffd_model:

#####
model
#####

This folder contains the Sensory wake word and command model files the Low Power FFD application.

.. note::
  Only a subset of the files below are used. See ``low_power_ffd.cmake`` for the files used by the
  application. Also note the nibble-swapped net-file is manually generated, via the ``nibble_swap``
  tool found in ``lib_qspi_fast_read``.

.. list-table:: Low Power FFD model
   :widths: 45 55
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - command-pc62w-6.1.0-op10-prod-net.bin
     - The command model's net-file, in binary-form
   * - command-pc62w-6.1.0-op10-prod-net.bin.nibble_swapped
     - The command model's net-file, in binary-form (nibble swapped, for supporting fast flash reads)
   * - command-pc62w-6.1.0-op10-prod-net.c
     - The command model's net-file, in source form
   * - command-pc62w-6.1.0-op10-prod-search.bin
     - The command model's search-file, in binary form
   * - command-pc62w-6.1.0-op10-prod-search.c
     - The command model's search-file, in source form
   * - command-pc62w-6.1.0-op10-prod-search.h
     - The command model's search header-file
   * - command.snsr
     - The command model's Sensory THF/TNL SDK "snsr" file
   * - wakeword-pc60w-6.1.0-op10-prod-net.bin
     - The wake word model's net-file, in binary-form
   * - wakeword-pc60w-6.1.0-op10-prod-net.c
     - The wake word model's net-file, in source form
   * - wakeword-pc60w-6.1.0-op10-prod-search.bin
     - The wake word model's search-file, in binary form
   * - wakeword-pc60w-6.1.0-op10-prod-search.c
     - The wake word model's search-file, in source form
   * - wakeword-pc60w-6.1.0-op10-prod-search.h
     - The wake word model's search header-file
   * - wakeword.snsr
     - The wake word model's Sensory THF/TNL SDK "snsr" file
