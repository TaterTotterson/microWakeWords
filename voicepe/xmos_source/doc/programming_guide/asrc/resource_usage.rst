
**************
Resource Usage
**************

Memory
======

Out of the 524288 bytes of memory available per tile, this application uses approximately 262000 bytes of memory on Tile 0
and 208000 bytes of memory on Tile 1.


Chanends
========

This application uses 19 chanends on the USB tile (tile 0) and 11 chanends on the |I2S| tile (tile 1)

The chanend use for both tiles is described in the :ref:`table-tile0-chanend-label` and :ref:`table-tile1-chanend-label` tables.

Tile 0
------

.. _table-tile0-chanend-label:

.. list-table:: Tile 0 chanend usage
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Resource
     - Chanends used
   * - RTOS scheduler
     - 5 (one per bare-metal core dedicated to RTOS)
   * - RTOS USB driver
     - 10 (2 per endpoint, per direction. 2 for SOF input)
   * - :ref:`intertile-context-label`
     - 3
   * - xscope
     - 1


Tile 1
------

.. _table-tile1-chanend-label:

.. list-table:: Tile 1 chanend usage
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Resource
     - Chanends used
   * - RTOS scheduler
     - 5 (one per bare-metal core dedicated to RTOS)
   * - RTOS |I2S| driver
     - 2
   * - :ref:`intertile-context-label`
     - 3
   * - xscope
     - 1


.. _intertile-context-label:

Intertile contexts
------------------

The application uses 3 intertile contexts for cross tile communication.

    * A dedicated intertile context for sending ASRC output data from the I2S tile to the USB tile.
    * A dedicated intertile context for sending ASRC output data from the USB tile to the I2S tile.
    * The intertile context for all other cross tile communication.


CPU
===

Profiling the CPU usage for this application using an RTOS friendly profiling tool is still TBD.
However, profiling some application tasks has taken place. These numbers along with some already existing profiling numbers for the drivers are listed in the :ref:`table-mips-tile0-label` and :ref:`table-mips-tile1-label` tables.
Each tile has 5 bare-metal cores being used for running RTOS tasks so each core has a fixed bandwidth of 120 MHz available.

Tile 0
------

.. _table-mips-tile0-label:

.. list-table:: Tile 0 tasks MIPS
   :widths: 50 50
   :header-rows: 1
   :align: left

   * - RTOS Task
     - MIPS
   * - XUD
     - 120 (from :ref:`table-CPU-sln-voice`)
   * - ASRC in the USB -> ASRC -> |I2S| path for the worst case of 48 kHz to 192 kHz upsampling
     - 85
   * - usb_task
     - 24
   * - i2s_to_usb_intertile
     - 14


Tile 1
------

.. _table-mips-tile1-label:

.. list-table:: Tile 1 tasks MIPS
   :widths: 50 50
   :header-rows: 1
   :align: left

   * - RTOS Task
     - MIPS
   * - |I2S| Slave
     - 96 (from :ref:`table-CPU-sln-voice`)
   * - ASRC in the |I2S| -> ASRC -> USB path for the worst case of 192 kHz to 48 kHz downsampling
     - 75
   * - usb_to_i2s_intertile
     - 0.7
   * - rate_server
     - 19
