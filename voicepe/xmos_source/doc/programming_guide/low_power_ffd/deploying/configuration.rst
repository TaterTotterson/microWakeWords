
.. _sln_voice_low_power_ffd_configuration:

Configuring the Firmware
========================

The default application performs as described in the :ref:`sln_voice_low_power_ffd_overview`. There
are numerous compile time options that can be added to change the example design without requiring
code changes. To change the options explained in the table below, add the desired configuration
variables to the APP_COMPILE_DEFINITIONS CMake variable located in the example's CMake file
`here <https://github.com/xmos/sln_voice/blob/develop/examples/low_power_ffd/low_power_ffd.cmake>`_.

If options are changed, the application firmware must be rebuilt.

.. list-table:: Low Power FFD Compile Options
   :widths: 90 85 20
   :header-rows: 1
   :align: left

   * - Compile Option
     - Description
     - Default Value
   * - appconfINTENT_RESET_DELAY_MS
     - Sets the period after the wake word phrase or subsequent command/wake word phrase has been heard for a valid command phrase
     - 4000
   * - appconfINTENT_UART_OUTPUT_ENABLED
     - Enables/disables the UART intent message
     - 1
   * - appconfINTENT_I2C_OUTPUT_ENABLED
     - Enables/disables the |I2C| intent message
     - 1
   * - appconfUART_BAUD_RATE
     - Sets the baud rate for the UART tx intent interface
     - 9600
   * - appconfINTENT_I2C_OUTPUT_DEVICE_ADDR
     - Sets the |I2C| slave address to transmit the intent to
     - 0x01
   * - appconfINTENT_TRANSPORT_DELAY_MS
     - Sets the delay between host wake up requested and |I2C| and UART keyword code transmission
     - 50
   * - appconfINTENT_QUEUE_LEN
     - Sets the maximum number of detected intents to hold while waiting for the host to wake up
     - 10
   * - appconfINTENT_WAKEUP_EDGE_TYPE
     - Sets the host wake up pin GPIO edge type. 0 for rising edge, 1 for falling edge
     - 0
   * - appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR
     - Enables/disables the IC and VNR
     - 0
   * - appconfAUDIO_PIPELINE_SKIP_NS
     - Enables/disables the NS
     - 0
   * - appconfAUDIO_PIPELINE_SKIP_AGC
     - Enables/disables the AGC
     - 0

|newpage|
