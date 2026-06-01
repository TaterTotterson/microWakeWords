
.. _sln_voice_low_power_ffd_host_integration:

################
Host Integration
################

Overview
========

This section describes the connections that would need to be made to an external host for plug and
play integration with existing devices.

When an intent is found, the XCORE device will check if the host is awake, by checking the Host
Status GPIO pin. If the host is awake the intent code will be transmitted over |I2C| and/or UART.

If the host is not awake, the XCORE device will trigger a transition of the Wakeup GPIO pin. This
can be configured to be a rising or falling edge. The XCORE device will then wait for a fixed
period of time, set at compile time, before transmitting the intent over the |I2C| and/or UART
interface. This behavior can be changed as desired by modifying the intent handling code.

.. figure:: diagrams/low_power_ffd_host_integration_diagram.drawio.png
   :align: center
   :scale: 80 %
   :alt: low power FFD host integration diagram

|newpage|

UART
^^^^
.. list-table:: UART Connections
   :widths: 50 50
   :header-rows: 1
   :align: left

   * - Low Power FFD Connection
     - Host Connection
   * - J4:24
     - UART RX
   * - J4:20
     - GND

|I2C|
^^^^^
.. list-table:: |I2C| Connections
   :widths: 50 50
   :header-rows: 1
   :align: left

   * - Low Power FFD Connection
     - Host Connection
   * - J4:3
     - SDA
   * - J4:5
     - SCL
   * - J4:9
     - GND

GPIO
^^^^
.. list-table:: GPIO Connections
   :widths: 50 50
   :header-rows: 1
   :align: left

   * - Low Power FFD Connection
     - Host Connection
   * - J4:19
     - Wake up input
   * - J4:21
     - Host Status output
