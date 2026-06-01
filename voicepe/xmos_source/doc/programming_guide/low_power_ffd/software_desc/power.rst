.. _sln_voice_low_power_ffd_power:

#########
src/power
#########

This folder contains the low power control logic and supporting logic.

.. list-table:: Low Power FFD power
   :widths: 45 55
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - low_power_audio_buffer.c
     - Implementation of an audio sample ring buffer. Aids in responsiveness to commands during a transition to full power mode.
   * - low_power_audio_buffer.c
     - Header for the low power audio buffer.
   * - power_control.c
     - Implementation of the power control logic.
   * - power_control.h
     - Header for power control logic.
   * - power_state.c
     - Implementation of Tile 1 power state logic.
   * - power_state.h
     - Header for power state logic.


Major Components
================

The power control module provides the application with the following primary API functions:

.. code-block:: c
    :caption: Power Control API (power_control.h)

    void power_control_task_create(unsigned priority, void *args);
    void power_control_exit_low_power(void);
    power_state_t power_control_state_get(void);
    void power_control_halt(void);
    void power_control_req_low_power(void);
    void power_control_ind_complete(void);

power_control_task_create
^^^^^^^^^^^^^^^^^^^^^^^^^

Creates and starts the power control task. To be called by each tile.

power_control_exit_low_power
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Applicable only for Tile 1. Begins a transition to full power mode and is intended to be called by
the power_state_set() routine.

power_control_state_get
^^^^^^^^^^^^^^^^^^^^^^^

Applicable only for Tile 1. Gets the current power state.

power_control_halt
^^^^^^^^^^^^^^^^^^

Applicable only for Tile 1. Halts the power control task. This is provided primarily for
end-of-evaluation logic, but severs to terminate the low power logic. When halted, the system
remains in full power mode.

power_control_req_low_power
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Applicable only for Tile 0. Requests a transition to low power mode.

power_control_ind_complete
^^^^^^^^^^^^^^^^^^^^^^^^^^

Applicable only for Tile 0. Indication that the last step for preparing for a low power transition
has completed and allows the power control task to continue with final steps. This is primarily to
ensure the LED indications are up-to-date before driver locks are taken (which include GPIO/LED control).

Power State Components
======================

The power state module provides the application with the following primary API functions:

.. code-block:: c
    :caption: Power State API (power_state.h)

    void power_state_init();
    void power_state_set(power_state_t state);
    uint8_t power_state_timer_expired_get(void);

This module is also responsible for providing the base power state datatype (`power_state_t`) used by
other low power logic.

power_state_init
^^^^^^^^^^^^^^^^

Initializes the power state module. Responsible to initializing the underlying timer that effectively
determines whether a low power request by Tile 0 is accepted or rejected.

power_state_set
^^^^^^^^^^^^^^^

Used by Tile 1's application to signal full power events (such as wake word detection or other
application-specific events). Used by Tile 1's power control logic to signal low power only after
Tile 0 has requested low power mode and the local timer has expired.

power_state_timer_expired_get
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Used by the Tile 1's power control logic to determine whether to accept or reject a low power request by Tile 0.
