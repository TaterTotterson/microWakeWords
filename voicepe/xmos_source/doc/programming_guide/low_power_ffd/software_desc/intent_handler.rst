.. _sln_voice_low_power_ffd_intent_handler:

##################
src/intent_handler
##################

This folder contains ASR output handling modules for the Low Power FFD application.

.. list-table:: FFD Intent handler
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - intent_handler.c
     - contains the implementation of default intent handling code
   * - intent_handler.h
     - header for intent handler code


Major Components
================

The intent handling module provides the application with one API function:

.. code-block:: c
    :caption: Intent Handler API (intent_handler.h)

    int32_t intent_handler_create(uint32_t priority, void *args);

If replacing the existing handler code, this is the only function that is required to be populated.


intent_handler_create
^^^^^^^^^^^^^^^^^^^^^

This function has the role of creating the keyword handling task for the ASR engine. In the case of
the Sensory model, the application provides a FreeRTOS Queue object. This handler is on the same
tile as the Sensory engine, tile 0.

The call to intent_handler_create() will create one thread on tile 0. This thread will receive ID
packets from the ASR engine over a FreeRTOS Queue object and output over various IO interfaces based
on configuration.
