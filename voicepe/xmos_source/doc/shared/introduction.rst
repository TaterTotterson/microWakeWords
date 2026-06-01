
###################
Product Description 
###################

The XCORE-VOICE Solution consists of example designs and a C-based SDK for the development of audio front-end applications to support far-field voice use cases on the xcore.ai family of chips (XU316). The XCORE-VOICE examples are currently based on FreeRTOS or bare-metal, leveraging the flexibility of the xcore.ai platform and providing designers with a familiar environment to customize and develop products.

XCORE-VOICE example designs include turn-key solutions to enable easier product development for smart home applications such as light switches, thermostats, and home appliances. xcore.ai’s unique architecture providing powerful signal processing and accelerated AI capabilities combined with the XCORE-VOICE framework allows designers to incorporate keyword, event detection, or advanced local dictionary support to create a complete voice interface solution. Bridging designs including PDM microphone to host aggregation are also included showcasing the use of xcore.ai as an interfacing and bridging solution for deployment in existing systems. 

The C SDK is composed of the following components:

- Peripheral IO libraries including; UART, I2C, I2S, SPI, QSPI, PDM microphones, and USB. These libraries support bare-metal and RTOS application development.
- Libraries core to DSP applications, including vectorized math and voice processing DSP.  These libraries support bare-metal and RTOS application development.
- Libraries for speech recognition applications.  These libraries support bare-metal and RTOS application development.
- Libraries that enable multi-core FreeRTOS development on xcore including a wide array of RTOS drivers and middleware.
- Pre-build and validated audio processing pipelines.  
- Code Examples - Examples showing a variety of xcore features based on bare-metal and FreeRTOS programming.
- Documentation - Tutorials, references and API guides.

.. figure:: diagrams/xcore-voice_component_diagram.drawio.png
   :align: center
   :scale: 80 %
   :alt: component diagram

############
Key Features 
############

The XCORE-VOICE Solution takes advantage of the flexible software-defined xcore-ai architecture to support numerous far-field voice use cases through the available example designs and the ability to construct user-defined audio pipeline from the SW components and libraries in the C-based SDK.

These include: 

**Voice Processing components**

- Two PDM microphone interfaces 
- Digital signal processing pipeline 
- Full duplex, stereo, Acoustic Echo Cancellation (AEC) 
- Reference audio via |I2S| with automatic bulk delay insertion 
- Point noise suppression via interference canceller 
- Switchable stationary noise suppressor 
- Programmable Automatic Gain Control (AGC) 
- Flexible audio output routing and filtering 
- Support for Sensory, Cyberon or other 3rd party Automatic Speech Recognition (ASR) software

**Device Interface components**

- Full speed USB2.0 compliant device supporting USB Audio Class (UAC) 2.0 
- Flexible Peripheral Interfaces 
- Programmable digital general-purpose inputs and outputs 

**Example Designs utilizing above components**

- Far-Field Voice Local Command 
- Low Power Far-Field Voice Local Command 
- Far-Field Voice Assistance 

**Firmware Management**

- Boot from QSPI Flash 
- Default firmware image for power-on operation 
- Option to boot from a local host processor via SPI 
- Device Firmware Update (DFU) via USB or other transport

**Power Consumption**

- FFD/FFVA: 300-350mW (Typical)
- Low Power FFD: 110mW (Full-Power), 54mW (Low-Power), <50mW possible with Sensory's LPSD under certain conditions.

######################
Obtaining the Hardware
######################

The XK-VOICE-L71 DevKit and Hardware Manual can be obtained from the |HARDWARE_URL| product information page. 

The XK-VOICE-L71 is based on the: `XU316-1024-QF60A <https://www.xmos.ai/file/xu316-1024-qf60b-xcore_ai-datasheet?version=latest>`_

The XCORE-AI-EXPLORER DevKit and Hardware Manual used in the :ref:`Microphone Aggregation <sln_voice_mic_aggregator_programming_guide>` example can be obtained from the |HARDWARE_URL| product information page. 

Learn more about the `The XMOS XS3 Architecture <https://www.xmos.ai/download/The-XMOS-XS3-Architecture.pdf>`_

######################
Obtaining the Software
######################

*****************
Development Tools
*****************

It is recommended that you download and install the latest release of the `XTC Tools <https://www.xmos.com/software/tools/>`__.  XTC Tools 15.2.1 or newer are required. If you already have the XTC Toolchain installed, you can check the version with the following command:

.. code-block:: console

    xcc --version

**************************
Application Demonstrations 
**************************

If you only want to run the example designs, pre-built firmware and other software can be downloaded from the |SOFTWARE_URL| product information page.  

***********
Source Code
***********

If you wish to modify the example designs, a zip archive of all source code can be downloaded from the |SOFTWARE_URL| product information page.  

See the :ref:`Programming Guide <sln_voice_programming_guide>` for information on:

- Prerequisites
- Instructions for building, running, and debugging the example designs
- Details on the software design and source code

Cloning the Repository
======================

Alternatively, the source code can be obtained by cloning the public GitHub repository.  

.. note::

  Cloning requires a `GitHub <https://github.com>`_ account configured with `SSH key authentication <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh>`_.  

Run the following `git` command to clone the repository and all submodules:

.. code-block:: console

  git clone --recurse-submodules git@github.com:xmos/sln_voice.git

If you have previously cloned the repository or downloaded a zip file of source code, the following commands can be used to update and fetch the submodules:

.. code-block:: console

    git pull
    git submodule update --init --recursive
