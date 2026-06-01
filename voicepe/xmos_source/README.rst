*******************************************
XCORE:registered:-VOICE Solution Repository
*******************************************

The XCORE-VOICE Solution consists of example designs and a C-based SDK for the development of audio front-end applications to support far-field voice use cases on the xcore.ai family of chips (XU316). Most of the XCORE-VOICE designs are based on FreeRTOS, leveraging the flexibility of the xcore.ai platform and providing designers with a familiar environment to customize and develop products. The only exception is the Microphone Aggregation example which runs on bare-metal; this architecture was selected as it suits high IO bandwidth and low latency bridging type applications.

XCORE-VOICE example designs provide turn-key solutions to enable easier product development for smart home applications such as light switches, thermostats, and home appliances. xcore.ai’s unique architecture providing powerful signal processing and accelerated AI capabilities combined with the XCORE-VOICE framework allows designers to incorporate keyword, event detection, or advanced local dictionary support to create a complete voice interface solution.

Obtaining the Source Code
*************************

If you are interested in compiling and/or modifying the source code, you can download it from GitHub. As some dependent components are included as git submodules, the following command must be used for cloning this repository:

::

    git clone --recurse-submodules git@github.com:xmos/sln_voice.git

If you have previously cloned the repository or downloaded a zip file of source code, the following commands can be used to update and fetch the submodules:

::

    git pull
    git submodule update --init --recursive

Documentation
*************

See the READMEs for the early example applications:

  * `ASRC Demo <https://github.com/xmos/sln_voice/blob/develop/examples/asrc_demo/README.rst>`_: an example design for an asynchronous sampling rate converter (ASRC)

  * `FFD <https://github.com/xmos/sln_voice/blob/develop/examples/ffd/README.rst>`_: two example designs for a far-field voice local control, each example contains a boot image and data partition binary

  * `FFVA <https://github.com/xmos/sln_voice/blob/develop/examples/ffva/README.rst>`_: two example designs for a far-field voice assistant, each example contains a boot image and data partition binary

  * `Low Power FFD <https://github.com/xmos/sln_voice/blob/develop/examples/low_power_ffd/README.rst>`_: an example design for a low-power far-field voice local control, the example contains a boot image and data partition binary

  * `Microphone Aggregator <https://github.com/xmos/sln_voice/blob/develop/examples/mic_aggregator/README.rst>`_: two example designs bridging 16 PDM microphones to either TDM16 slave or USB Audio

Getting Help
************

A `Github issue <https://github.com/xmos/sln_voice/issues/new/choose>`_ should be the primary method of getting in touch with the XMOS development team.

License
*******

This Software is subject to the terms of the `XMOS Public Licence: Version 1 <https://github.com/xmos/sln_voice/blob/develop/LICENSE.rst>`_.

Additional third party copyrighted code is included under the following licenses:

The Sensory TrulyHandsfree™ speech recognition library is *Copyright (C) 1995-2022 Sensory Inc.* and is provided as an expiring development license. Commercial licensing is granted by `Sensory Inc <https://www.sensory.com/>`_.

The Cyberon DSPotter™ speech recognition library is provided as an expiring development license. Commercial licensing is granted by `Cyberon Corporation <https://www.cyberon.com.tw/>`_ and is subject to deployment only on part number XU316-1024-QF60B-C24-CY.
