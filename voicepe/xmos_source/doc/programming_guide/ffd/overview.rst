
.. _sln_voice_ffd_overview:

********
Overview
********

This is the far-field voice local command (FFD) example design. Two examples are provided: both examples include speech recognition and a local dictionary. One example uses the Sensory TrulyHandsfree™ (THF) libraries, and the other one uses the Cyberon DSPotter™ libraries.

When a wakeword phrase is detected followed by a command phrase, the application will output an audio response and a discrete message over |I2C| and UART.

Sensory's THF and Cyberon's DSpotter™ libraries ship with an expiring development license. The Sensory one will suspend recognition after 11.4 hours or 107 recognition events, and the Cyberon one will suspend recognition after 100 recognition events. After the maximum number of recognitions is reached, a device reset is required to resume normal operation. To perform a reset, either power cycle the device or press the SW2 button.

More information on the Sensory speech recognition library can be found here: :ref:`sln_voice_ffd_speech_recognition_sensory`.

More information on the Cyberon speech recognition library can be found here: :ref:`sln_voice_ffd_speech_recognition_cyberon`

