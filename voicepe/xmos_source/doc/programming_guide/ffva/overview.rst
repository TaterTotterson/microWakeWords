
********
Overview
********

This is the XCORE-VOICE far-field voice assistant example design.

This application can be used out of the box as a voice processor solution, or expanded to run local wakeword engines.

This application features a full duplex acoustic echo cancellation stage, which can be provided reference audio via |I2S| or USB audio.  An audio output ASR stream is also available via |I2S| or USB audio.

By default, there are two audio integration options. The INT (Integrated) configuration uses |I2S| for reference and output audio streams. The UA (USB Accessory) configuration uses USB UAC 2.0 for reference and output audio streams.