
********
Overview
********

This is the XCORE-VOICE automated speech recognition (ASR) porting example design.  This example can be used by 3rd-party ASR developers and ISVs to port their ASR library to xcore.ai.  

The example reads a 1 channel, 16-bit, 16kHz wav file, slices it up into bricks, and calls the ASR library with each brick.  The default brick length is 240 samples but this is configurable.  ASR ports that implement the public API defined in ``modules/asr/asr.h`` can easily be added to current and future XCORE-VOICE example designs that support speech recognition.

An oversimplified ASR port example is provided.  This ASR port recognizes the "Hello XMOS" keyword if any acoustic activity is observed in 75 consecutive bricks.

