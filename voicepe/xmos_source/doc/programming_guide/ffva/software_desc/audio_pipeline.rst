.. _sln_voice_ffva_audio_pipeline:

###############
Audio Pipelines
###############

This folder contains preconfigured audio pipelines for the FFVA application.

.. list-table:: FFVA Audio Pipelines
   :widths: 30 50
   :header-rows: 1
   :align: left

   * - Filename/Directory
     - Description
   * - api directory
     - include folder for audio pipeline modules
   * - src directory
     - contains preconfigured XMOS DSP audio pipelines
   * - audio_pipeline.cmake
     - cmake for adding audio pipeline targets


Major Components
================

The audio pipeline module provides the application with three API functions:

.. code-block:: c
    :caption: Audio Pipeline API (audio_pipeline.h)

    void audio_pipeline_init(
            void *input_app_data,
            void *output_app_data);

    void audio_pipeline_input(
            void *input_app_data,
            int32_t **input_audio_frames,
            size_t ch_count,
            size_t frame_count);

    int audio_pipeline_output(
            void *output_app_data,
            int32_t **output_audio_frames,
            size_t ch_count,
            size_t frame_count);


audio_pipeline_init
^^^^^^^^^^^^^^^^^^^

This function has the role of creating the audio pipeline task(s) and initializing DSP stages.

audio_pipeline_input
^^^^^^^^^^^^^^^^^^^^

This function is application defined and populates input audio frames used by the audio pipeline. In FFVA, this function is defined in `main.c`.

audio_pipeline_output
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This function is application defined and populates input audio frames used by the audio pipeline. In FFVA, this function is defined in `main.c`.
