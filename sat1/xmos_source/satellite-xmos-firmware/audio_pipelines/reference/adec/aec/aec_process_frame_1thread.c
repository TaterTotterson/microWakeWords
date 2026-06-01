// Copyright 2021-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <stdio.h>
#include <string.h>
#include "aec_defines.h"
#include "aec_api.h"

/* This is an example of processing one frame of data through the AEC pipeline stage. The example runs on 1 thread and
 * can be compiled for both bare metal and x86.
 */
static unsigned X_energy_recalc_bin = 0;
void aec_process_frame_1thread(
        aec_state_t *main_state,
        aec_state_t *shadow_state,
        int32_t (*output_main)[AEC_FRAME_ADVANCE],
        int32_t (*output_shadow)[AEC_FRAME_ADVANCE],
        const int32_t (*y_data)[AEC_FRAME_ADVANCE],
        const int32_t (*x_data)[AEC_FRAME_ADVANCE])
{
    // Read number of mic and reference channels. These are specified as part of the configuration when aec_init() is called.
    int num_y_channels = main_state->shared_state->num_y_channels; //Number of mic channels
    int num_x_channels = main_state->shared_state->num_x_channels; //Number of reference channels

    // Set up the input BFP structures main_state->shared_state->y and main_state->shared_state->x to point to the new frame.
    // Initialise some other BFP structures that need to be initialised at the beginning of each frame
    aec_frame_init(main_state, shadow_state, y_data, x_data);

    // Calculate Exponential moving average (EMA) energy of the mic and reference input.
    for(int ch=0; ch<num_y_channels; ch++) {
        aec_calc_time_domain_ema_energy(&main_state->shared_state->y_ema_energy[ch], &main_state->shared_state->y[ch],
                AEC_PROC_FRAME_LENGTH - AEC_FRAME_ADVANCE, AEC_FRAME_ADVANCE, &main_state->shared_state->config_params);
    }
    for(int ch=0; ch<num_x_channels; ch++) {
        aec_calc_time_domain_ema_energy(&main_state->shared_state->x_ema_energy[ch], &main_state->shared_state->x[ch],
                AEC_PROC_FRAME_LENGTH - AEC_FRAME_ADVANCE, AEC_FRAME_ADVANCE, &main_state->shared_state->config_params);
    }

    // Calculate mic input spectrum for all num_y_channels of mic input
    /* The spectrum calculation is done in place. Taking mic input as example, after the aec_forward_fft() call
     * main_state->shared_state->Y[ch].data and main_state->shared_state->y[ch].data point to the same memory address.
     * The spectral representation of the input is used after this function. Time domain input
     * BFP structure main_state->shared_state->y should not be used.
     * main_state->shared_state->Y[ch].data points to AEC_PROC_FRAME_LENGTH/2 + 1 complex 32bit spectrum samples,
     * which represent the spectrum samples from DC to Nyquist frequency.
     * Same is true for reference spectrum samples pointed to by  main_state->shared_state->X[ch].data
     * as well.
     */
    for(int ch=0; ch<num_y_channels; ch++) {
        aec_forward_fft(&main_state->shared_state->Y[ch], &main_state->shared_state->y[ch]);
    }
    // Calculate reference input spectrum for all num_x_channels of reference input
    for(int ch=0; ch<num_x_channels; ch++) {
        aec_forward_fft(&main_state->shared_state->X[ch], &main_state->shared_state->x[ch]);
    }

    // Calculate sum of X energy over X FIFO phases for all num_x_channels reference channels
    /* AEC data structures store a single copy of the X FIFO that is shared between the main and shadow filter.
     * Since main filter phases main_state->num_phases are more than the shadow filter phases shadow_state->num_phases,
     * X FIFO holds main_state->num_phases most recent frames of reference input spectrum, where the frames are ordered
     * from most recent to least recent. For shadow filter operation, out of this shared X FIFO, the first shadow_state->num_phases
     * frames are considered.
     */
    for(int ch=0; ch<num_x_channels; ch++) {
        // Calculate sum of X energy for main filter
        /* BFP struct main_state->X_energy[ch] points to AEC_PROC_FRAME_LENGTH/2 + 1 real 32bit values where value at index n is
         * the nth X sample's energy summed over main_state->num_phases number of frames in the X FIFO.
         */
        aec_calc_X_fifo_energy(main_state, ch, X_energy_recalc_bin);

        // Calculate sum of X energy for shadow filter
        /* BFP struct shadow_state->X_energy[ch] points to AEC_PROC_FRAME_LENGTH/2 + 1 real 32bit values where value at index n is
         * the nth X sample's energy summed over shadow_state->num_phases number of frames in the X FIFO.
         */
        aec_calc_X_fifo_energy(shadow_state, ch, X_energy_recalc_bin);
    }

    // Increment X_energy_recalc_bin to the next sample index.
    /* Passing X_energy_recalc_bin to aec_calc_X_fifo_energy() ensures that energy of sample at index X_energy_recalc_bin
     * is recalculated without the speed optimisations so that quantisation error can be kept in check
     */
    X_energy_recalc_bin += 1;
    if(X_energy_recalc_bin == (AEC_PROC_FRAME_LENGTH/2) + 1) { // Wrap around to 0 on completing one (AEC_PROC_FRAME_LENGTH/2) + 1 samples pass.
        X_energy_recalc_bin = 0;
    }

    // Update X-FIFO and calculate sigma_XX.
    /* Add the current X frame to the X FIFO and remove the oldest X frame from the X FIFO.
     * Also, calculate state->shared_state->sigma_XX. sigma_XX is the EMA of current X frame energy.
     * It is later used to time smooth the X_energy while calculating the normalisation spectrum
     */
    for(int ch=0; ch<num_x_channels; ch++) {
        aec_update_X_fifo_and_calc_sigmaXX(main_state, ch);
    }

    // Copy state->shared_state->X_fifo BFP struct to main_state->X_fifo_1d and shadow_state->X_fifo_1d BFP structs
    /* The updated state->shared_state->X_FIFO BFP structures are copied to an alternate set of BFP structs present in the
     * main and shadow filter state structure, that are used to efficiently access the X FIFO in the Error computation and filter
     * update steps.
     */
    aec_update_X_fifo_1d(main_state);
    aec_update_X_fifo_1d(shadow_state);

    // Calculate error spectrum and estimated mic spectrum for main and shadow adaptive filters
    for(int ch=0; ch<num_y_channels; ch++) {
        // main_state->Error[ch] and main_state->Y_hat[ch] are updated
        aec_calc_Error_and_Y_hat(main_state, ch);

        // shadow_state->Error[ch] and shadow_state->Y_hat[ch] are updated
        aec_calc_Error_and_Y_hat(shadow_state, ch);
    }

    // Calculate time domain error and time domain estimated mic input from their spectrums calculated in the previous step.
    /* The time domain estimated mic_input (y_hat) is used to calculate the average coherence between y and y_hat in aec_calc_coherence.
     * Only the estimated mic input calculated using the main filter is needed for coherence calculation, so the y_hat calculation is
     * done only for main filter.
     */
    for(int ch=0; ch<num_y_channels; ch++) {
        aec_inverse_fft(&main_state->error[ch], &main_state->Error[ch]);
        aec_inverse_fft(&shadow_state->error[ch], &shadow_state->Error[ch]);
        aec_inverse_fft(&main_state->y_hat[ch], &main_state->Y_hat[ch]);
    }

    // Calculate average coherence and average slow moving coherence between mic and estimated mic time domain signals
    for(int ch=0; ch<num_y_channels; ch++) {
        // main_state->shared_state->coh_mu_state[ch].coh and main_state->shared_state->coh_mu_state[ch].coh_slow are updated
        aec_calc_coherence(main_state, ch);
    }

    // Calculate AEC filter time domain output. This is the output sent to downstream pipeline stages
    for(int ch=0; ch<num_y_channels; ch++) {
        aec_calc_output(main_state, &output_main[ch], ch);
        /* Application can choose to not generate AEC shadow filter output by passing NULL as output_shadow argument.
         * Note that aec_calc_output() will still need to be called since this function also windows the error signal
         * which is needed for subsequent processing of the shadow filter even when output is not generated.
         */
        if(output_shadow != NULL) {
            aec_calc_output(shadow_state, &output_shadow[ch], ch);
        }
        else {
            aec_calc_output(shadow_state, NULL, ch);
        }
    }

    // Calculate exponential moving average of main_filter time domain error.
    /* The EMA error energy is used in ERLE calculations which are done only for the main filter,
     * so not calling this function to calculate shadow filter error EMA energy.
     */
    for(int ch=0; ch<num_y_channels; ch++) {
        //create a bfp_s32_t structure to point to output array
        bfp_s32_t temp;
        bfp_s32_init(&temp, &output_main[ch][0], -31, AEC_FRAME_ADVANCE, 1);
        aec_calc_time_domain_ema_energy(&main_state->error_ema_energy[ch], &temp, 0, AEC_FRAME_ADVANCE, &main_state->shared_state->config_params);
    }

    // Convert shadow and main filters error back to frequency domain since subsequent AEC functions will use the error spectrum.
    // The error spectrum is later used to compute T values which are then used while updating the adaptive filter.
    for(int ch=0; ch<num_y_channels; ch++) {
        // main_state->Error[ch] is updated
        aec_forward_fft(&main_state->Error[ch], &main_state->error[ch]);

        // shadow_state->Error[ch] is updated
        aec_forward_fft(&shadow_state->Error[ch], &shadow_state->error[ch]
               );
    }

    // Calculate energies of mic input and error spectrum of main and shadow filters.
    // These energy values are later used in aec_compare_filters_and_calc_mu() to estimate how well the filters are performing.
    for(int ch=0; ch<num_y_channels; ch++) {
        // main_state->overall_Error[ch] is updated
        aec_calc_freq_domain_energy(&main_state->overall_Error[ch], &main_state->Error[ch]);

        // shadow_state->overall_Error[ch] is updated
        aec_calc_freq_domain_energy(&shadow_state->overall_Error[ch], &shadow_state->Error[ch]);

        // main_state->shared_state->overall_Y[ch] is updated
        aec_calc_freq_domain_energy(&main_state->shared_state->overall_Y[ch], &main_state->shared_state->Y[ch]);
    }

    // Compare and update filters. Calculate adaption step_size mu
    /* At this point we're ready to check how well the filters are performing and update them if needed.
     *
     * main_state->shared_state->shadow_filter_params are updated to indicate the current state of filter comparison algorithm.
     * main_state->H_hat, main_state->Error, shadow_state->H_hat, shadow_state->Error are optionally updated depending on the update needed.
     *
     * After the filter comparison and update step, the adaption step size mu is calculated for main and shadow filter.
     * main_state->mu and shadow_state->mu are updated.
     */
    aec_compare_filters_and_calc_mu(
            main_state,
            shadow_state);

    // Calculate smoothed reference FIFO energy that is later used to scale the X FIFO in the filter update step.
    // This calculation is done differently for main and shadow filters, so a flag indicating filter type is specified as one of the input arguments.
    for(int ch=0; ch<num_x_channels; ch++) {
        // main_state->inv_X_energy[ch] is updated.
        aec_calc_normalisation_spectrum(main_state, ch, 0);

        // shadow_state->inv_X_energy[ch] is updated.
        aec_calc_normalisation_spectrum(shadow_state, ch, 1);
    }

    for(int ych=0; ych<num_y_channels; ych++) {
        // Compute T values.
        // T is a function of state->mu, state->Error and state->inv_X_energy.
        for(int xch=0; xch<num_x_channels; xch++) {
            // main_state->T[ch] is updated
            aec_calc_T(main_state, ych, xch);

            // shadow_state->T[ch] is updated
            aec_calc_T(shadow_state, ych, xch);
        }
        // Update filters

        // Update main_state->H_hat
        aec_filter_adapt(main_state, ych);

        // Update shadow_state->H_hat
        aec_filter_adapt(shadow_state, ych);
    }
}
