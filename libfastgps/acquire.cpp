/*
* Copyright (c) 2008, Morgan Quigley, Pieter Abbeel and Scott Gleason
* ... (License header) ...
*
* This file has been modified to support parallel, thread-safe acquisition.
* The static global buffers have been removed and are now passed
* in via an AcqThreadBuffers struct. The main functions have been
* refactored into "tasks" that return an AcqResult struct instead
* of modifying global state.
*/

#include <omp.h>  // OpenMP for parallelization
#include "fastgps.h"
#include "acquisition.h" // <-- Include the new header
#include <thread>
#include <future>

extern "C"
{
#include "kiss_fft.h"
#include "kiss_fftr.h"
}

// DECLARATIONS for functions *local* to this file
AcqFineResult fine_acquisition_task(int prn, int best_doppler_idx, double max_shift,
                                    const AcqThreadBuffers& buffers);

// --- ORIGINAL GLOBALS (STILL USED) ---

char acq_buf[153000];     // sampling freq 38.192e6 max
unsigned acq_buf_write_pos;

// These are READ-ONLY and thread-safe after init_fft_acq()
static kiss_fft_cpx *code_fft[MAX_SATELLITES];
extern gps_real_t dopplers[NUM_COARSE_DOPPLERS],
    fine_dopplers[NUM_FINE_DOPPLERS];

// Debug file handles (Thread-unsafe, avoid in parallel tasks)
FILE *acq_debug;
FILE *acq_debug2;


// --- NEW BUFFER MANAGEMENT FUNCTIONS ---

/**
 * @brief Allocates and initializes a new set of buffers for an acquisition thread.
 */
AcqThreadBuffers create_acq_thread_buffers(int acq_fft_len) {
    AcqThreadBuffers buffers;

    // Allocate memory
    buffers.sample_buf = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * acq_fft_len);
    buffers.sample_fft = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * acq_fft_len);
    buffers.inverse_fft = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * acq_fft_len);
    buffers.mult_result = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * acq_fft_len);
    buffers.max_shift_energy = (double *)malloc(sizeof(double) * acq_fft_len);

    // Initialize to zero
    memset(buffers.sample_buf, 0, sizeof(kiss_fft_cpx) * acq_fft_len);
    memset(buffers.sample_fft, 0, sizeof(kiss_fft_cpx) * acq_fft_len);
    memset(buffers.inverse_fft, 0, sizeof(kiss_fft_cpx) * acq_fft_len);
    memset(buffers.mult_result, 0, sizeof(kiss_fft_cpx) * acq_fft_len);
    memset(buffers.max_shift_energy, 0, sizeof(double) * acq_fft_len);

    // Allocate FFT configurations
    buffers.sample_fft_cfg = kiss_fft_alloc(acq_fft_len, 0, NULL, NULL);
    buffers.inverse_fft_cfg = kiss_fft_alloc(acq_fft_len, 1, NULL, NULL);

    return buffers;
}

/**
 * @brief Frees all memory associated with an AcqThreadBuffers struct.
 */
void free_acq_thread_buffers(AcqThreadBuffers& buffers) {
    free(buffers.sample_buf);
    free(buffers.sample_fft);
    free(buffers.inverse_fft);
    free(buffers.mult_result);
    free(buffers.max_shift_energy);
    kiss_fft_free(buffers.sample_fft_cfg);
    kiss_fft_free(buffers.inverse_fft_cfg);
}


// --- ORIGINAL FUNCTION (Kept for reference, but no longer used by parallel task) ---
// This function writes to acq_debug2, which is not thread-safe.
void complete_delay_search(unsigned int prn, double freq)
{
    char nco_sin, nco_cos, *code;
    int code_idx_prompt;
    unsigned s;
    double fa_energy;
    code = CODE_TABLE[prn-1];
    unsigned i;
    double code_step = 0.1;
    unsigned loops = (unsigned) (1023.0/code_step);

    double code_inc = (CODE_FREQ) / system_vars.sampling_freq;
    double car_phase_inc = 2 * M_PI * (freq) / system_vars.sampling_freq;
    double code_prompt = 0;

    for (i = 0; i < loops; i++)
    {
        code_prompt += code_step;
        double car_phase = 0, ip = 0, qp = 0;

        for (s = 0; s < 16368; s++)
        {
            char sample = acq_buf[s];
            nco_sin = sample * GPS_SIN(car_phase);
            nco_cos = sample * GPS_COS(car_phase);
            car_phase += car_phase_inc;
            code_idx_prompt = (int)(code_prompt);
            code_prompt += code_inc;

            ip += code[code_idx_prompt] * nco_sin;
            qp += code[code_idx_prompt] * nco_cos;

            if (car_phase > M_PI)
                car_phase -= 2 * M_PI;
            if (code_prompt > CHIPS_PER_CODE + 1)
                code_prompt -= CHIPS_PER_CODE;
        }
        fa_energy = ip*ip + qp*qp;
        // NOT THREAD SAFE:
        // fprintf(acq_debug2, "%.5f ",fa_energy); // store it
    }  // end i loop
    // NOT THREAD SAFE:
    // fprintf(acq_debug2, "\n");
}

// **********************************************************************
//  REFACTORED: Perform fine Doppler search (Thread-Safe Task)
// **********************************************************************
AcqFineResult fine_acquisition_task(int prn, int best_doppler_idx, double max_shift,
                                    const AcqThreadBuffers& buffers)
{
    char nco_sin, nco_cos, *code;
    int code_idx_prompt;
    unsigned s;
    double fa_max_energy = 0, fa_energy;

    AcqFineResult result;
    result.found = false;

    // Local variables (no longer read from/written to a channel struct)
    double doppler, code_inc, car_phase_inc, code_prompt;
    double car_phase, ip, qp;

    // retrieve PRN code for this satellite
    code = CODE_TABLE[prn-1];

    // Loop through fine Doppler bins at signal code delay (max_shift)
    for (int fine_doppler_idx = 0; fine_doppler_idx < NUM_FINE_DOPPLERS; fine_doppler_idx++)
    {
        doppler = dopplers[best_doppler_idx] + fine_dopplers[fine_doppler_idx];
        code_inc = (CODE_FREQ + doppler * CARRIER_AID_SF) / system_vars.sampling_freq;
        car_phase_inc = 2 * M_PI * (system_vars.IF + doppler) / system_vars.sampling_freq;
        code_prompt = CHIPS_PER_CODE - max_shift + 1;
        ip = 0;
        qp = 0;
        car_phase = 0; // Initialize car_phase for each bin

        // Perform correlation at this code and freq
        for (s = 0; s < system_vars.acq_buf_len; s++)
        {
            char sample = acq_buf[s];
            nco_sin = sample * GPS_SIN(car_phase);
            nco_cos = sample * GPS_COS(car_phase);
            car_phase += car_phase_inc;
            code_idx_prompt = (int)(code_prompt);
            code_prompt += code_inc;
            ip += code[code_idx_prompt] * nco_cos;
            qp -= code[code_idx_prompt] * nco_sin;
            if (car_phase > M_PI)
                car_phase -= 2 * M_PI;
            if (code_prompt > CHIPS_PER_CODE + 1)
                code_prompt -= CHIPS_PER_CODE;
        }

        // calculate max energy
        fa_energy = ip*ip + qp*qp;

        // keep the best value
        if (fa_energy > fa_max_energy)
        {
            fa_max_energy = fa_energy;
            result.found = true;
            result.doppler_idx = fine_doppler_idx;
            result.doppler_val = doppler;
        }
    }

    // Return the best values found
    return result;
}


// **********************************************************************
//  REFACTORED: Main Acquisition Task (Thread-Safe)
//  This replaces acq_buffer_full2()
// **********************************************************************
AcqResult acquire_satellite_task(int prn, const AcqThreadBuffers& buffers)
{
    double tempd;
    unsigned s; // sample index

    // Create a result struct to return. Defaults to 'found = false'.
    AcqResult result;
    result.prn = prn;

    unsigned acq_fft_len = kiss_fft_next_fast_size(system_vars.acq_buf_len);

    // Use thread-local buffers from the struct
    for (s = 0; s < system_vars.acq_buf_len; s++){
        buffers.max_shift_energy[s] = 0;
        buffers.sample_buf[s].r = 0;
        buffers.sample_buf[s].i = 0;
    }

    // Local variables for this task's search state
    double avg_energy = 1.0;
    double max_energy = 0.0;
    double max_doppler = 0.0;
    double max_shift = 0.0;
    int best_doppler_idx = 0;

    // Doppler loop
    for (int doppler_idx = 0; doppler_idx < NUM_COARSE_DOPPLERS; doppler_idx++)
    {
        int nco_sin, nco_cos, i, q;
        double car_phase = 0.0;
        double car_phase_inc = 2 * M_PI * (system_vars.IF + dopplers[doppler_idx]) /
                               system_vars.sampling_freq;

        // load I and Q samples into FFT buffer
        for (s = 0; s < system_vars.acq_buf_len; s++)
        {
            char sample = acq_buf[s]; // acq_buf is global, but read-only
            nco_sin = GPS_SIN(car_phase);
            nco_cos = GPS_COS(car_phase);
            i =  sample * nco_cos;
            q = -sample * nco_sin;
            car_phase += car_phase_inc;
            UNWRAP_ANGLE(car_phase);
            buffers.sample_buf[s].r = i;
            buffers.sample_buf[s].i = q;
        }

        // Perform FFT on data and multiply with (pre-calculated) PRN code FFT
        // Note: code_fft is global, but read-only and thus thread-safe
        kiss_fft(buffers.sample_fft_cfg, buffers.sample_buf, buffers.sample_fft);
        for (s = 0; s < system_vars.acq_buf_len; s++)
        {
            buffers.mult_result[s].r = buffers.sample_fft[s].r * code_fft[prn-1][s].r -
                                       buffers.sample_fft[s].i * code_fft[prn-1][s].i;
            buffers.mult_result[s].i = buffers.sample_fft[s].r * code_fft[prn-1][s].i +
                                       buffers.sample_fft[s].i * code_fft[prn-1][s].r;
        }

        // Perform Inverse FFT of frequency domain multiplication
        kiss_fft(buffers.inverse_fft_cfg, buffers.mult_result, buffers.inverse_fft);

        // search the IFFT result for signal peaks
        for (s = 0; s < system_vars.acq_buf_len; s++)
        {
            double d = buffers.inverse_fft[s].r * buffers.inverse_fft[s].r +
                       buffers.inverse_fft[s].i * buffers.inverse_fft[s].i;
            unsigned idx = s % (system_vars.acq_buf_len / ACQ_MS);

            if (d > buffers.max_shift_energy[idx])
            {
                buffers.max_shift_energy[idx] = d;
                if (d > max_energy)
                {
                    max_doppler = dopplers[doppler_idx];
                    best_doppler_idx = doppler_idx;

                    max_shift = (s % (acq_fft_len / ACQ_MS))   /
                                (double)(acq_fft_len / ACQ_MS) * (double)(CHIPS_PER_CODE);

                    max_energy = d;
                }
            }
            avg_energy += d;
        }
    } // end of Doppler loop

    avg_energy /= (system_vars.acq_buf_len * NUM_COARSE_DOPPLERS); // Correct avg energy calc
    tempd = max_energy / avg_energy;

    // If the max to avg ratio indicates a signal is present
    if(tempd > COARSE_ACQ_THRESH)
    {
        fastgps_printf("coarse acq for PRN %d, best doppler: %f\n", prn, max_doppler);

        // Perform fine frequency scan, without FFT's
        AcqFineResult fine_res = fine_acquisition_task(prn, best_doppler_idx,
                                                       max_shift, buffers);

        if (fine_res.found)
        {
            // --- SUCCESS ---
            // Store results for tracking
            result.found = true;
            result.doppler = fine_res.doppler_val;
            result.ratio = tempd;
            result.code_phase = max_shift;

            result.true_car_phase_inc = 2 * M_PI * (system_vars.IF + result.doppler) /
                                        system_vars.sampling_freq;
            result.true_code_inc = (CODE_FREQ + result.doppler * CARRIER_AID_SF) /
                                   system_vars.sampling_freq;
            result.track_carrier_freq = system_vars.IF + result.doppler;
            result.track_code_freq = CODE_FREQ + result.doppler * CARRIER_AID_SF;

            fastgps_printf("PRN %d FOUND: freq: %f, doppler = %f, ratio: %f \n",
                           prn, result.track_carrier_freq,
                           result.doppler, result.ratio);

            // This debug block is NOT thread-safe due to file I/O.
            // It has been removed for the parallel task.
            /*
            if(system_vars.acq_log_flag == DEBUG_LOGGING)
            {
                if (acq_debug == NULL)
                    acq_debug = fopen("acq_debug_log.dat","w");
                if (acq_debug2 == NULL)
                    acq_debug2 = fopen("acq_debug2_log.dat","w");

                complete_delay_search(prn, result.track_carrier_freq);

                for (s = 0; s < system_vars.acq_buf_len; s++)
                {
                    double dd;
                    dd = buffers.inverse_fft[s].r * buffers.inverse_fft[s].r +
                         buffers.inverse_fft[s].i * buffers.inverse_fft[s].i;
                    fprintf(acq_debug, "%.5f ",dd);
                }
                fprintf(acq_debug, "\n");
            }
            */

            return result; // Return the populated result struct
        }
    }

    // --- FAILURE ---
    // No signal found, return the default 'found = false' struct
    return result;
}


// **********************************************************************
//  KissFFT initialization routine
//  MODIFIED: This function no longer allocates the thread-local buffers.
//  It only allocates the shared, read-only code_fft tables.
// **********************************************************************
void init_fft_acq()
{
    // sample the C/A code for all satellites
    uint8_t sv_num, ch_idx;
    gps_real_t code_time, code_time_inc;
    unsigned s;
    kiss_fft_scalar *sampled_code;
    kiss_fftr_cfg forward_fft_cfg;

    fastgps_printf("precalculating C/A code FFT tables...");
    // sample_idx = 0;  // <-- This line was removed as it was undeclared and unused
    code_time_inc = 1.0 / system_vars.sampling_freq;
    unsigned acq_fft_len = kiss_fft_next_fast_size(system_vars.acq_buf_len);
    sampled_code = (kiss_fft_scalar *)malloc(sizeof(kiss_fft_cpx) * acq_fft_len);
    memset(sampled_code, 0, sizeof(kiss_fft_cpx) * acq_fft_len);
    fastgps_printf("acq buf len = %d\n", acq_fft_len);
    forward_fft_cfg = kiss_fftr_alloc(acq_fft_len, 0, NULL, NULL);
    acq_buf_write_pos = 0;

    for (sv_num = 0; sv_num < MAX_SATELLITES; sv_num++)
    {
        code_fft[sv_num] = (kiss_fft_cpx *)malloc(sizeof(kiss_fft_cpx) * acq_fft_len);
        memset(code_fft[sv_num], 0, sizeof(kiss_fft_cpx) * acq_fft_len);
        code_time = 0;
        for (s = 0; s < system_vars.acq_buf_len / ACQ_MS; s++)
        {
            uint8_t ms_counter;
            code_time += code_time_inc;
            for (ms_counter = 0; ms_counter < ACQ_MS; ms_counter++)
                sampled_code[s + ms_counter * system_vars.acq_buf_len / ACQ_MS] =
                    CODE_TABLE[sv_num][(int)floor(CHIPS_PER_CODE * 1000 * code_time) + 1];
        }
        kiss_fftr(forward_fft_cfg, sampled_code, code_fft[sv_num]);
        // why are we doing this flip? I can't remember
        for (s = 0; s < system_vars.acq_buf_len; s++)
            code_fft[sv_num][s].i *= -1;
    }

    // --- REMOVED STATIC BUFFER ALLOCATION ---
    // This is now done in create_acq_thread_buffers()

    for (ch_idx = 0; ch_idx < MAX_CHANNELS; ch_idx++)
        c[ch_idx].acq.failure_count = 0;

    kiss_fft_free(forward_fft_cfg);
    free(sampled_code);
    fastgps_printf("done\n");
}

// **********************************************************************
//  KissFFT clean up
//  MODIFIED: This function no longer frees the thread-local buffers.
// **********************************************************************
void shutdown_fft_acq()
{
    for (uint8_t sv = 0; sv < MAX_SATELLITES; sv++)
        free(code_fft[sv]);

    // --- REMOVED STATIC BUFFER FREEING ---
    // This is now done in free_acq_thread_buffers()

    kiss_fft_cleanup();
}


/**
 * @brief Reads acquisition results from a log file.
 *
 * This function is used when system_vars.file_acquisition is ON.
 * It parses the "fastgps_acquisition_log.dat" file to populate
 * the tracking channels, bypassing the need for FFT-based acquisition.
 */
unsigned read_acquisiton_file()
{
    double testd = 0;
    int testi = 0;
    unsigned long testul = 0;
    char tempc = 0;
    FILE *infile;
    unsigned tempchan = 0;
    int prn_num = 0;
    double temp_ratio = 0;
    int acq_finish_time = 0;
    int sv_num;

    /* Open configuration file */
    infile = fopen("fastgps_acquisition_log.dat", "r");
    if (!infile)
    {
        fastgps_printf("Couldn't open acquisition file 'fastgps_acquisition_log.dat'.\n");
        return PROBLEM;
    }
    rewind(infile);  // make sure we are at the beginning

    system_vars.num_channels = 0; // Reset channel count

    /* Read in info from file */
    while (!feof(infile))  /* until end of file */
    {
        VERIFY_IO(fread(&tempc, 1, 1, infile), 1);
        if (tempc == '/')
        {
            VERIFY_IO(fread(&tempc, 1, 1, infile), 1);
            if (tempc == 'A') // Found an acquired satellite
            {
                if (system_vars.num_channels >= MAX_CHANNELS)
                {
                    fastgps_printf("Found more satellites in acq file than MAX_CHANNELS. Stopping.\n");
                    break; // We have filled all available channels
                }

                // Use the next available channel index
                unsigned ch_idx = system_vars.num_channels;

                /* Read acq info entry */
                VERIFY_IO(fscanf(infile, " %lf", &testd), 1); // process_time
                VERIFY_IO(fscanf(infile, " %lu", &testul), 1); // loop_count
                VERIFY_IO(fscanf(infile, " %d", &sv_num), 1); // sv
                VERIFY_IO(fscanf(infile, " %d", &tempchan), 1); // tempchan (original, unused)
                VERIFY_IO(fscanf(infile, " %d", &prn_num), 1); // prn_num
                VERIFY_IO(fscanf(infile, " %lf", &testd), 1); // 0.0
                VERIFY_IO(fscanf(infile, " %lf", &temp_ratio), 1); // ratio
                VERIFY_IO(fscanf(infile, " %lf", &testd), 1); // 0.0

                // Read the rest of the data directly into the channel struct
                VERIFY_IO(fscanf(infile, " %lf", &c[ch_idx].true_car_phase_inc), 1);
                VERIFY_IO(fscanf(infile, " %lf", &c[ch_idx].true_code_inc), 1);
                VERIFY_IO(fscanf(infile, " %lf", &c[ch_idx].track.carrier_freq), 1);
                VERIFY_IO(fscanf(infile, " %lf", &c[ch_idx].track.code_freq), 1);
                VERIFY_IO(fscanf(infile, " %lf", &c[ch_idx].doppler), 1);
                VERIFY_IO(fscanf(infile, " %lf", &c[ch_idx].code_prompt), 1);
                VERIFY_IO(fscanf(infile, " %d", &acq_finish_time), 1); // Read acq_finish_time

                // Populate the channel struct
                c[ch_idx].prn_num = prn_num;
                c[ch_idx].acq.ratio = temp_ratio;
                c[ch_idx].acq.acq_finish_time = acq_finish_time;

                c[ch_idx].num_ms = 0;
                c[ch_idx].tracking_lock = 0;
                c[ch_idx].state = CH_STATE_POSTACQ_SPIN;

                // Update global state
                system_vars.sats_found |= ((uint64_t)1 << prn_num);
                system_vars.prn_to_channel_map[prn_num] = ch_idx;
                system_vars.num_channels++; // Increment for the satellite we just added

                // Prepare the *next* channel index (if not maxed out)
                if (system_vars.num_channels < MAX_CHANNELS) {
                    init_correlator_channel(system_vars.num_channels);
                }

                fastgps_printf("Read acq data for PRN %d, allocated to channel %d\n", prn_num, ch_idx);
            }
        } // end if '/'
    } // end while

    fclose(infile);

    if (system_vars.num_channels > 0)
        return SUCCESS;
    else
        return PROBLEM;
}
