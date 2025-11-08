#ifndef ACQUISITION_H
#define ACQUISITION_H

#include "kiss_fft.h"
#include "kiss_fftr.h"

/*
 * Note: <stdbool.h> or a C++ equivalent is needed for 'bool'.
 * If you get an error, add `#include <stdbool.h>` here.
 * C++ files like fastgps.cpp will understand 'bool' automatically.
 */


/**
 * @brief Holds the temporary buffers required for one acquisition task.
 *
 * This struct allows each acquisition thread to have its own private set of
 * buffers, eliminating the thread-safety problems from using static globals
 * in acquire.cpp.
 */
struct AcqThreadBuffers {
    kiss_fft_cpx *sample_buf;
    kiss_fft_cpx *sample_fft;
    kiss_fft_cpx *mult_result;
    kiss_fft_cpx *inverse_fft;
    double *max_shift_energy;
    kiss_fft_cfg sample_fft_cfg;
    kiss_fft_cfg inverse_fft_cfg;
};

/**
 * @brief Holds the result of a fine acquisition search.
 * (Internal helper struct for acquire.cpp)
 */
struct AcqFineResult {
    bool   found = false;
    double doppler_val = 0.0;
    int    doppler_idx = 0;
};

/**
 * @brief Holds the complete result of a parallel acquisition task for one PRN.
 *
 * This struct is returned by the task and contains all information
 * needed by the main thread to initialize a tracking channel.
 */
struct AcqResult {
    bool   found = false;
    int    prn = 0;
    double doppler = 0.0;
    double code_phase = 0.0;
    double ratio = 0.0;

    // Fields needed to initialize the channel
    double true_car_phase_inc = 0.0;
    double true_code_inc = 0.0;
    double track_carrier_freq = 0.0;
    double track_code_freq = 0.0;
};


// --- Public Function Prototypes from acquire.cpp ---

/**
 * @brief The main parallelizable acquisition task.
 * Searches for a single satellite.
 */
AcqResult acquire_satellite_task(int prn, const AcqThreadBuffers& buffers);

/**
 * @brief Allocates and initializes a new set of buffers for an acquisition thread.
 */
AcqThreadBuffers create_acq_thread_buffers(int acq_fft_len);

/**
 * @brief Frees all memory associated with an AcqThreadBuffers struct.
 */
void free_acq_thread_buffers(AcqThreadBuffers& buffers);


#endif // ACQUISITION_H
