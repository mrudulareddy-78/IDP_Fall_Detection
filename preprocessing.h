// ============================================================
// preprocessing.h
// Butterworth LP filter (zero-phase) + Z-score normalization
// Values taken directly from Colab output
// ============================================================
#pragma once
#include <string.h>
#include <math.h>

#define N_CH  8
#define WIN   300

// ── Z-score constants from norm_mean.npy / norm_std.npy ──────
// Verified from Colab Cell 25 output
static const float NORM_MEAN[N_CH] = {
    -0.01956503f,   // ch0 acc_x
    -1.64415810f,   // ch1 acc_y
    -0.22443111f,   // ch2 acc_z
    -0.11315934f,   // ch3 gyro_x
     0.52972990f,   // ch4 gyro_y
    -0.05186092f,   // ch5 gyro_z
     2.54105880f,   // ch6 acc_magnitude
     4.50325800f    // ch7 gyro_magnitude
};

static const float NORM_STD[N_CH] = {
     1.0817714f,    // ch0 acc_x
     1.4196227f,    // ch1 acc_y
     1.2396472f,    // ch2 acc_z
     6.9351864f,    // ch3 gyro_x
     7.3953150f,    // ch4 gyro_y
     4.3507776f,    // ch5 gyro_z
     0.8308321f,    // ch6 acc_magnitude
     9.8446320f     // ch7 gyro_magnitude
};

// ── Butterworth filter coefficients ──────────────────────────
// scipy.signal.butter(6, 20.0/50.0, btype='low')
// order=6, cutoff=20Hz, fs=100Hz
#define FORD 6

static const double B[FORD+1] = {
    0.00042654, 0.00255924, 0.00639810,
    0.00853080, 0.00639810, 0.00255924, 0.00042654
};
static const double A[FORD+1] = {
    1.00000000, -2.97684959,  4.18149626,
   -3.68684262,  2.12189544, -0.76864498,  0.14658064
};

// ── One-direction IIR pass (Direct Form II Transposed) ───────
static void _filter_pass(float* sig, int n, double w[FORD]) {
    for (int i = 0; i < n; i++) {
        double x = sig[i];
        double y = B[0]*x + w[0];
        for (int j = 1; j < FORD; j++)
            w[j-1] = B[j]*x - A[j]*y + w[j];
        w[FORD-1] = B[FORD]*x - A[FORD]*y;
        sig[i] = (float)y;
    }
}

static void _reverse(float* sig, int n) {
    for (int i = 0; i < n/2; i++) {
        float t = sig[i]; sig[i] = sig[n-1-i]; sig[n-1-i] = t;
    }
}

// ── Zero-phase filter = Python filtfilt ──────────────────────
static void zero_phase_filter(float* sig, int n) {
    double w[FORD] = {0};
    _filter_pass(sig, n, w);   // forward
    _reverse(sig, n);
    memset(w, 0, sizeof(w));
    _filter_pass(sig, n, w);   // backward
    _reverse(sig, n);
}

// ── Full pipeline ─────────────────────────────────────────────
// Call this on the 300-sample window just before inference
void preprocess_window(float window[WIN][N_CH]) {
    static float col[WIN];

    // Step 1 — filter each channel
    for (int ch = 0; ch < N_CH; ch++) {
        for (int t = 0; t < WIN; t++) col[t] = window[t][ch];
        zero_phase_filter(col, WIN);
        for (int t = 0; t < WIN; t++) window[t][ch] = col[t];
    }

    // Step 2 — Z-score normalize
    for (int ch = 0; ch < N_CH; ch++) {
        float mu = NORM_MEAN[ch];
        float sd = NORM_STD[ch];
        for (int t = 0; t < WIN; t++)
            window[t][ch] = (window[t][ch] - mu) / sd;
    }
}
