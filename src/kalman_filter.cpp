#include "kalman_filter.h"

float estimated_altitude = 0.0;
float error_covariance_bmp = 1.0;
float process_variance_bmp = 0.001;
float measurement_variance_bmp = 0.1;
float kalman_gain_bmp = 0.1;

float x_acc_offset = 0.0;

/**
 * Kalman filter matrices
 */
void init_kalman_matrices() {
    F = {1, 0.002, 0, 1};
    G = {0.5 * 0.002 * 0.002, 0.002};
    H = {1, 0};
    I = {1, 0, 0, 1};
    Q = G * ~G * 4.0f * 4.0f;
    R = {0.3 * 0.3};
    P = {0, 0, 0, 0};
    S = {0, 0};
}