/**
 * @file kalman_filter.h
 * 
 * Implement Kalman Filter
 * 
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "defs.h"

extern float estimated_altitude;
extern float error_covariance_bmp;
extern float process_variance_bmp;
extern float measurement_variance_bmp;
extern float kalman_gain_bmp;

/* Kalman matrices for altitude and vertical velocity */
extern float altitude_kalman, velocity_vertical_kalman;
extern BLA::Matrix<2,2> F, P, Q, I;
extern BLA::Matrix<2,1> G, S, K;
extern BLA::Matrix<1,2> H;
extern BLA::Matrix<1,1> R, L, inv_L, Acc, M;

/* MPU variables */
extern float x_acc;
extern float x_acc_g;
extern float x_acc_offset;
extern float roll, pitch;

extern float bmp_altitude;

void init_kalman_matrices();

#endif