/*
 *  Copyright (c) 2020, Michael Platzer (TU Wien)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  SPDX-License-Identifier: BSD-2-Clause
 */


#include "kalman.h"

#include "fix32math.h"

// delta_t
#define DELTA_T         0x4EC4EC4F  // 1/208 with a scaling factor of 2^38
#define DELTA_T_SCALE   38

// delta_t^2 / 2
#define DELTA_T_SQ_HALF         0x60F25DEB  // 1/86528 with scaling factor 2^47
#define DELTA_T_SQ_HALF_SCALE   47

// State vector estimate (i.e. position and velocity vector estimates):
static int32_t xi[3]   = { 0, 0, 0 };
static int32_t d_xi[3] = { 0, 0, 0 };

// Estimated error covariance matrix (estimated accuracy of state estimate),
// initialized with the uncertainty of the initial position and velocity.
// Covariance matrices are symmetric, so instead of using the entire 2x2 matrix
// we use a vector with 3 values: P[0] is the variance of xi (P[0,0] in the
// full matrix), P[1] is the covariance of xi and d_xi (P[0,1] and P[1,0]) and
// P[2] is the variance of d_xi (P[1,1] in the full matrix).
// The scaling factor is choosen such that P does not overflow if there is no
// position update for 10 seconds; in that case P[0] becomes the largest
// element with a value of 0.001179; hence we use a scaling factor of 2^40:
static int32_t P[3] = {
    0, 0, 0
};
#define P_MAX       0x4D445671  // 0.001179
#define P_SCALE     40

// Observation noise variance matrix (for position and velocity measurements),
// based on an estimated position measurement noise with a standard deviation
// of 1 mm (i.e. 0.001 m) and a velocity measurement noise with a standard
// deviation of 10 mm (i.e. 0.01 m);
// R has the same scaling factor of 2^40 as P:
static const int32_t R[2] = {
    //0x0010C6F8, 0x068DB8BB  // 10^-6, 10^-4
    0x0010C6F8, 0x0010C6F8  // 10^-6, 10^-6
};
#define R_SCALE     (P_SCALE)

// Process noise covariance matrix, based on an estimated accelerometer noise
// with a standard deviation of 0.0027611 g, i.e. 0.027086 m/s^2;
// the corresponding variance is multiplied with G * G^T to obtain Q:
// Q = /  delta_t^4 / 4    delta_t^3 / 2  \ * 0.027086^2
//     \  delta_t^3 / 2      delta_t^2    /
// Similar to P, we use a vector with 3 values instead of the full matrix to
// take advantage of the symmetry of covariance matrices; Q[0] corresponds to
// Q[0,0], Q[1] to Q[0,1] as well as Q[1,0] and Q[2] corresponds to Q[1,1].
// Q has the same scaling factor of 2^40 as P:
static const int32_t Q[3] = {
//    0x00000000, 0x0000002D, 0x000048D5  // 9.799E-14, 4.076E-11, 1.696E-8
    //0x00000001, 0x000001C0, 0x0002D86D  // 9.799E-13, 4.076E-10, 1.696E-7
    0x00000010, 0x00001C00, 0x002D86D0
//    { 0x00000000, 0x0000002D }, // 9.799E-14, 4.076E-11
//    { 0x0000002D, 0x000048D5 }, // 4.076E-11, 1.696E-8
};
#define Q_SCALE     (P_SCALE)

//#include "uart.h"

/**
 * Predict the current position and velocity based on accelerometer data.
 * Return 0 for normal operation, 1 if P is saturated.
 */
int kalman_predict(const int32_t dd_xi[3],
                   int32_t xi_est[3], int32_t d_xi_est[3])
{
    // Update position vector estimate (requires current velocity estimate,
    // therefore the position must be updated first):
#define DT_DXI_2_XI_SHFT    (DELTA_T_SCALE + D_XI_SCALE - XI_SCALE)
#define DT_DDXI_2_XI_SHFT   (DELTA_T_SQ_HALF_SCALE + DD_XI_SCALE - XI_SCALE)
    xi_est[0] = xi[0] += fix32_mul(DELTA_T, d_xi[0], DT_DXI_2_XI_SHFT)
                      +  fix32_mul(DELTA_T_SQ_HALF, dd_xi[0], DT_DDXI_2_XI_SHFT);
    xi_est[1] = xi[1] += fix32_mul(DELTA_T, d_xi[1], DT_DXI_2_XI_SHFT)
                      +  fix32_mul(DELTA_T_SQ_HALF, dd_xi[1], DT_DDXI_2_XI_SHFT);
    xi_est[2] = xi[2] += fix32_mul(DELTA_T, d_xi[2], DT_DXI_2_XI_SHFT)
                      +  fix32_mul(DELTA_T_SQ_HALF, dd_xi[2], DT_DDXI_2_XI_SHFT);

    // Update velocity vector estimate:
#define DT_DDXI_2_DXI_SHFT  (DELTA_T_SCALE + DD_XI_SCALE - D_XI_SCALE)
    d_xi_est[0] = d_xi[0] += fix32_mul(DELTA_T, dd_xi[0], DT_DDXI_2_DXI_SHFT);
    d_xi_est[1] = d_xi[1] += fix32_mul(DELTA_T, dd_xi[1], DT_DDXI_2_DXI_SHFT);
    d_xi_est[2] = d_xi[2] += fix32_mul(DELTA_T, dd_xi[2], DT_DDXI_2_DXI_SHFT);

    // Stop updating P if P[0] exceeds P_MAX (saturation mode):
    if (P[0] > P_MAX)
        return 1;

    // Update error covariance matrix estimate; take care to update any values
    // which depend on others first:
    //int32_t p11_dt = fix32_mul(DELTA_T, P[1][1], DELTA_T_SCALE);
    //P[0][0] += fix32_mul(DELTA_T, P[0][1] + P[1][0] + p11_dt, DELTA_T_SCALE) +
                                                                       //Q[0][0];
    //P[0][1] += p11_dt + Q[0][1];
    //P[1][0] += p11_dt + Q[1][0];
    //P[1][1] += Q[1][1];
    int32_t p2_dt = fix32_mul(DELTA_T, P[2], DELTA_T_SCALE);
    P[0] += fix32_mul(DELTA_T, P[1] + P[1] + p2_dt, DELTA_T_SCALE) + Q[0];
    P[1] += p2_dt + Q[1];
    P[2] += Q[2];

    //uart_printf("`drone_ctrl.log_write(2, '%8ld, %8ld, %8ld\\n')`\n",
    //            P[0], P[1], P[2]);
    return 0;
}

/**
 * Update the position and velocity estimates with a new position measurement.
 */
void kalman_update_pos(const int32_t xi_meas[3],
                       int32_t xi_est[3], int32_t d_xi_est[3])
{
    // the smallest possible value of S is R[0] (i.e. 10^-6), thus the largest
    // value of S_inv is 10^6; choosing a scaling factor of 2^11 for S_inv:
    int32_t S = P[0] + R[0],
            S_inv;
#define S_INV_SCALE     11
    {
        int scale = P_SCALE;
        int32_t S_inv_sqrt = fix32_invsqrt(S, &scale);
        S_inv = fix32_mul(S_inv_sqrt, S_inv_sqrt, 2 * scale - S_INV_SCALE);
    }

    // use a scaling factor for K such that we retain maximum precision while
    // guaranteed to avoid overflow:
#define K_SCALE     (P_SCALE + S_INV_SCALE - 31)
#define K_SHFT      (P_SCALE + S_INV_SCALE - K_SCALE)
    int32_t K[2] = {
        fix32_mul(P[0], S_inv, K_SHFT),
        fix32_mul(P[1], S_inv, K_SHFT)
    };

    // Difference between estimate and measurement:
    int32_t y[3] = {
        xi_meas[0] - xi[0],
        xi_meas[1] - xi[1],
        xi_meas[2] - xi[2]
    };

    // Update position vector estimate:
#define K_Y_2_XI_SHFT   (K_SCALE + XI_SCALE - XI_SCALE)
    xi_est[0] = xi[0] += fix32_mul(K[0], y[0], K_Y_2_XI_SHFT);
    xi_est[1] = xi[1] += fix32_mul(K[0], y[1], K_Y_2_XI_SHFT);
    xi_est[2] = xi[2] += fix32_mul(K[0], y[2], K_Y_2_XI_SHFT);

    // Update velocity vector estimate:
#define K_Y_2_DXI_SHFT  (K_SCALE + XI_SCALE - D_XI_SCALE)
    d_xi_est[0] = d_xi[0] += fix32_mul(K[1], y[0], K_Y_2_DXI_SHFT);
    d_xi_est[1] = d_xi[1] += fix32_mul(K[1], y[1], K_Y_2_DXI_SHFT);
    d_xi_est[2] = d_xi[2] += fix32_mul(K[1], y[2], K_Y_2_DXI_SHFT);

    // Update error covariance matrix estimate; take care to update any values
    // which depend on others first:
    //int32_t _1 = (1 << K_SCALE); // the value 1 with scaling factor K_SCALE
    //P[0][0]  = fix32_mul(_1 - K[0], P[0][0], K_SCALE);
    //P[0][1]  = fix32_mul(_1 - K[0], P[0][1], K_SCALE);
    //P[1][0] += fix32_mul(-K[1], P[0][0], K_SCALE); // + P[1][0];
    //P[1][1] += fix32_mul(-K[1], P[0][1], K_SCALE); // + P[1][1];
    P[0] -= fix32_mul(K[0], P[0], K_SCALE);
    P[2] -= fix32_mul(K[1], P[1], K_SCALE); // uses P[1], modify first!
    P[1] -= fix32_mul(K[0], P[1], K_SCALE);
}

/**
 * Update the position and velocity estimates with a new velocity measurement.
 */
void kalman_update_vel(const int32_t d_xi_meas[3],
                       int32_t xi_est[3], int32_t d_xi_est[3])
{
    // the smallest possible value of S is R[1] (i.e. 10^-4), thus the largest
    // value of S_inv is 10^4; choosing a scaling factor of 2^11 for S_inv:
    int32_t S = P[2] + R[1],
            S_inv;
#define S_INV_SCALE     11
    {
        int scale = P_SCALE;
        int32_t S_inv_sqrt = fix32_invsqrt(S, &scale);
        S_inv = fix32_mul(S_inv_sqrt, S_inv_sqrt, 2 * scale - S_INV_SCALE);
    }

    // use a scaling factor for K such that we retain maximum precision while
    // guaranteed to avoid overflow:
#define K_SCALE     (P_SCALE + S_INV_SCALE - 31)
#define K_SHFT      (P_SCALE + S_INV_SCALE - K_SCALE)
    int32_t K[2] = {
        fix32_mul(P[1], S_inv, K_SHFT),
        fix32_mul(P[2], S_inv, K_SHFT)
    };

    // Difference between estimate and measurement:
    int32_t y[3] = {
        d_xi_meas[0] - d_xi[0],
        d_xi_meas[1] - d_xi[1],
        d_xi_meas[2] - d_xi[2]
    };

    // Update position vector estimate:
#define K_Y_2_XI_SHFT   (K_SCALE + XI_SCALE - XI_SCALE)
    xi_est[0] = xi[0] += fix32_mul(K[0], y[0], K_Y_2_XI_SHFT);
    xi_est[1] = xi[1] += fix32_mul(K[0], y[1], K_Y_2_XI_SHFT);
    xi_est[2] = xi[2] += fix32_mul(K[0], y[2], K_Y_2_XI_SHFT);

    // Update velocity vector estimate:
#define K_Y_2_DXI_SHFT  (K_SCALE + XI_SCALE - D_XI_SCALE)
    d_xi_est[0] = d_xi[0] += fix32_mul(K[1], y[0], K_Y_2_DXI_SHFT);
    d_xi_est[1] = d_xi[1] += fix32_mul(K[1], y[1], K_Y_2_DXI_SHFT);
    d_xi_est[2] = d_xi[2] += fix32_mul(K[1], y[2], K_Y_2_DXI_SHFT);

    // Update error covariance matrix estimate; take care to update any values
    // which depend on others first:
    P[0] -= fix32_mul(K[0], P[1], K_SCALE);
    P[1] -= fix32_mul(K[0], P[2], K_SCALE);
    P[2] -= fix32_mul(K[1], P[2], K_SCALE);
}
