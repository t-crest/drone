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


#include "ahrs.h"

#include "fix32math.h"

//#include "debug.h"

// Filter gain 'beta' in degrees per second; should be choosen according to:
// beta = sqrt(3/4) * w_beta
// where w_beta is the estimated mean zero gyroscope measurement error;
// we choose a w_beta of 5 deg/s, which corresponds to 0.087266463 rad/s:
#define BETA            0x4D6386A7  // sqrt(3/4) * w_beta, scaling factor 2^34
#define BETA_SCALE      34

// Sampling period 'delta_t' in seconds:
//#define DELTA_T         0x418937AC  // 0.001 with a scaling factor of 2^40
//#define DELTA_T_SCALE   40
//#define DELTA_T         0x51EB851F  // 0.08 with a scaling factor of 2^34
//#define DELTA_T_SCALE   34
#define DELTA_T         0x4EC4EC4F  // 1/208 with a scaling factor of 2^38
#define DELTA_T_SCALE   38

#define SQ(a, n)    fix_mul(a, a, n)

// attitude quaternion; scaling factor 2^30, initialized to (1,0,0,0):
static int32_t q1 = 1<<30, q2 = 0, q3 = 0, q4 = 0;

/**
 * Update the attitude quaternion with accelerometer and gyroscope data.
 *
 * Scale and unit of the acceleration vector are irrelevant (gets normalized).
 * Angular rate vector is in radians per second with a scaling factor of 2^25.
 */
void ahrs_update(int32_t a_x, int32_t a_y, int32_t a_z,
                 int32_t w_x, int32_t w_y, int32_t w_z)
{
    //DEBUG("AHRS Update:");
    //DEBUG("a = ", DBG_FIXVEC(a_x, a_y, a_z, 0), "\n");
    //DEBUG("w = ", DBG_FIXVEC(w_x, w_y, w_z, 25));

    // Normalize acceleration vector (the units and scale of the acceleration
    // vector are irrelevant, since it is first normalized here):
    {
        // calculate inverse norm; shifting by 34 bits after multiplying avoids
        // arithmetic overflow (also while summing); so initialize scale to 34:
        int32_t invn; int scale = 34;
        invn = fix_invsqrt(SQ(a_x, 34) + SQ(a_y, 34) + SQ(a_z, 34), &scale);

        // multiply each vector element by the inverse norm; after this:
        // -1 <= a_x, a_y, a_z <= 1 ; a scaling factor of 2^28 is sufficient,
        // so we shift by an additional 6 bits to get from 2^34 to 2^28:
        a_x = fix_mul(a_x, invn, scale + 6);
        a_y = fix_mul(a_y, invn, scale + 6);
        a_z = fix_mul(a_z, invn, scale + 6);
    }

    //DEBUG("a / |a| = ", DBG_FIXVEC(a_x, a_y, a_z, 28), "\n");

    // Objective function:       / 2 (q2 * q4 - q1 * q3) - a_x \
    //                     f_g = | 2 (q1 * q2 + q3 * q4) - a_y |
    //                           \ 2 (0.5 - q2^2 - q3^2) - a_z /
    // all values of the attitude quaternion and the acceleration vector are
    // within [-1,1], thus the elements of f_g are within [-4,4]; we choose a
    // scaling factor of 2^28 (the attitude quaternion has a scaling of 2^30):
    int32_t f_1 = fix_mul(q2, q4, 31) - fix_mul(q1, q3, 31) - a_x,
            f_2 = fix_mul(q1, q2, 31) + fix_mul(q3, q4, 31) - a_y,
            f_3 = (1<<28) - fix_mul(q2, q2, 31) - fix_mul(q3, q3, 31) - a_z;

    //DEBUG("f_g = ", DBG_FIXVEC(f_1, f_2, f_3, 28), "\n");

    // Gradient of the attitude quaternion based on accelerometer data:
    // grad(f_g) = J_g^T * f_g, where J_g^T is the transposed Jacobian; the
    // Jacobian itself is:       /  -2 q3     2 q4    -2 q1    2 q2  \
    //                     J_g = |   2 q2     2 q1     2 q4    2 q3  |
    //                           \    0      -4 q2    -4 q3     0    /
    // we keep the scaling factor of 2^28 (the attitude quaternion has 2^30)
    // and take care of the factors 2 and 4 during the scaling operations:
    int32_t qhd_1, qhd_2, qhd_3, qhd_4;
    qhd_1 = fix_mul(q2, f_2, 29) - fix_mul(q3, f_1, 29);
    qhd_2 = fix_mul(q4, f_1, 29) + fix_mul(q1, f_2, 29) - fix_mul(q2, f_3, 28);
    qhd_3 = fix_mul(q4, f_2, 29) - fix_mul(q1, f_1, 29) - fix_mul(q3, f_3, 28);
    qhd_4 = fix_mul(q2, f_1, 29) + fix_mul(q3, f_2, 29);

    //DEBUG("qhd = ", DBG_FIXQUAT(qhd_1, qhd_2, qhd_3, qhd_4, 28), "\n");

    // Normalize the gradient and multiply it with beta:
    {
        // calculate inverse norm; values of the gradient are within [-4,4], so
        // the sum of squares is within [0,64]; use a scaling factor of 2^24,
        // by shifting the squares by an additional 4 bits from 2^28 to 2^24:
        int32_t invn; int scale = 24;
        invn = fix_invsqrt(SQ(qhd_1, 32) + SQ(qhd_2, 32) + SQ(qhd_3, 32) +
                           SQ(qhd_4, 32), &scale);

        // multiply the inverse norm with beta; for the result a scaling factor
        // of 2^(scale + BETA_SCALE - 32) is choosen:
        invn = fix_mul(invn, BETA, 32);

        // multiply each element of qhd by the inverse norm times beta; after
        // this the range of each value of qhd is [-beta, beta]; regardless, we
        // switch to a scaling factor of 2^26 for qhd (from currently 2^28) due
        // to the subsequent subtraction of qhd from qd (the derivative of q):
        scale = (scale + BETA_SCALE - 32) + (28 - 26); //(20 - 28);
        qhd_1 = fix_mul(qhd_1, invn, scale);
        qhd_2 = fix_mul(qhd_2, invn, scale);
        qhd_3 = fix_mul(qhd_3, invn, scale);
        qhd_4 = fix_mul(qhd_4, invn, scale);
    }

    //DEBUG("qhd / |qhd| * beta = ", DBG_FIXQUAT(qhd_1, qhd_2, qhd_3, qhd_4, 26));
    //DEBUG(" (with beta = ", DBG_FIX(BETA, BETA_SCALE), ")\n");

    // Derivative of the attitude quaternion from angular rate measurements:
    // qd = dq / dt = 0.5 q (*) [ 0  w_x  w_y  w_z ]
    // where (*) is the quaternion product and (w_x, w_y, w_z) are the angular
    // rate measures; all values of the attitude quaternion are within [-1,1],
    // hence we use a scaling factor of 2^26 for qd, twice that of the angular
    // rate data, to take into account the factor of 0.5 (i.e. division by 2):
    int32_t qd_1, qd_2, qd_3, qd_4;
    qd_1 = -fix_mul(q2, w_x, 30) - fix_mul(q3, w_y, 30) - fix_mul(q4, w_z, 30);
    qd_2 =  fix_mul(q1, w_x, 30) + fix_mul(q3, w_z, 30) - fix_mul(q4, w_y, 30);
    qd_3 =  fix_mul(q1, w_y, 30) + fix_mul(q4, w_x, 30) - fix_mul(q2, w_z, 30);
    qd_4 =  fix_mul(q1, w_z, 30) + fix_mul(q2, w_y, 30) - fix_mul(q3, w_x, 30);

    //DEBUG("qd = ", DBG_FIXQUAT(qd_1, qd_2, qd_3, qd_4, 26), "\n");

    // Attitude quaternion difference from derivative and gradient estimates:
    // subtract the gradient from the derivative estimate and integrate the
    // result over the time interval delta_t (i.e. multiply with delta_t):
    int32_t dq1, dq2, dq3, dq4;
    dq1 = fix_mul(qd_1 - qhd_1, DELTA_T, DELTA_T_SCALE - 4);
    dq2 = fix_mul(qd_2 - qhd_2, DELTA_T, DELTA_T_SCALE - 4);
    dq3 = fix_mul(qd_3 - qhd_3, DELTA_T, DELTA_T_SCALE - 4);
    dq4 = fix_mul(qd_4 - qhd_4, DELTA_T, DELTA_T_SCALE - 4);

    //DEBUG("(qd - qhd) * delta_t = ", DBG_FIXQUAT(dq1, dq2, dq3, dq4, 30), "\n");

    // also, the values must be converted from degrees to radiants, therefore
    // delta_t is first multiplied with pi/180 (constant DEG2RAD); the values
    // are converted to the scaling factor of 2^30 of the attitude quaternion:
    //{
        //int32_t delta_t_deg2rad = fix_mul(DELTA_T, DEG2RAD, 32);
        //int scaling = (DELTA_T_SCALE + DEG2RAD_SCALE - 32) + (30 - 20);
        //q1 += fix_mul(qd_1 - qhd_1, delta_t_deg2rad, scaling);
    //}

    // Update attitude quaternion:
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;
    q4 += dq4;

    //DEBUG("q = ", DBG_FIXQUAT(q1, q2, q3, q4, 30), "\n");

    // Normalize the new attitude quaternion:
    {
        // calculate inverse norm; should be close to 1, so we keep the scaling
        // factor of 2^30 for the sum of squares:
        int32_t invn; int scale = 30;
        invn = fix_invsqrt(SQ(q1, 30) + SQ(q2, 30) + SQ(q3, 30) + SQ(q4, 30),
                           &scale);

        // multiply each vector element by the inverse norm:
        q1 = fix_mul(q1, invn, scale);
        q2 = fix_mul(q2, invn, scale);
        q3 = fix_mul(q3, invn, scale);
        q4 = fix_mul(q4, invn, scale);
    }

    //DEBUG("q / |q| = ", DBG_FIXQUAT(q1, q2, q3, q4, 30), "\n");

    //uart_printf("`drone_ctrl.update(np.arctan2(2 * ((%8ld/(2**30)) * (%8ld/(2**30)) + (%8ld/(2**30)) * (%8ld/(2**30))), 1 - 2 * ((%8ld/(2**30))**2 + (%8ld/(2**30))**2)),", q1, q2, q3, q4, q2, q3)
    //uart_printf(" np.arcsin(2 * ((%8ld/(2**30)) * (%8ld/(2**30)) - (%8ld/(2**30)) * (%8ld/(2**30)))),", q1, q3, q4, q2)
    //uart_printf(" np.arctan2(2 * ((%8ld/(2**30)) * (%8ld/(2**30)) + (%8ld/(2**30)) * (%8ld/(2**30))), 1 - 2 * ((%8ld/(2**30))**2 + (%8ld/(2**30))**2)))`\n", q1, q4, q2, q3, q3, q4)
}

void ahrs_get_attitude(int32_t *q0p, int32_t *q1p, int32_t *q2p, int32_t *q3p)
{
    *q0p = q1; *q1p = q2; *q2p = q3; *q3p = q4;
}
