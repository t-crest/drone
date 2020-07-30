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


#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

/**
 * Predict the current position and velocity based on accelerometer data.
 *
 * If there is no position update for approximately 10 seconds the filter goes
 * into saturation mode and stops increasing the covariances of the state
 * estimate to avoid overflow of the state covariance matrix.
 *
 * @param[in]   dd_xi       acceleration vector
 * @param[out]  xi_est      estimate of position vector
 * @param[out]  d_xi_est    estimate of velocity vector
 * @return                  0 for normal operation, 1 if saturated
 */
int kalman_predict(const int32_t dd_xi[3],
                   int32_t xi_est[3], int32_t d_xi_est[3]);

/**
 * Update the position and velocity estimates with a new position measurement.
 *
 * @param[in]   xi_meas     measured position vector
 * @param[out]  xi_est      estimate of position vector
 * @param[out]  d_xi_est    estimate of velocity vector
 */
void kalman_update_pos(const int32_t xi_meas[3],
                       int32_t xi_est[3], int32_t d_xi_est[3]);

/**
 * Update the position and velocity estimates with a new velocity measurement.
 *
 * @param[in]   d_xi_meas   measured velocity vector
 * @param[out]  xi_est      estimate of position vector
 * @param[out]  d_xi_est    estimate of velocity vector
 */
void kalman_update_vel(const int32_t d_xi_meas[3],
                       int32_t xi_est[3], int32_t d_xi_est[3]);


#define XI_SCALE        25  // range limited to [-63, 63]
#define D_XI_SCALE      25  // range limited to [-63, 63]
#define DD_XI_SCALE     23  // accelerometer range up to 16 g, i.e. [-157, 157]

#endif // KALMAN_H
