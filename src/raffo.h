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


/**
 * Fixed-point implementation of a quadcopter controller.
 *
 * The drone controller uses the method proposed by Raffo et. al. [1],
 * implemented with fixed-point computation. More details on the algorithm can
 * be found in the dissertation of the author [2].
 *
 * This is a combined attitude and position controller, but it can also be used
 * for attitude control only by setting the position and velocity errors to 0.
 * Although the control algorithm technically requires the rotors to be tilted
 *
 * [1] G. V. Raffo, M. G. Ortega and F. R. Rubio, "Nonlinear H-infinity
 *     controller for the quad-rotor helicopter with input coupling", 18th
 *     IFAC World Congress, vol. 44, no. 1, pp. 13834-13839, 2011.
 *
 * [2] G. V. Raffo, "Robust control strategies for a quadrotor helicopter",
 *     Doctoral Thesis, Universidad de Sevilla, Escuela Tecnica Superior de
 *     Ingenieria, 2011.
 */


#ifndef RAFFO_H
#define RAFFO_H

#include <stdint.h>

/**
 * @param[in]   R_W         rotation matrix from body to world frame
 * @param[in]   omega       angular rate vector (in body frame coordinates)
 * @param[in]   xi_err      position error vector (in world frame coordinates)
 * @param[in]   d_xi_err    velocity error vector ( -"- )
 * @param[in]   dd_xi_des   desired acceleration vector ( -"- )
 * @param[in]   yaw_des     desired yaw angle ( -"- )
 * @param[in]   d_yaw_des   desired derivative of yaw angle ( -"- )
 * @param[out]  dd_xi_ctrl  target acceleration of controller ( -"- )
 * @param[out]  f_prop      propeller thrusts (force normal to each propeller)
 */
void raffo_update(const int32_t R_W[3][3], const int32_t omega[3],
                  const int32_t xi_err[3], const int32_t d_xi_err[3],
                  const int32_t dd_xi_des[3],
                  int32_t yaw_des, int32_t d_yaw_des,
                  //int32_t dd_xi_ctrl[3],
                  int32_t f_prop[4]);

#define R_W_SCALE       30  // all values within [-1, 1]
#define OMEGA_SCALE     25  // gyroscope measurements may be in range [-40, 40]
#define XI_SCALE        25  // range limited to [-63, 63]
#define D_XI_SCALE      25  // range limited to [-63, 63]
#define DD_XI_SCALE     23  // accelerometer range up to 16 g, i.e. [-157, 157]
#define YAW_SCALE       28  // all values within [-pi, pi]
#define F_PROP_SCALE    24  // range limited to [-127,127]

#endif // RAFFO_H
