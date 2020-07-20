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
 * Fixed-point implementation for an efficient attitude estimation algorithm.
 *
 * The attitude estimation uses the method proposed by Madgwick et al. [1],
 * implemented with fixed-point computation rather than the floating-point
 * arithmetic used by the reference implementation provided by the author [2].
 *
 * [1] S. O. H. Madgwick, A. J. L. Harrison and R. Vaidyanathan, "Estimation of
 *     imu and marg orientation using a gradient descent algorithm", 2011 IEEE
 *     International Conference on Rehabilitation Robotics, pp. 1-7, June 2011.
 *
 * [2] S. O. H. Madgwick, "An efficient orientation filter for inertial and
 *     inertial/magnetic sensor arrays", available online:
 *     https://x-io.co.uk/res/doc/madgwick_internal_report.pdf
 */


#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>

/**
 * Update the attitude quaternion with accelerometer and gyroscope data.
 *
 * @param[in] a_x, a_y, a_z: Acceleration vector, i.e. the acceleration in the
 *                           x-, y- and z-axis direction respectively; scale
 *                           irrelevant (will be normalized), should be as
 *                           large as allowed by the 32-bit integer type for
 *                           maximum precision.
 *
 * @param[in] w_x, w_y, w_z: Angular rate vector, i.e. the angular velocity in
 *                           radians per second around the x-, y- and z-axis
 *                           respectively, with a scaling factor of 2^19; thus,
 *                           values in the range [-4096, 4096) are allowed.
 */
void ahrs_update(int32_t a_x, int32_t a_y, int32_t a_z,
                 int32_t w_x, int32_t w_y, int32_t w_z);

/**
 * Get the current values of the attitude quaternion.
 *
 * @param[out] q0, q1, q2, q3: Attitude quaternion; the 32-bit integers
 *                             referenced by these pointers will be filled with
 *                             the current values of the attitude quaternion.
 *                             This is a unit quaternion (i.e. a quaternion of
 *                             norm 1, also called a versor; see:
 *                             https://en.wikipedia.org/wiki/Versor ) which
 *                             represents the attitude estimated from IMU
 *                             measurements in the rotation group SO(3).  The
 *                             scaling factor of the quaternion values is 2^30,
 *                             though all values are guaranteed to be in the
 *                             range [-1, 1].
 */
void ahrs_get_attitude(int32_t *q0, int32_t *q1, int32_t *q2, int32_t *q3);
#define QUAT_SCALE  30

#endif // AHRS_H
