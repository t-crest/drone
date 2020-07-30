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


#include <stdint.h>
#include <stdio.h>
#include <machine/spm.h>

//#include "uart.h"
//#include "i2c_master.h"
//#include "imu.h"
#include "ahrs.h"
#include "kalman.h"
#include "raffo.h"

//#define FIX_MATH_MUL_OVERFLOW_ACTION(val)                                      \
//    uart_puts("`raise ValueError('FIX_MUL OVERFLOW')`\n")
#include "fix32math.h"

//#include "debug.h"


void get_imu_data(int32_t accel[3], int32_t omega[3]);

int kalman_estimate(const int32_t dd_xi[3],
                    int32_t xi_est[3], int32_t d_xi_est[3],
                    int32_t update_xi[3], int32_t update_dxi[3]);

void control(const int32_t R_W[3][3], const int32_t omega[3],
             const int32_t xi_est[3], const int32_t d_xi_est[3],
             int cnt, int32_t dd_xi_ctrl[3]);


//#define KALMAN_ESTIMATE

int main()
{
    /*
    if (imu_init() < 0) {
        uart_puts("IMU init failed\n");
        return -1;
    }
    uart_puts("IMU init done\n");
    */

#define CTRL_LOOP_ITERATIONS 1024

    int idx;
    for (idx = 0; idx < CTRL_LOOP_ITERATIONS; idx++) {
        int32_t accel[3], omega[3];
        get_imu_data(accel, omega);

        ahrs_update(accel[0], accel[1], accel[2], omega[0], omega[1], omega[2]);

        // Rotation matrix from attitude quaternion:
        int32_t R_W[3][3];
        {
            int32_t q0, q1, q2, q3;
            ahrs_get_attitude(&q0, &q1, &q2, &q3);

            static const int shft = QUAT_SCALE + QUAT_SCALE - R_W_SCALE - 1;

            int32_t _2q1sq = fix32_mul(q1, q1, shft), // 2 * q1^2
                    _2q2sq = fix32_mul(q2, q2, shft), // 2 * q2^2
                    _2q3sq = fix32_mul(q3, q3, shft), // 2 * q3^2
                    _2q0q1 = fix32_mul(q0, q1, shft), // 2 * q0 * q1
                    _2q0q2 = fix32_mul(q0, q2, shft), // 2 * q0 * q2
                    _2q0q3 = fix32_mul(q0, q3, shft), // 2 * q0 * q3
                    _2q1q2 = fix32_mul(q1, q2, shft), // 2 * q1 * q2
                    _2q1q3 = fix32_mul(q1, q3, shft), // 2 * q1 * q3
                    _2q2q3 = fix32_mul(q2, q3, shft), // 2 * q2 * q3
                    _1 = 1 << R_W_SCALE; // 1 with the scaling factor of R_W

            R_W[0][0] = _1 - (_2q2sq + _2q3sq);
            R_W[1][1] = _1 - (_2q1sq + _2q3sq);
            R_W[2][2] = _1 - (_2q1sq + _2q2sq);
            R_W[0][1] = _2q1q2 - _2q0q3;
            R_W[0][2] = _2q1q3 + _2q0q2;
            R_W[1][0] = _2q1q2 + _2q0q3;
            R_W[1][2] = _2q2q3 - _2q0q1;
            R_W[2][0] = _2q1q3 - _2q0q2;
            R_W[2][1] = _2q2q3 + _2q0q1;
        }

        // Gravitational constant g = 9.81 m/s^2;
        // using the same scaling factor of 2^23 as for dd_xi:
        int32_t g_const = 0x04E7AE14; // 9.81 with scaling factor 2^23

        // Convert accelerometer data to body acceleration vector (i.e. convert
        // g to m/s^2 by multiplying with the gravitational constant);
        // accelerometer data has a range of +/-8 g and uses the full range of
        // the data type, so when given in g, the scaling factor is 2^28; the
        // corresponding acceleration vector has a range of +/-78.48 m/s^2:
        accel[0] = fix32_mul(g_const, accel[0], 28);
        accel[1] = fix32_mul(g_const, accel[1], 28);
        accel[2] = fix32_mul(g_const, accel[2], 28);

        // Convert acceleration vector to world frame:
        int32_t dd_xi[3] = {
            fix32_mul(R_W[0][0], accel[0], R_W_SCALE) + fix32_mul(R_W[0][1], accel[1], R_W_SCALE) + fix32_mul(R_W[0][2], accel[2], R_W_SCALE),
            fix32_mul(R_W[1][0], accel[0], R_W_SCALE) + fix32_mul(R_W[1][1], accel[1], R_W_SCALE) + fix32_mul(R_W[1][2], accel[2], R_W_SCALE),
            fix32_mul(R_W[2][0], accel[0], R_W_SCALE) + fix32_mul(R_W[2][1], accel[1], R_W_SCALE) + fix32_mul(R_W[2][2], accel[2], R_W_SCALE)
        };
        // Subtract gravity from acceleration vector:
        dd_xi[2] -= g_const;


        int32_t xi_est[3] = { 0, 0, 0 }, d_xi_est[3] = { 0, 0, 0 };
#ifdef KALMAN_ESTIMATE
        int32_t update_xi[3]  = { 0x80000000, 0x80000000, 0x80000000 },
                update_dxi[3] = { 0x80000000, 0x80000000, 0x80000000 };
        kalman_estimate(dd_xi_ctrl, xi_est, d_xi_est, update_xi, update_dxi);
#endif

        int32_t dd_xi_ctrl[3] = { 0, 0, 0 };
        control(R_W, omega, xi_est, d_xi_est, idx, dd_xi_ctrl);
    }

    return 0;
}


void get_imu_data(int32_t accel[3], int32_t omega[3])
{
    int32_t a_x, a_y, a_z, // acceleration vector from accel measurements
            g_x, g_y, g_z; // angular rate vector from gyro measurements

    /*
    while (imu_poll_accel(&a_x, &a_y, &a_z) < 0 && *imu_status_ptr == 0)
        ;
    while (imu_poll_gyro(&g_x, &g_y, &g_z) < 0 && *imu_status_ptr == 0)
        ;
    */
    a_x = 0;
    a_y = 0;
    a_z = 1<<12;

    g_x = 0;
    g_y = 0;
    g_z = 0;

    // rotate IMU data 180 deg about y axis (i.e. invert x and z axis):
    a_x = -a_x; a_z = -a_z;
    g_x = -g_x; g_z = -g_z;

    // Convert the gyroscope data to degrees per second:
    {
        // when using a range of +-2000 degrees per second, the sensitivity
        // of the gyroscope data is 0.07 dps / LSB (least significant bit),
        // which corresponds to 1.22173 mrad / s / LSB; therefore the
        // gyroscope values must be multiplied by 0.00122173; the 16 bit
        // values from the ADC potentially fill the range [-40.03, 40.03],
        // thus we select a scaling factor of 2^25:
        //int32_t _70m = 0x47AE147B; // 0.07 with a scaling factor of 2^34
        int32_t lsb2rad = 0x50113A65; // .00122173 with scaling factor 2^40
        g_x = fix32_mul(g_x, lsb2rad, 40 - 25);
        g_y = fix32_mul(g_y, lsb2rad, 40 - 25);
        g_z = fix32_mul(g_z, lsb2rad, 40 - 25);
    }

    // Shift accelerometer data to retain maximum precision:
    {
        // the 16 bit values from the ADC are shifted by 16
        // for maximum resolution:
        a_x <<= 16;
        a_y <<= 16;
        a_z <<= 16;
    }

    accel[0] = a_x;
    accel[1] = a_y;
    accel[2] = a_z;
    omega[0] = g_x;
    omega[1] = g_y;
    omega[2] = g_z;
}


#ifdef KALMAN_ESTIMATE
int kalman_estimate(const int32_t dd_xi[3],
                    int32_t xi_est[3], int32_t d_xi_est[3],
                    int32_t update_xi[3], int32_t update_dxi[3])
{
    kalman_predict(dd_xi, xi_est, d_xi_est);

    // Check if we have received new position data, in which case the
    // Kalman position estimate is updated with the new data:
    {
        static unsigned int magic = 0;
#define MAGIC   0x33CC33CC

        // search for magic number
        {
            int val;
            while (magic != MAGIC && (val = uart_try_getc()) >= 0) {
                magic <<= 8;
                magic |= val & 0xff;
            }
        }

        static unsigned int time_since_last_update = 0;

        if (magic == MAGIC) {
            static uint8_t uart_buf[24];
            static int pos = 0;

            int val;
            while (pos < sizeof(uart_buf) && (val = uart_try_getc()) >= 0)
                uart_buf[pos++] = val;

            if (pos == sizeof(uart_buf)) {
                int32_t xi_meas[3] = {
                    *((int32_t *)(uart_buf)),
                    *((int32_t *)(uart_buf + 4)),
                    *((int32_t *)(uart_buf + 8))
                };
                int32_t d_xi_meas[3] = {
                    *((int32_t *)(uart_buf + 12)),
                    *((int32_t *)(uart_buf + 16)),
                    *((int32_t *)(uart_buf + 20))
                };

                // reject outliers:
#define XY_RANGE    0x06000000  // 3 with a scaling factor of 2^25
#define Z_POS_RANGE 0x06000000  // 3 with a scaling factor of 2^25
#define Z_NEG_RANGE 0x02000000  // 1 with a scaling factor of 2^25
                if ( -XY_RANGE   <= xi_meas[0] && xi_meas[0] <=  XY_RANGE  &&
                     -XY_RANGE   <= xi_meas[1] && xi_meas[1] <=  XY_RANGE  &&
                    -Z_NEG_RANGE <= xi_meas[2] && xi_meas[2] <= Z_POS_RANGE)
                    kalman_update_pos(xi_meas, xi_est, d_xi_est);
                    kalman_update_vel(d_xi_meas, xi_est, d_xi_est);

                    update_xi[0] = xi_meas[0];
                    update_xi[1] = xi_meas[1];
                    update_xi[2] = xi_meas[2];
                    update_dxi[0] = d_xi_meas[0];
                    update_dxi[1] = d_xi_meas[1];
                    update_dxi[2] = d_xi_meas[2];

                pos = 0;
                magic = 0;
                time_since_last_update = 0;
            }
        }
        else if (++time_since_last_update > 54)
            return -1;
    }

    return 0;
}
#endif


void control(const int32_t R_W[3][3], const int32_t omega[3],
             const int32_t xi_est[3], const int32_t d_xi_est[3],
             int cnt, int32_t dd_xi_ctrl[3])
{
    int32_t target_height = 1 << (XI_SCALE - 3); // 0.125 meter

    int32_t xi_err[3] = {
        xi_est[0], xi_est[1], xi_est[2] - target_height
    };
    int32_t d_xi_err[3] = {
        d_xi_est[0], d_xi_est[1], d_xi_est[2]
    };
    int32_t dd_xi_des[3] = {
        0, 0, 0
    };

    int32_t f_prop[4];
    raffo_update(R_W, omega, xi_err, d_xi_err, dd_xi_des, 0, 0, f_prop);

    dd_xi_ctrl[0] = 0;
    dd_xi_ctrl[1] = 0;
    dd_xi_ctrl[2] = 0;


    {
        uint32_t motor_pwm_out[4] = { 57000, 57000, 57000, 57000 };

        int idx;
        for (idx = 0; idx < 4; idx++) {
            int32_t pwm = 57000 + fix32_mul(5200, f_prop[idx], F_PROP_SCALE);
            if (pwm < 53500)
                pwm = 53500;
            if (pwm > 107000)
                pwm = 107000;
            motor_pwm_out[idx] = pwm;
        }

        //motor_out(motor_pwm_out);
    }
}





