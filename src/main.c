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

#include "uart.h"
#include "i2c_master.h"
#include "imu.h"
#include "ahrs.h"
#include "kalman.h"
#include "raffo.h"

//#define FIX_MATH_MUL_OVERFLOW_ACTION(val)                                      \
//    uart_puts("`raise ValueError('FIX_MUL OVERFLOW')`\n")
#include "libfix/fix_math.h"

#include "debug.h"

static volatile _SPM int *sram_base = (volatile _SPM int *)0xF00E0000;
void sram_test()
{
    uint32_t vals[] = { 0x55AA55AA, 0xAA55AA55, 0x33CC33CC, 0xCC33CC33 };
    const int vals_cnt = sizeof(vals) / sizeof(uint32_t);

    int i;
    for (i = 0; i < vals_cnt; i++) {
        volatile _SPM int *addr = sram_base + (i << 2);
        uint32_t val = vals[i];
        *addr = vals[i];
        uart_printf("writing 0x%08lx to SRAM address 0x%08lx\n", val, (uint32_t)addr);
    }
    for (i = 0; i < vals_cnt; i++) {
        volatile _SPM int *addr = sram_base + (i << 2);
        uint32_t val = *addr;
        uart_printf("reading 0x%08lx from SRAM address 0x%08lx\n", val, (uint32_t)addr);
    }
}

void run_simulation(void)
{
    typedef struct {
        uint32_t magic;
        int32_t R_W[3][3],
                omega[3],
                xi[3],
                d_xi[3],
                dd_xi[3];
    } uart_recv_data;
    //uart_printf("Starting sim, uart_recv_data has size %lu bytes\n",
    //            sizeof(uart_recv_data));
    uart_puts("`drone_ctrl.init_sim()`\n");

    int cnt;
    for (cnt = 0; 1; cnt++) {
        // trigger writing of state vector to uart:
        uart_printf("`patpydow_ser_write(drone_ctrl.get_state(%d))`\n", cnt);
        //uart_puts("Requested state vector transmission\n");

        uart_recv_data recv_data;
        {
            unsigned u;
            for (u = 0; u < sizeof(recv_data); u++)
                ((char *)(&recv_data))[u] = uart_getc();
        }

        if (recv_data.magic != 0x33CC33CC) {
            uart_puts("Sync lost\n");
            continue;
        }
        //uart_printf("Received state update\n");

        //int32_t dd_xi_ctrl[3], f_prop[4];
        int32_t f_prop[4];
        raffo_update(recv_data.R_W, recv_data.omega,
                     recv_data.xi, recv_data.d_xi, recv_data.dd_xi,
                     0, 0,
                     //dd_xi_ctrl,
                     f_prop);

        //uart_printf("Updated control thrusts\n");

        uart_printf("`drone_ctrl.ctrl_thrusts(%d, [ %8ld, %8ld, %8ld, %8ld ], %d)`\n",
                    cnt, f_prop[0], f_prop[1], f_prop[2], f_prop[3], F_PROP_SCALE);
    }
}



static volatile _SPM unsigned int *timer_cycles_low_ptr = (volatile _SPM unsigned int *) 0xF0020004;

void get_imu_data(int32_t accel[3], int32_t omega[3]);

int kalman_estimate(const int32_t dd_xi[3],
                    int32_t xi_est[3], int32_t d_xi_est[3],
                    int32_t update_xi[3], int32_t update_dxi[3]);

//static int32_t ctrl_log[512][23];
//static int32_t ctrl_log[1024][26];
static int32_t ctrl_log[1024][28];

void control(const int32_t R_W[3][3], const int32_t omega[3],
             const int32_t xi_est[3], const int32_t d_xi_est[3],
             int cnt, int32_t dd_xi_ctrl[3]);

void update_gui_compact(const int32_t pos[3], const int32_t vel[3]);
//void update_gui(const int32_t pos[3]);

static volatile _SPM int *motor_0 = (volatile _SPM int *)0xF00B0000;
static volatile _SPM int *motor_1 = (volatile _SPM int *)0xF00B0004;
static volatile _SPM int *motor_2 = (volatile _SPM int *)0xF00B0008;
static volatile _SPM int *motor_3 = (volatile _SPM int *)0xF00B000C;
void motor_out(uint32_t pwm_out[4])
{
    *motor_0 = pwm_out[0];
    *motor_1 = pwm_out[1];
    *motor_2 = pwm_out[2];
    *motor_3 = pwm_out[3];
}
void motor_test(void);

//#define RUN_SIMULATION
#define KALMAN_ESTIMATE
#define RUN_CONTROL
//#define CTRL_PANEL_GUI

int main()
{
    uart_puts("Hello Artix!\n");
    //uart_puts("`import drone_ctrl_mod as drone_ctrl`\n");

    //motor_test();
    //sram_test();
    //return 0;

#ifdef RUN_SIMULATION
    run_simulation(); // does not return
#endif

    if (imu_init() < 0) {
        uart_puts("IMU init failed\n");
        return -1;
    }
    uart_puts("IMU init done\n");

#ifdef CTRL_PANEL_GUI
    uart_puts("`drone_ctrl.init_gui()`\n");
#endif

    //uart_puts("`drone_ctrl.open_logfile(1, 'accel_data.csv')`\n");
    //uart_puts("`drone_ctrl.open_logfile(2, 'kalman.csv')`\n");

    //uint32_t t_start, t_data, t_conv, t_update, t_end;


    // arm motor controllers
    {
        uint32_t motor_pwm_out[4] = { 53500, 53500, 53500, 53500 };
        motor_out(motor_pwm_out);
    }

    // startup procedure
    int idx;
    for (idx = 0; idx < 2048; idx++) {
        int32_t accel[3], omega[3];
        get_imu_data(accel, omega);

        // verify accelerometer orientation
        if (idx < 1024) {
            if (accel[0] < -(1<<24) || accel[0] > (1<<24) ||
                accel[1] < -(1<<24) || accel[1] > (1<<24) ||
                accel[2] < (1<<28) - (1<<26) || accel[2] > (1<<28) + (1<<24)
               ) {
                uart_printf("ERROR: wrong orientation (accel values: %ld, %ld, %ld)\n", accel[0], accel[1], accel[2]);

                uint32_t motor_pwm_out[4] = { 53500, 53500, 53500, 53500 };
                motor_out(motor_pwm_out);
                return 0;
            }
        }

        // start motors halfway through
        if (idx == 1024) {
            uint32_t motor_pwm_out[4] = { 60000, 60000, 60000, 60000 };
            motor_out(motor_pwm_out);
        }
        if (idx == 1024 + 256) {
            uint32_t motor_pwm_out[4] = { 65000, 65000, 65000, 65000 };
            motor_out(motor_pwm_out);
        }
        if (idx == 1024 + 512) {
            uint32_t motor_pwm_out[4] = { 68000, 68000, 68000, 68000 };
            motor_out(motor_pwm_out);
        }
        if (idx == 1024 + 512 + 256) {
            uint32_t motor_pwm_out[4] = { 70000, 70000, 70000, 70000 };
            motor_out(motor_pwm_out);
        }
    }

    uart_puts("calibrating accelerometer\n");

    // calibrate accelerometer
    static int32_t acc_calib[3] = { 0, 0, 0 };

    for (idx = 0; idx < 1024; idx++) {
        int32_t accel[3], omega[3];
        get_imu_data(accel, omega);

        // accelerometer calibration:
        /*
        a_x += -0x00804D0C; // -0.031323479217233 with scaling factor 2^28
        a_y +=  0x00423077; //  0.016159502047937 with scaling factor 2^28
        //a_z +=  0x004EDD7B; //  0.019254188578749 with scaling factor 2^28
        a_z +=  0x005A1CAC; //  0.022 with scaling factor 2^28
        */

        acc_calib[0] += accel[0] >> 10;
        acc_calib[1] += accel[1] >> 10;
        acc_calib[2] += (accel[2] - (1<<28)) >> 10;
    }

    //uart_printf("accel calib: %ld, %ld, %ld\n", acc_calib[0], acc_calib[1], acc_calib[2]);
    uart_puts("`drone_ctrl.init_optitrack(patpydow_ser_write)`\n");

    uint32_t t_state_estim, t_control;

    int32_t dd_xi_ctrl[3] = { 0, 0, 0 };

    for (idx = 0; 1; idx++) {
        int32_t accel[3], omega[3];
        get_imu_data(accel, omega);

        accel[0] -= acc_calib[0];
        accel[1] -= acc_calib[1];
        accel[2] -= acc_calib[2];

        //uart_printf("`drone_ctrl.log_write(1, ','.join([ str(v / (2**28)) for v in [ %ld, %ld, %ld ] ]) + '\\n')`\n", a_x, a_y, a_z);

        /*
        if (((*imu_status_ptr) & 1)) {
            uart_printf("AUX: a = ( %6d, %6d, %6d ) ; g = ( %6d, %6d, %6d )\n",
                        IMU_GYRO_X, IMU_GYRO_Y, IMU_GYRO_Z,
                        IMU_ACCEL_X, IMU_ACCEL_Y, IMU_ACCEL_Z);
        } else
            uart_puts("No update on AUX interface\n");
        */

        //t_conv = *timer_cycles_low_ptr;

        uint32_t t_state_estim_start, t_state_estim_stop;
        t_state_estim_start = *timer_cycles_low_ptr;

        //ahrs_update(a_x, a_y, a_z, g_x, g_y, g_z);
        ahrs_update(accel[0], accel[1], accel[2], omega[0], omega[1], omega[2]);

        // Rotation matrix from attitude quaternion:
        int32_t R_W[3][3];
        {
            int32_t q0, q1, q2, q3;
            ahrs_get_attitude(&q0, &q1, &q2, &q3);

            static const int shft = QUAT_SCALE + QUAT_SCALE - R_W_SCALE - 1;

            int32_t _2q1sq = fix_mul(q1, q1, shft), // 2 * q1^2
                    _2q2sq = fix_mul(q2, q2, shft), // 2 * q2^2
                    _2q3sq = fix_mul(q3, q3, shft), // 2 * q3^2
                    _2q0q1 = fix_mul(q0, q1, shft), // 2 * q0 * q1
                    _2q0q2 = fix_mul(q0, q2, shft), // 2 * q0 * q2
                    _2q0q3 = fix_mul(q0, q3, shft), // 2 * q0 * q3
                    _2q1q2 = fix_mul(q1, q2, shft), // 2 * q1 * q2
                    _2q1q3 = fix_mul(q1, q3, shft), // 2 * q1 * q3
                    _2q2q3 = fix_mul(q2, q3, shft), // 2 * q2 * q3
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

        /*
        if ((idx & 0xFF) == 0) {
            DEBUG("      / ",  DBG_FIXVEC(R_W[0][0], R_W[0][1], R_W[0][2], R_W_SCALE), " \\\n");
            DEBUG("R_W = | ",  DBG_FIXVEC(R_W[1][0], R_W[1][1], R_W[1][2], R_W_SCALE), " |\n");
            DEBUG("      \\ ", DBG_FIXVEC(R_W[2][0], R_W[2][1], R_W[2][2], R_W_SCALE), " /\n \n");
        }
        */

        // Gravitational constant g = 9.81 m/s^2;
        // using the same scaling factor of 2^23 as for dd_xi:
        int32_t g_const = 0x04E7AE14; // 9.81 with scaling factor 2^23

        // Convert accelerometer data to body acceleration vector (i.e. convert
        // g to m/s^2 by multiplying with the gravitational constant);
        // accelerometer data has a range of +/-8 g and uses the full range of
        // the data type, so when given in g, the scaling factor is 2^28; the
        // corresponding acceleration vector has a range of +/-78.48 m/s^2:
        //int32_t accel[3] = {
            //fix_mul(g_const, a_x, 28),
            //fix_mul(g_const, a_y, 28),
            //fix_mul(g_const, a_z, 28)
        //};
        accel[0] = fix_mul(g_const, accel[0], 28);
        accel[1] = fix_mul(g_const, accel[1], 28);
        accel[2] = fix_mul(g_const, accel[2], 28);

        // Convert acceleration vector to world frame:
        int32_t dd_xi[3] = {
            fix_mul(R_W[0][0], accel[0], R_W_SCALE) + fix_mul(R_W[0][1], accel[1], R_W_SCALE) + fix_mul(R_W[0][2], accel[2], R_W_SCALE),
            fix_mul(R_W[1][0], accel[0], R_W_SCALE) + fix_mul(R_W[1][1], accel[1], R_W_SCALE) + fix_mul(R_W[1][2], accel[2], R_W_SCALE),
            fix_mul(R_W[2][0], accel[0], R_W_SCALE) + fix_mul(R_W[2][1], accel[1], R_W_SCALE) + fix_mul(R_W[2][2], accel[2], R_W_SCALE)
        };
        // Subtract gravity from acceleration vector:
        dd_xi[2] -= g_const;

        //uart_printf("`drone_ctrl.log_write(1, ','.join([ str(v / (2**%d)) for v in [ %ld, %ld, %ld ] ]) + '\\n')`\n", DD_XI_SCALE, dd_xi[0], dd_xi[1], dd_xi[2]);

        /*
        if ((idx & 0xFF) == 0) {
            DEBUG("accel = ", DBG_FIXVEC(accel_W[0], accel_W[1], accel_W[2], 28), "\n \n");
        }
        */
        //uart_printf("accel_W = ( %08lX, %08lX, %08lX )\n", a_x, a_y, a_z);

        t_state_estim_stop = *timer_cycles_low_ptr;
        t_state_estim = t_state_estim_stop - t_state_estim_start;

        //static int32_t xi_est[3] = { 0, 0, 0 }, d_xi_est[3] = { 0, 0, 0 };
        int32_t xi_est[3] = { 0, 0, 0 }, d_xi_est[3] = { 0, 0, 0 };
#ifdef KALMAN_ESTIMATE
        //int32_t omega[3] = { g_x, g_y, g_z };

        int32_t update_xi[3] = { 0x80000000, 0x80000000, 0x80000000 },
                update_dxi[3] = { 0x80000000, 0x80000000, 0x80000000 };
        int abort = kalman_estimate(dd_xi_ctrl, xi_est, d_xi_est, update_xi, update_dxi);

        /*
        if ((idx & 0xFF) == 0xFF) {
            int32_t xi_meas[3] = { 0, 0, 0 };
            kalman_update(xi_meas, xi_est, d_xi_est);
        }
        */

#ifdef RUN_CONTROL
        //if (idx < 512)
        if (idx < 1024 && abort == 0) {
            uint32_t t_control_start, t_control_stop;
            t_control_start = *timer_cycles_low_ptr;

            control(R_W, omega, xi_est, d_xi_est, idx, dd_xi_ctrl);

            t_control_stop = *timer_cycles_low_ptr;
            t_control = t_control_stop - t_control_start;
        }
        else {
            uint32_t motor_pwm_out[4] = { 53500, 53500, 53500, 53500 };
            motor_out(motor_pwm_out);
            break;
        }

        ctrl_log[idx][17] = dd_xi[0];
        ctrl_log[idx][18] = dd_xi[1];
        ctrl_log[idx][19] = dd_xi[2];
        //ctrl_log[idx][20] = dd_xi_ctrl[0];
        //ctrl_log[idx][21] = dd_xi_ctrl[1];
        //ctrl_log[idx][22] = dd_xi_ctrl[2];
        ctrl_log[idx][20] = update_xi[0];
        ctrl_log[idx][21] = update_xi[1];
        ctrl_log[idx][22] = update_xi[2];
        ctrl_log[idx][23] = update_dxi[0];
        ctrl_log[idx][24] = update_dxi[1];
        ctrl_log[idx][25] = update_dxi[2];

        ctrl_log[idx][26] = t_state_estim;
        ctrl_log[idx][27] = t_control;
#endif
#endif

        //t_update = *timer_cycles_low_ptr;

#ifdef CTRL_PANEL_GUI
        if ((idx & 0xF) == 0)
            //update_gui(xi_est);
            update_gui_compact(xi_est, d_xi_est);
#endif
    }

    uart_puts("Motors off, transmitting log\n");

    uart_puts("`drone_ctrl.open_logfile(7, 'ctrl_log.csv')`\n");
    uart_puts("`drone_ctrl.open_logfile(8, 'timing_log.csv')`\n");
    //for (idx = 0; idx < 512; idx++) {
    for (idx = 0; idx < 1024; idx++) {
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,%ld,')`\n", ctrl_log[idx][0],  ctrl_log[idx][1],  ctrl_log[idx][2], ctrl_log[idx][3]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,')`\n", ctrl_log[idx][4],  ctrl_log[idx][5], ctrl_log[idx][6]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,')`\n", ctrl_log[idx][7], ctrl_log[idx][8], ctrl_log[idx][9]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,')`\n", ctrl_log[idx][10], ctrl_log[idx][11], ctrl_log[idx][12]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,%ld,')`\n", ctrl_log[idx][13], ctrl_log[idx][14], ctrl_log[idx][15], ctrl_log[idx][16]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,')`\n", ctrl_log[idx][17], ctrl_log[idx][18], ctrl_log[idx][19]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld,')`\n", ctrl_log[idx][20], ctrl_log[idx][21], ctrl_log[idx][22]);
        uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld\\n')`\n", ctrl_log[idx][23], ctrl_log[idx][24], ctrl_log[idx][25]);
        //uart_printf("`drone_ctrl.log_write(7, '%ld,%ld,%ld\\n')`\n", ctrl_log[idx][17], ctrl_log[idx][18], ctrl_log[idx][19]);
        uart_printf("`drone_ctrl.log_write(8, '%ld,%ld\\n')`\n", ctrl_log[idx][26], ctrl_log[idx][27]);
    }

    uart_puts("Done, terminating\n");
    return 0;
}

static volatile _SPM int *imu_ctrl_ptr   = (volatile _SPM int *) 0xF00D0000;
static volatile _SPM int *imu_status_ptr = (volatile _SPM int *) 0xF00D0004;
#define IMU_GYRO_X (*((volatile _SPM int *) 0xF00D0008))
#define IMU_GYRO_Y (*((volatile _SPM int *) 0xF00D000C))
#define IMU_GYRO_Z (*((volatile _SPM int *) 0xF00D0010))
#define IMU_ACCEL_X (*((volatile _SPM int *) 0xF00D0014))
#define IMU_ACCEL_Y (*((volatile _SPM int *) 0xF00D0018))
#define IMU_ACCEL_Z (*((volatile _SPM int *) 0xF00D001C))

// Factor for converting degrees to radiants:
//#define DEG2RAD         0x477D1A89  // pi/180 with a scaling factor of 2^36
//#define DEG2RAD_SCALE   36

void get_imu_data(int32_t accel[3], int32_t omega[3])
{
    //t_start = *timer_cycles_low_ptr;

    int32_t a_x, a_y, a_z, // acceleration vector from accel measurements
            g_x, g_y, g_z; // angular rate vector from gyro measurements

    while (imu_poll_accel(&a_x, &a_y, &a_z) < 0 && *imu_status_ptr == 0)
        ;
    while (imu_poll_gyro(&g_x, &g_y, &g_z) < 0 && *imu_status_ptr == 0)
        ;

    //t_data = *timer_cycles_low_ptr;

    // rotate IMU data 180 deg about x axis (i.e. invert y and z axis):
    //a_y = -a_y; a_z = -a_z;
    //g_y = -g_y; g_z = -g_z;

    // rotate IMU data 180 deg about y axis (i.e. invert x and z axis):
    a_x = -a_x; a_z = -a_z;
    g_x = -g_x; g_z = -g_z;

    //uart_printf("\nNEW DATA: a = ( %6ld, %6ld, %6ld ) ; ", a_x, a_y, a_z);
    //uart_printf("g = `np.array([ %8ld, %8ld, %8ld ]) * .07` dps ", g_x, g_y, g_z);

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
        g_x = fix_mul(g_x, lsb2rad, 40 - 25);
        g_y = fix_mul(g_y, lsb2rad, 40 - 25);
        g_z = fix_mul(g_z, lsb2rad, 40 - 25);
    }

    //DEBUG("= ", DBG_FIXVEC(g_x, g_y, g_z, 25), " rad / s\n");

    //uart_printf("a = ( %8lX, %8lX, %8lX ) ; g = ( %8lX, %8lX, %8lX )\n",
    //uart_printf("a = ( %6ld, %6ld, %6ld ) ; g = ( %6ld, %6ld, %6ld )\n",
    //            a_x, a_y, a_z, g_x, g_y, g_z);

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
            //static uint8_t uart_buf[12];
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

        /*
        static uint8_t uart_buf[16];
        static int pos = 0;

        int val;
        while (pos < sizeof(uart_buf) && (val = uart_try_getc()) >= 0)
            uart_buf[pos++] = val;

        if (pos == sizeof(uart_buf)) {
            uint32_t magic = *((uint32_t *)uart_buf);
            if (magic != 0x55aa55aa)
                uart_puts("ERROR: sync lost\n");
            else {
                int32_t xi_meas[3] = {
                    *((int32_t *)(uart_buf + 4)),
                    *((int32_t *)(uart_buf + 8)),
                    *((int32_t *)(uart_buf + 12))
                };
                kalman_update(xi_meas, xi_est, d_xi_est);
                //xi_est[0] = xi_meas[0]; xi_est[1] = xi_meas[1]; xi_est[2] = xi_meas[2];
            }
            pos = 0;
        }
        */
    }

    return 0;
}

void control(const int32_t R_W[3][3], const int32_t omega[3],
             const int32_t xi_est[3], const int32_t d_xi_est[3],
             int cnt, int32_t dd_xi_ctrl[3])
{
    int32_t target_height = 1 << (XI_SCALE - 3); // 0.125 meter
    //int32_t target_height = 1 << (XI_SCALE - 4); // 0.0625 meter
    //int32_t target_height = 1 << (XI_SCALE - 5); // 0.03125 meter
    //int32_t target_height = 0;
    //if (2048 <= cnt && cnt < 3072)
        //target_height = 1 << (XI_SCALE - 4); // 0.0625 meter
        //target_height = 1 << (XI_SCALE - 3); // 0.125 meter

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
        int32_t q0, q1, q2, q3;
        ahrs_get_attitude(&q0, &q1, &q2, &q3);

        ctrl_log[cnt][0]  = q0;
        ctrl_log[cnt][1]  = q1;
        ctrl_log[cnt][2]  = q2;
        ctrl_log[cnt][3]  = q3;
        ctrl_log[cnt][4]  = omega[0];
        ctrl_log[cnt][5]  = omega[1];
        ctrl_log[cnt][6]  = omega[2];
        ctrl_log[cnt][7]  = xi_err[0];
        ctrl_log[cnt][8]  = xi_err[1];
        ctrl_log[cnt][9]  = xi_err[2];
        ctrl_log[cnt][10] = d_xi_err[0];
        ctrl_log[cnt][11] = d_xi_err[1];
        ctrl_log[cnt][12] = d_xi_err[2];
        ctrl_log[cnt][13] = f_prop[0];
        ctrl_log[cnt][14] = f_prop[1];
        ctrl_log[cnt][15] = f_prop[2];
        ctrl_log[cnt][16] = f_prop[3];
    }

    /*
    if ((cnt & 0xFF) == 0) {
        DEBUG("XI_EST = ", DBG_FIXVEC(xi_est[0], xi_est[1], xi_est[2], XI_SCALE), "\n");
        DEBUG("F_PROP = ", DBG_FIXQUAT(f_prop[0], f_prop[1], f_prop[2], f_prop[3], F_PROP_SCALE), "\n");
    }
    */

    {
        //uint32_t motor_pwm_out[4] = { 53500, 53500, 53500, 53500 };
        //uint32_t motor_pwm_out[4] = { 55000, 55000, 55000, 55000 };
        uint32_t motor_pwm_out[4] = { 57000, 57000, 57000, 57000 };
        //motor_pwm_out[0] += fix_mul(6000, f_prop[0], F_PROP_SCALE);
        //motor_pwm_out[1] += fix_mul(6000, f_prop[1], F_PROP_SCALE);
        //motor_pwm_out[2] += fix_mul(6000, f_prop[2], F_PROP_SCALE);
        //motor_pwm_out[3] += fix_mul(6000, f_prop[3], F_PROP_SCALE);

        int idx;
        for (idx = 0; idx < 4; idx++) {
            //if (f_prop[idx] >= 0)
                //motor_pwm_out[idx] += fix_mul(6000, f_prop[idx], F_PROP_SCALE);
                //motor_pwm_out[idx] += fix_mul(6230, f_prop[idx], F_PROP_SCALE);

                //motor_pwm_out[idx] += fix_mul(5700, f_prop[idx], F_PROP_SCALE);
                //motor_pwm_out[idx] += fix_mul(5500, f_prop[idx], F_PROP_SCALE);
                //motor_pwm_out[idx] += fix_mul(5200, f_prop[idx], F_PROP_SCALE);

            //if (motor_pwm_out[idx] > 107000)
                //motor_pwm_out[idx] = 107000;

            int32_t pwm = 57000 + fix_mul(5200, f_prop[idx], F_PROP_SCALE);
            if (pwm < 53500)
                pwm = 53500;
            if (pwm > 107000)
                pwm = 107000;
            motor_pwm_out[idx] = pwm;
        }

        //motor_out(motor_pwm_out);
    }
}

void update_gui_compact(const int32_t pos[3], const int32_t vel[3])
{
    int32_t q0, q1, q2, q3;
    ahrs_get_attitude(&q0, &q1, &q2, &q3);

    uint32_t vals[10] = {
        (uint32_t)q0, (uint32_t)q1, (uint32_t)q2, (uint32_t)q3,
        (uint32_t)pos[0], (uint32_t)pos[1], (uint32_t)pos[2],
        (uint32_t)vel[0], (uint32_t)vel[1], (uint32_t)vel[2],
    };

    uart_printf("`drone_ctrl.ug('%08lX%08lX%08lX%08lX"
                                "%08lX%08lX%08lX"
                                "%08lX%08lX%08lX')`\n",
                vals[0], vals[1], vals[2], vals[3],
                vals[4], vals[5], vals[6],
                vals[7], vals[8], vals[9]);
}

/*
void update_gui(const int32_t pos[3])
{
    int32_t q0, q1, q2, q3;
    ahrs_get_attitude(&q0, &q1, &q2, &q3);

    uart_puts("`drone_ctrl.update_gui(");
    // Roll:
    {
        int32_t q0q1_q2q3 = fix_mul(q0, q1, 31) + fix_mul(q2, q3, 31); // scaling factor: 2^29
        int32_t q1sq_q2sq = fix_mul(q1, q1, 31) + fix_mul(q2, q2, 31); // scaling factor: 2^29
        // intermediate results are both multiplied by 2;
        // done implicitly by halving scaling factor to 2^28:
        uart_printf("np.arctan2(%8ld/(2**28), 1 - (%8ld/(2**28))), ", q0q1_q2q3, q1sq_q2sq);
    }

    // Pitch:
    {
        int32_t q0q2_q3q1 = fix_mul(q0, q2, 31) - fix_mul(q3, q1, 31); // scaling factor: 2^29
        uart_printf("np.arcsin(%8ld/(2**28)), ", q0q2_q3q1);
    }

    // Yaw:
    {
        int32_t q0q3_q1q2 = fix_mul(q0, q3, 31) + fix_mul(q1, q2, 31); // scaling factor: 2^29
        int32_t q2sq_q3sq = fix_mul(q2, q2, 31) + fix_mul(q3, q3, 31); // scaling factor: 2^29
        uart_printf("np.arctan2(%8ld/(2**28), 1 - (%8ld/(2**28))), ", q0q3_q1q2, q2sq_q3sq);
    }

    uart_printf("%8ld/(2**25), %8ld/(2**25), %8ld/(2**25))`\n", pos[0], pos[1], pos[2])

    //t_end = *timer_cycles_low_ptr;
    //uart_printf("update %d: start %lu, data %lu, conv %lu, update %lu, now %lu\n", idx, t_start, t_data, t_conv, t_update, t_end);

    //uart_printf("`drone_ctrl.update(np.arctan2(2 * ((%8ld/(2**30)) * (%8ld/(2**30)) + (%8ld/(2**30)) * (%8ld/(2**30))), 1 - 2 * ((%8ld/(2**30))**2 + (%8ld/(2**30))**2)),", q0, q1, q2, q3, q1, q2);
    //uart_printf(" np.arcsin(2 * ((%8ld/(2**30)) * (%8ld/(2**30)) - (%8ld/(2**30)) * (%8ld/(2**30)))),", q0, q2, q3, q1);
    //uart_printf(" np.arctan2(2 * ((%8ld/(2**30)) * (%8ld/(2**30)) + (%8ld/(2**30)) * (%8ld/(2**30))), 1 - 2 * ((%8ld/(2**30))**2 + (%8ld/(2**30))**2)))`\n", q0, q3, q1, q2, q2, q3);
}
*/

void motor_test(void)
{
    //uint32_t motor_pwm_out[4] = { 107000, 107000, 107000, 107000 };
    uint32_t motor_pwm_out[4] = { 53500, 53500, 53500, 53500 };
    //uint32_t motor_pwm_out[4] = { 160500, 160500, 160500, 160500 };
    motor_out(motor_pwm_out);

    int idx;
    for (idx = 0; idx < 4; idx++) {
        // wait
        {
            int i, j;
            for (i = 8000; i != 0; --i)
                for (j = 8000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
        }

        uart_puts("Throttling up on motor\n");
        motor_pwm_out[idx] += 5000;
        {
            int i, j;
            for (i = 15000; i != 0; --i) {
                motor_pwm_out[idx] += 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }
        uart_puts("Throttling down on motor\n");
        {
            int i, j;
            for (i = 15000; i != 0; --i) {
                motor_pwm_out[idx] -= 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }

        motor_pwm_out[0] = 53500;
        motor_pwm_out[1] = 53500;
        motor_pwm_out[2] = 53500;
        motor_pwm_out[3] = 53500;
        motor_out(motor_pwm_out);
    }


    motor_pwm_out[0] = 0;
    motor_pwm_out[1] = 0;
    motor_pwm_out[2] = 0;
    motor_pwm_out[3] = 0;
    motor_out(motor_pwm_out);
    return;


    while (1) {
        /*
        {
            int i, j;
            for (i = 8000; i != 0; --i)
                for (j = 8000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
        }

        motor_pwm_out[2] = 53500;
        motor_out(motor_pwm_out);

        {
            int i, j;
            for (i = 8000; i != 0; --i)
                for (j = 8000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
        }
        */

        {
            int i, j;
            for (i = 8000; i != 0; --i)
                for (j = 8000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
        }

        uart_puts("Throttling up on motor 0\n");

        {
            int i, j;
            for (i = 20000; i != 0; --i) {
                motor_pwm_out[0] += 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }

        uart_puts("Throttling down on motor 0, up on motor 1\n");

        {
            int i, j;
            for (i = 20000; i != 0; --i) {
                motor_pwm_out[0] -= 1;
                motor_pwm_out[1] += 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }

        uart_puts("Throttling down on motor 1, up on motor 2\n");

        {
            int i, j;
            for (i = 20000; i != 0; --i) {
                motor_pwm_out[1] -= 1;
                motor_pwm_out[2] += 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }

        uart_puts("Throttling down on motor 2, up on motor 3\n");

        {
            int i, j;
            for (i = 20000; i != 0; --i) {
                motor_pwm_out[2] -= 1;
                motor_pwm_out[3] += 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }

        uart_puts("Throttling down on motor 3\n");

        {
            int i, j;
            for (i = 20000; i != 0; --i) {
                motor_pwm_out[3] -= 1;
                motor_out(motor_pwm_out);

                for (j = 2000; j != 0; --j)
                    *motor_0 = motor_pwm_out[0];
            }
        }

        motor_pwm_out[0] = 53500;
        motor_pwm_out[1] = 53500;
        motor_pwm_out[2] = 53500;
        motor_pwm_out[3] = 53500;
        motor_out(motor_pwm_out);
    }
}
