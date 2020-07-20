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


#include "raffo.h"
#include "raffo_params.h"

//#include "debug.h"
//#include "uart.h"

// trigger a python exception in case of multiplication overflow:
//#define FIX_MATH_MUL_OVERFLOW_ACTION(val)                                      \
    //uart_printf("`raise ValueError('FIX_MUL OVERFLOW: %08llX %08llX')`\n",     \
                //val >> 32, (val) & 0xFFFFFFFFLL)
//#define FIX_MATH_MUL_OVERFLOW_ACTION(val)                                      \
//    uart_puts("FIX_MUL OVERFLOW\n")
    //uart_puts("`raise ValueError('FIX_MUL OVERFLOW')`\n")

#include "fix32math.h"

#define DELTA_T         0x4EC4EC4F  // 1/208 with a scaling factor of 2^38
//#define DELTA_T         0x51EB851F  // 1/200 with a scaling factor of 2^38
#define DELTA_T_SCALE   38

// weighted sum:
#define WGSUM_3(v1, w1, v2, w2, v3, w3, scale)                                 \
    (fix_mul(v1, w1, scale) + fix_mul(v2, w2, scale) + fix_mul(v3, w3, scale))
//#define WGSUM_6(v1, w1, v2, w2, v3, w3, v4, w4, v5, w5, v6, w6, n)             \
//    (WGSUM_3(v1, w1, v2, w2, v3, w3, n) + WGSUM_3(v4, w4, v5, w5, v6, w6, n))

//#include "uart.h"
//#define DEBUG(fmt, ...) {                                                      \
//    uart_printf("[%s:%d] " fmt, __FILE__, __LINE__, ##__VA_ARGS__);            \
//}

/*
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }
*/
//#define SHIFT_CHECK(VAL)    ct_assert(VAL >= 31)
//#define SHIFT_CHECK(VAL)    const int checkval_ ## VAL = VAL;

//#define SHIFT_CHECK(VAL)    DEBUG("shift value " #VAL " shifted by %d\n", VAL)
#define SHIFT_CHECK(VAL)

/**
 * Update the desired target acceleration and corresponding propeller thrusts
 * based on a new state estimate.
 *
 * The orientation is given by a rotation matrix (from body to world frame) and
 * the angular rates are relative to the body frame. Only the desired yaw angle
 * and yaw rate can be specified, as pitch and roll are coupled to translation.
 * Position, velocity and acceleration are in world frame coordinates. For
 * position and velocity, the state estimate and desired values are specified
 * as a combined error vector.
 */
void raffo_update(
    const int32_t R_W[3][3],    // rotation matrix from body to world frame
    const int32_t omega[3],     // angular rate (in body frame coordinates)
    const int32_t xi_err[3],    // position error vector (in world frame coord)
    const int32_t d_xi_err[3],  // velocity error vector ( -"- )
    const int32_t dd_xi_des[3], // desired acceleration vector ( -"- )
    int32_t yaw_des,            // desired yaw angle, i.e. heading ( -"- )
    int32_t d_yaw_des,          // desired derivative of yaw angle ( -"- )
    //int32_t dd_xi_ctrl[3],      // output vector for target acceleration ( -"-)
    int32_t f_prop[4]           // output vector for propeller thrusts
)
{
    // Integrate the position error vector over time; the scaling factor must
    // be choosen such that overflowing any of the integral values is avoided,
    // even if a large error persists for several seconds; thus we reduce the
    // scaling factor of XI_SCALE by 2^5 (i.e. 32), which prevents overflow
    // even if extreme values of xi persist for 30 seconds:
#define I_XI_SCALE  (XI_SCALE - 5)
#define I_XI_SHIFT  (XI_SCALE + DELTA_T_SCALE - I_XI_SCALE)
    static int32_t i_xi_err[3] = { 0, 0, 0 };
    i_xi_err[0] += fix_mul(xi_err[0], DELTA_T, I_XI_SHIFT);
    i_xi_err[1] += fix_mul(xi_err[1], DELTA_T, I_XI_SHIFT);
    i_xi_err[2] += fix_mul(xi_err[2], DELTA_T, I_XI_SHIFT);

    //DEBUG("i_xi     = ", DBG_FIXVEC(i_xi_err[0], i_xi_err[1], i_xi_err[2], I_XI_SCALE), "\n");

    // Calculate the yaw angle (resp. the yaw error) from the rotation matrix;
    // yaw angle has a scaling factor of 2^28 (as returned by atan2 function):
    int32_t yaw_err = fix_atan2(R_W[1][0], R_W[0][0], R_W_SCALE) - yaw_des;
    //int32_t yaw_err = fix_atan2(R_W[2][1], R_W[2][2], R_W_SCALE) - yaw_des;

    // TODO: integrate the yaw error as well
#define I_YAW_SCALE

    // Angular rates often appear in pairs (e.g. in KD * dq & Coriolis matrix),
    // calculating each pair here to avoid redundant multiplications;
    // results are shifted by 31 bits to avoid overflow, thus the resulting
    // pairs have a scaling factor of 2 * OMEGA_SCALE - 31:
    int32_t pq = fix_mul(omega[0], omega[1], 31),
            pr = fix_mul(omega[0], omega[2], 31),
            qr = fix_mul(omega[1], omega[2], 31);
#define OMEGA_SQ_SCALE  (2 * (OMEGA_SCALE) - 31)

    ////////////////////////////////////////////////////////////////////////////
    // CONTROL ACCELERATION:

    // The scaling factor of the control acceleration is set such that overflow
    // is avoided when summing the individual values.
//#define CTRL_ACC_ROT_SCALE  22  // allow values in the range [-500,500]
#define CTRL_ACC_ROT_SCALE  21  // allow values in the range [-1000,1000]
#define CTRL_ACC_XI_SCALE   (DD_XI_SCALE)

    // Result of multiplying the integral gain matrix KI with the integral of
    // the position and orientation error vector:
#define KI_ROT_SHFT (KI_ROT_SCALE + I_YAW_SCALE - CTRL_ACC_ROT_SCALE)
#define KI_XI_SHFT  (KI_XI_SCALE + I_XI_SCALE - CTRL_ACC_XI_SCALE)
    SHIFT_CHECK(KI_ROT_SHFT); SHIFT_CHECK(KI_XI_SHFT);
    int32_t KI_q[6] = {
        0, // fix_mul(fix_mul(i_yaw_err, omega[1], ), KI_P, KI_ROT_SHFT),
        0, // fix_mul(fix_mul(i_yaw_err, omega[0], ), KI_Q, KI_ROT_SHFT),
        0, // fix_mul(i_yaw_err, KI_R, KI_ROT_SHFT),
        fix_mul(KI_XI, i_xi_err[0], KI_XI_SHFT),
        fix_mul(KI_XI, i_xi_err[1], KI_XI_SHFT),
        fix_mul(KI_XI, i_xi_err[2], KI_XI_SHFT)
    };

    //DEBUG("KI*i_xi  = ", DBG_FIXVEC(KI_q[3], KI_q[4], KI_q[5], CTRL_ACC_XI_SCALE), "\n");

    // Result of multiplying the proportional gain matrix KP with the position
    // and orientation error vector:
#define KP_ROT_SHFT (KP_ROT_SCALE + YAW_SCALE - CTRL_ACC_ROT_SCALE)
#define KP_XI_SHFT  (KP_XI_SCALE + XI_SCALE - CTRL_ACC_XI_SCALE)
    SHIFT_CHECK(KP_ROT_SHFT); SHIFT_CHECK(KP_XI_SHFT);
    int32_t KP_q[6] = {
        0, // fix_mul(fix_mul(yaw_err, omega[1], ), KP_P, KP_ROT_SHFT),
        0, // fix_mul(fix_mul(yaw_err, omega[0], ), KP_Q, KP_ROT_SHFT),
        fix_mul(yaw_err, KP_R, KP_ROT_SHFT),
        fix_mul(KP_XI, xi_err[0], KP_XI_SHFT),
        fix_mul(KP_XI, xi_err[1], KP_XI_SHFT),
        fix_mul(KP_XI, xi_err[2], KP_XI_SHFT)
    };

    //DEBUG("KP*xi    = ", DBG_FIXVEC(KP_q[3], KP_q[4], KP_q[5], CTRL_ACC_XI_SCALE), "\n");

    // Result of multiplying the derivative gain matrix KD with the velocity
    // and angular rate error vector:
#define KD_R1_SHFT  (KD_ROT1_SCALE + OMEGA_SCALE - CTRL_ACC_ROT_SCALE)
#define KD_R2_SHFT  (KD_ROT2_SCALE + OMEGA_SQ_SCALE - CTRL_ACC_ROT_SCALE)
#define KD_XI_SHFT  (KD_XI_SCALE + D_XI_SCALE - CTRL_ACC_XI_SCALE)
    SHIFT_CHECK(KD_R1_SHFT); SHIFT_CHECK(KD_R2_SHFT); SHIFT_CHECK(KD_XI_SHFT);
    int32_t KD_q[6] = {
        fix_mul(KD_PP, omega[0], KD_R1_SHFT) + fix_mul(KD_PQR, qr, KD_R2_SHFT),
        fix_mul(KD_QQ, omega[1], KD_R1_SHFT) + fix_mul(KD_QPR, pr, KD_R2_SHFT),
        fix_mul(KD_RR, omega[2], KD_R1_SHFT) + fix_mul(KD_RPQ, pq, KD_R2_SHFT),
        fix_mul(KD_XI, d_xi_err[0], KD_XI_SHFT),
        fix_mul(KD_XI, d_xi_err[1], KD_XI_SHFT),
        fix_mul(KD_XI, d_xi_err[2], KD_XI_SHFT)
    };
    // TODO: add desired yaw rate

    //DEBUG("KD*omega = ", DBG_FIXVEC(KD_q[0], KD_q[1], KD_q[2], CTRL_ACC_ROT_SCALE), "\n");
    //DEBUG("KD*d_xi  = ", DBG_FIXVEC(KD_q[3], KD_q[4], KD_q[5], CTRL_ACC_XI_SCALE), "\n");

    // Control acceleration from desired acceleration and gain matrices:
    int32_t ctrl_accel[6] = {
        -KD_q[0] - KP_q[0] - KI_q[0],
        -KD_q[1] - KP_q[1] - KI_q[1],
        -KD_q[2] - KP_q[2] - KI_q[2],
        dd_xi_des[0] - KD_q[3] - KP_q[3] - KI_q[3],
        dd_xi_des[1] - KD_q[4] - KP_q[4] - KI_q[4],
        dd_xi_des[2] - KD_q[5] - KP_q[5] - KI_q[5],
    };
    //dd_xi_ctrl[0] = ctrl_accel[3];
    //dd_xi_ctrl[1] = ctrl_accel[4];
    //dd_xi_ctrl[2] = ctrl_accel[5];

    ////////////////////////////////////////////////////////////////////////////
    // CONTROL TORQUES AND FORCES:

    // Coriolis matrix multiplied by angular rate:
#define CORIO_SHFT  (I_SCALE + OMEGA_SQ_SCALE - GAMMA_TORQUE_SCALE)
    SHIFT_CHECK(CORIO_SHFT);
    int32_t Corio_omega[3] = {
        fix_mul(I_ZZ - I_YY, qr, CORIO_SHFT),
        fix_mul(I_XX - I_ZZ, pr, CORIO_SHFT),
        fix_mul(I_YY - I_XX, pq, CORIO_SHFT)
    };

    // Calculate Gamma, i.e. the desired torques and forces. Equivalent to the
    // system equations the torques are in body frame and the forces are in
    // world frame coordinates:
#define GAMMA_TORQUE_SHFT   (CTRL_ACC_ROT_SCALE + I_SCALE - GAMMA_TORQUE_SCALE)
#define GAMMA_FORCE_SHFT    (CTRL_ACC_XI_SCALE + MASS_SCALE - GAMMA_FORCE_SCALE)
    SHIFT_CHECK(GAMMA_TORQUE_SHFT); SHIFT_CHECK(GAMMA_FORCE_SHFT);
    int32_t Gamma[6] = {
        fix_mul(ctrl_accel[0], I_XX, GAMMA_TORQUE_SHFT) + Corio_omega[0],
        fix_mul(ctrl_accel[1], I_YY, GAMMA_TORQUE_SHFT) + Corio_omega[1],
        fix_mul(ctrl_accel[2], I_ZZ, GAMMA_TORQUE_SHFT) + Corio_omega[2],
        fix_mul(ctrl_accel[3], MASS, GAMMA_FORCE_SHFT),
        fix_mul(ctrl_accel[4], MASS, GAMMA_FORCE_SHFT),
        fix_mul(ctrl_accel[5], MASS, GAMMA_FORCE_SHFT) + G_MASS
    };

    // Transform the forces vector into body frame coordinates by multiplying
    // with the transpose of R_W; torques are already in body frame, thus we
    // obtain the combined torques and forces vector in body frame coordinates;
    // the multiplication results are shifted by R_W_SCALE bits
    // to keep the current scaling factor of GAMMA_XI_SCALE:
    int32_t TF[6] = {
        Gamma[0],
        Gamma[1],
        Gamma[2],
        WGSUM_3(Gamma[3], R_W[0][0], Gamma[4], R_W[1][0], Gamma[5], R_W[2][0],
                R_W_SCALE),
        WGSUM_3(Gamma[3], R_W[0][1], Gamma[4], R_W[1][1], Gamma[5], R_W[2][1],
                R_W_SCALE),
        WGSUM_3(Gamma[3], R_W[0][2], Gamma[4], R_W[1][2], Gamma[5], R_W[2][2],
                R_W_SCALE)
    };

    ////////////////////////////////////////////////////////////////////////////
    // PROPELLER THRUSTS:

    // Obtain the propeller thrusts by multiplying the torques and forces
    // vector with the pseudo-inverse of the input coupling matrix B;
    // the multiplication results are shifted such that the result has a
    // scaling factor of F_PROP_SCALE:
    {
        int shf_torque = GAMMA_TORQUE_SCALE + BPI_SCALE - F_PROP_SCALE,
            shf_force = GAMMA_FORCE_SCALE + BPI_SCALE - F_PROP_SCALE;
        SHIFT_CHECK(shf_torque); SHIFT_CHECK(shf_force);

        f_prop[0] = WGSUM_3(TF[0], Bpi[0][0], TF[1], Bpi[0][1],
                            TF[2], Bpi[0][2], shf_torque) +
                    WGSUM_3(TF[3], Bpi[0][3], TF[4], Bpi[0][4],
                            TF[5], Bpi[0][5], shf_force);
        f_prop[1] = WGSUM_3(TF[0], Bpi[1][0], TF[1], Bpi[1][1],
                            TF[2], Bpi[1][2], shf_torque) +
                    WGSUM_3(TF[3], Bpi[1][3], TF[4], Bpi[1][4],
                            TF[5], Bpi[1][5], shf_force);
        f_prop[2] = WGSUM_3(TF[0], Bpi[2][0], TF[1], Bpi[2][1],
                            TF[2], Bpi[2][2], shf_torque) +
                    WGSUM_3(TF[3], Bpi[2][3], TF[4], Bpi[2][4],
                            TF[5], Bpi[2][5], shf_force);
        f_prop[3] = WGSUM_3(TF[0], Bpi[3][0], TF[1], Bpi[3][1],
                            TF[2], Bpi[3][2], shf_torque) +
                    WGSUM_3(TF[3], Bpi[3][3], TF[4], Bpi[3][4],
                            TF[5], Bpi[3][5], shf_force);
    }
}
