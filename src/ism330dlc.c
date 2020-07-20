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


#include "ism330dlc.h"

#include "i2c_master.h"
//#include "uart.h"

//#define DEBUG(fmt, ...) {                                                     \
//    uart_printf("[%s:%d] " fmt, __FILE__, __LINE__, ##__VA_ARGS__);           \
//}


// register definitions for the ISM330DLC

#define DEV_ADDR        0x6A

#define REG_INT1_CTRL   0x0D
#define REG_INT2_CTRL   0x0E
#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11

#define REG_CTRL6_C     0x15
#define REG_CTRL7_G     0x16

#define REG_STATUS      0x1E
#define REG_OUT_TEMP    0x20    // temperature data output value (16 bit)
#define REG_OUTX_G      0x22    // gyro x-axis (pitch) output value (16 bit)
#define REG_OUTY_G      0x24    // gyro y-axis (roll) output value (16 bit)
#define REG_OUTZ_G      0x26    // gyro z-axis (yaw) output value (16 bit)
#define REG_OUTX_XL     0x28    // accel x-axis output value (16 bit)
#define REG_OUTY_XL     0x2A    // accel y-axis output value (16 bit)
#define REG_OUTZ_XL     0x2C    // accel z-axis output value (16 bit)


//#define REG_WR(dev, reg, data)  {                                             \
//    DEBUG("writing register 0x%04x (" #reg "), value: 0x%04x\n", reg, data);  \
//    if (i2c_reg8_write8(dev, reg, data) < 0) return -1;                       \
//}
#define REG_WR(dev, reg, data)  {                                             \
    if (i2c_reg8_write8(dev, reg, data) < 0) return -1;                       \
}


int imu_init()
{
    if (i2c_probe(DEV_ADDR) < 0) {
        DEBUG("no device with address 0x%02X\n", DEV_ADDR);
        return -1;
    }

    int data;
    if ((data = i2c_reg8_read8(DEV_ADDR, REG_WHO_AM_I)) != 0x6A) {
        DEBUG("whoami register reads 0x%02X, expected 0x6A\n", data);
        return -1;
    }

    //REG_WR(DEV_ADDR, REG_INT1_CTRL, 0x03); // trigger int1 on gyro or accel dat

    //REG_WR(DEV_ADDR, , );
    //REG_WR(DEV_ADDR, REG_CTRL6_C, 0x00);    // use high perf mode
    //REG_WR(DEV_ADDR, REG_CTRL1_XL, 0x1C);   // ODR: 12.5 Hz, range: +-8 g
    REG_WR(DEV_ADDR, REG_CTRL1_XL, 0x5C);   // ODR: 208 Hz, range: +-8 g


    //REG_WR(DEV_ADDR, , );
    //REG_WR(DEV_ADDR, REG_CTRL7_G, 0x00);    // use high perf mode
    //REG_WR(DEV_ADDR, REG_CTRL2_G, 0x1C);    // ODR: 12.5 Hz, range: +-2000 dps
    REG_WR(DEV_ADDR, REG_CTRL2_G, 0x5C);    // ODR: 208 Hz, range: +-2000 dps

    return 0;
}

int imu_poll_accel(int32_t *a_x, int32_t *a_y, int32_t *a_z)
{
    int status = i2c_reg8_read8(DEV_ADDR, REG_STATUS);
    if (status < 0 || (status & 1) == 0)
        return -1;

    uint8_t buf[6];
    if (i2c_reg8_read_stream(DEV_ADDR, REG_OUTX_XL, buf, 6) < 0)
        return -1;
    *a_x = (int16_t)(((int)buf[0]) | (((int)buf[1]) << 8));
    *a_y = (int16_t)(((int)buf[2]) | (((int)buf[3]) << 8));
    *a_z = (int16_t)(((int)buf[4]) | (((int)buf[5]) << 8));
    return 0;
}

int imu_poll_gyro(int32_t *g_x, int32_t *g_y, int32_t *g_z)
{
    int status = i2c_reg8_read8(DEV_ADDR, REG_STATUS);
    if (status < 0 || (status & 2) == 0)
        return -1;

    uint8_t buf[6];
    if (i2c_reg8_read_stream(DEV_ADDR, REG_OUTX_G, buf, 6) < 0)
        return -1;
    *g_x = (int16_t)(((int)buf[0]) | (((int)buf[1]) << 8));
    *g_y = (int16_t)(((int)buf[2]) | (((int)buf[3]) << 8));
    *g_z = (int16_t)(((int)buf[4]) | (((int)buf[5]) << 8));
    return 0;
}
