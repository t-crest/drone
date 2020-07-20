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
 * Convenience functions for writing debug information to UART.
 */


#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "uart.h"

#define DEBUG_PRINT(str)    uart_puts(str);

struct debug_fix{
    uint16_t magic;
    int scale;
    int32_t v[4];
};
#define DEBUG_MAGIC_FIX     0x0100
#define DEBUG_MAGIC_VECT    0x0200
#define DEBUG_MAGIC_QUAT    0x0300

#define DBG_FIX(val, scale)                                                    \
    (&((struct debug_fix){ DEBUG_MAGIC_FIX, scale, { val } }))
#define DBG_FIXVEC(v1, v2, v3, scale)                                          \
    (&((struct debug_fix){ DEBUG_MAGIC_VECT, scale, { v1, v2, v3 } }))
#define DBG_FIXQUAT(q1, q2, q3, q4, scale)                                     \
    (&((struct debug_fix){ DEBUG_MAGIC_QUAT, scale, { q1, q2, q3, q4 } }))

static void debug_print(int guard, ...)
{
    char p_str[256];
    size_t p_pos = 0;
    struct debug_fix *fx;
    va_list ap;
    va_start(ap, guard);
    while (1) {
        void *arg = va_arg(ap, void *);
        switch (*((uint16_t *)arg)) {
            case 0:
                DEBUG_PRINT(p_str);
                return;
            case DEBUG_MAGIC_FIX:
                fx = (struct debug_fix *)arg;
                p_pos += snprintf(p_str + p_pos, sizeof(p_str) - p_pos,
                         "`%8ld / (2**%d)`", fx->v[0], fx->scale);
                break;
            case DEBUG_MAGIC_VECT:
                fx = (struct debug_fix *)arg;
                p_pos += snprintf(p_str + p_pos, sizeof(p_str) - p_pos,
                         //"`np.array([ 0x%08lX, 0x%08lX, 0x%08lX ]) / (2**%d)`",
                         "`np.array([ %8ld, %8ld, %8ld ]) / (2**%d)`",
                         fx->v[0], fx->v[1], fx->v[2], fx->scale);
                break;
            case DEBUG_MAGIC_QUAT:
                fx = (struct debug_fix *)arg;
                p_pos += snprintf(p_str + p_pos, sizeof(p_str) - p_pos,
                         //"`[ 0x%08lX, 0x%08lX, 0x%08lX, 0x%08lX ] / (2**%d)`",
                         "`np.array([ %8ld, %8ld, %8ld, %8ld ]) / (2**%d)`",
                         fx->v[0], fx->v[1], fx->v[2], fx->v[3], fx->scale);
                break;
            default:
                p_pos += snprintf(p_str + p_pos, sizeof(p_str) - p_pos,
                                  "%s", (char *)arg);
        }
    }
}

/**
 * Print debug information; accepts a comma-separated list of strings and/or
 * the macros DBG_FIX, DBG_FIXVEC and DBG_FIXQUAT to print a fixed point
 * number, a vector of fixed point numbers or a quaternion respectively.
 *
 * ATTENTION: Each string passed to this macro must be at least 2 bytes long,
 *            i.e. at least one character and a terminating 0 byte.
 */
#define DEBUG(...) debug_print(0, __VA_ARGS__, "\x00")


// https://stackoverflow.com/questions/45756920/c-c-preprocessor-extract-every-second-variadic-parameter

#endif // DEBUG_H
