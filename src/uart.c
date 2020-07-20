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


#include "uart.h"

#include <machine/spm.h>

volatile _SPM int *uart_ptr = (volatile _SPM int *) 0xF0080004;
volatile _SPM int *uart_stat = (volatile _SPM int *) 0xF0080000;

void uart_putc(char c)
{
    while (((*uart_stat) & 0x01) == 0);
    *uart_ptr = c;
}

//void uart_puth(uint8_t val)
//{
    //int tmp;
    //tmp = (val >> 4) & 0xf;
    //while (((*uart_stat) & 0x01) == 0);
    //*uart_ptr = (tmp < 10) ? tmp + '0' : tmp - 10 + 'A';
    //tmp = val & 0xf;
    //while (((*uart_stat) & 0x01) == 0);
    //*uart_ptr = (tmp < 10) ? tmp + '0' : tmp - 10 + 'A';
//}

void uart_puts(const char *str)
{
    const char *p;
    for (p = str; *p != 0; p++) {
        while (((*uart_stat) & 0x01) == 0);
        *uart_ptr = *p;
    }
}

int uart_getc(void)
{
    while (((*uart_stat) & 0x02) == 0);
    return *uart_ptr;
}

int uart_try_getc(void)
{
    if (((*uart_stat) & 0x02) == 0)
        return -1;
    return *uart_ptr;
}
