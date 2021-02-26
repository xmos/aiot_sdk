/*
 * Copyright (c) 2001, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __ARCH_CC_H__
#define __ARCH_CC_H__

#include <stdint.h>
#include <xcore/assert.h>

#include "rtos_printf.h"

#define BYTE_ORDER LITTLE_ENDIAN

#define S16_F "d"
#define U16_F "u"
#define X16_F "x"

#define S32_F "d"
#define U32_F "u"
#define X32_F "x"

#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

/*
 * 1. load byte by byte, construct 16 bits word and add: not efficient for most platforms
 * 2. load first byte if odd address, loop processing 16 bits words, add last byte.
 * 3. load first byte and word if not 4 byte aligned, loop processing 32 bits words, add last word/byte.
 */
#define LWIP_CHKSUM_ALGORITHM 3

#define LWIP_PLATFORM_DIAG(x)           do {rtos_printf(x);} while(0)
#define LWIP_PLATFORM_ASSERT(message)   do {rtos_printf("LWIP Assertion at %d:%d:\n%s\n",__FILE__, __LINE__, message); xassert(0);} while(0)

#define LWIP_PROVIDE_ERRNO

#define LWIP_RAND()  (0)
#warning NEED TO DEFINE RAND

#endif /* __ARCH_CC_H__ */
