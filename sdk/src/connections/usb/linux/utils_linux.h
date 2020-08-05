/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef UTILS_LINUX_H
#define UTILS_LINUX_H

#include <errno.h>
#include <sys/ioctl.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define MAX_PACKET_SIZE (58)
#define MAX_BUF_SIZE (MAX_PACKET_SIZE + 2)

#ifdef DEBUG_USB
#include <iostream>
#include <linux/uvcvideo.h>
#include <string.h>
#endif

static int xioctl(int fh, unsigned long request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
        // printf("Error=%d\n",errno);
    } while (-1 == r && (EINTR == errno || EIO == errno) && errno != 0);

#ifdef DEBUG_USB
    if (request == (int)UVCIOC_CTRL_QUERY) {
        static int count = 0;
        struct uvc_xu_control_query *cq = (struct uvc_xu_control_query *)arg;
        count++;
        printf("%d Calling IOCTL with bRequest %02x, wValue %04x, wIndex %04x, "
               "wLength %04x\n",
               count, cq->query, cq->selector, cq->unit, cq->size);
        if (r != 0) {
            printf("Return values: %d \n", r);
            printf("IOCTL failed, error num: %d, %s\n", errno, strerror(errno));
        }
    }
#endif
    return r;
}

#endif // UTILS_LINUX_H
