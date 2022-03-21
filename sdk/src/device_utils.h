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
#ifndef DEVICE_UTILS_H
#define DEVICE_UTILS_H

#include <assert.h>
#include <cstddef>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <inttypes.h>

namespace aditof {
static void deinterleave(const char *source, uint16_t *destination,
                         size_t source_len, size_t dest_width,
                         size_t dest_height) {

    if (source == nullptr) {
        LOG(ERROR) << "Received source null pointer";
        return;
    }

    if (destination == nullptr) {
        LOG(ERROR) << "Received destination null pointer";
        return;
    }
    assert(source_len > 0);

    size_t offset[2] = {0, dest_height * dest_width / 2};
    size_t offset_idx = 0;
    size_t j = 0;

    if (dest_width == 668) {
        for (size_t i = 0; i < source_len; i += 3) {
            if ((i != 0) & (i % (336 * 3) == 0)) {
                j -= 4;
            }

            destination[j] =
                (((unsigned short)*(((unsigned char *)source) + i)) << 4) |
                (((unsigned short)*(((unsigned char *)source) + i + 2)) &
                 0x000F);
            j++;

            destination[j] =
                (((unsigned short)*(((unsigned char *)source) + i + 1)) << 4) |
                ((((unsigned short)*(((unsigned char *)source) + i + 2)) &
                  0x00F0) >>
                 4);
            j++;
        }
    } else {
        for (size_t i = 0; i < source_len; i += 3) {

            offset_idx = ((j / dest_width) % 2);

            destination[offset[offset_idx]] =
                (((unsigned short)*(((unsigned char *)source) + i)) << 4) |
                (((unsigned short)*(((unsigned char *)source) + i + 2)) &
                 0x000F);
            offset[offset_idx]++;

            destination[offset[offset_idx]] =
                (((unsigned short)*(((unsigned char *)source) + i + 1)) << 4) |
                ((((unsigned short)*(((unsigned char *)source) + i + 2)) &
                  0x00F0) >>
                 4);
            offset[offset_idx]++;

            j += 2;
        }
    }
}
} // namespace aditof

#endif // DEVICE_UTILS_H
