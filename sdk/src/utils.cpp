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
#include "utils.h"

uint16_t *aditof::Utils::buildCalibrationCache(float gain, float offset,
                                               int16_t maxPixelValue,
                                               int range) {
    uint16_t *cache = new uint16_t[maxPixelValue + 1];
    for (int16_t current = 0; current <= maxPixelValue; ++current) {
        int16_t currentValue =
            static_cast<int16_t>(static_cast<float>(current) * gain + offset);
        cache[current] = currentValue <= range ? currentValue : range;
    }
    return cache;
}

void aditof::Utils::calibrateFrame(uint16_t *calibrationData, uint16_t *frame,
                                   unsigned int width, unsigned int height) {
    uint16_t *cache = calibrationData;

    unsigned int size = width * height / 2;

    uint16_t *end = frame + (size - size % 8);
    uint16_t *framePtr = frame;

    for (; framePtr < end; framePtr += 8) {
        *framePtr = *(cache + *framePtr);
        *(framePtr + 1) = *(cache + *(framePtr + 1));
        *(framePtr + 2) = *(cache + *(framePtr + 2));
        *(framePtr + 3) = *(cache + *(framePtr + 3));
        *(framePtr + 4) = *(cache + *(framePtr + 4));
        *(framePtr + 5) = *(cache + *(framePtr + 5));
        *(framePtr + 6) = *(cache + *(framePtr + 6));
        *(framePtr + 7) = *(cache + *(framePtr + 7));
    }

    end += (size % 8);

    for (; framePtr < end; framePtr++) {
        *framePtr = *(cache + *framePtr);
    }
}
