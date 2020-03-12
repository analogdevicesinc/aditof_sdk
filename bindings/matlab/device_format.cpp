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
#include "device_format.h"

//******************************************
//  CONSTRUCTOR/DESTRUCTOR
//******************************************
// Initialize the data members of the class.
DeviceFormat::DeviceFormat(void)
    : m_formatHeight(0), m_formatWidth(0), m_formatNumBands(0) {}

DeviceFormat::~DeviceFormat() {}

// Get/set frame height.
void DeviceFormat::setFormatHeight(int value) { m_formatHeight = value; }

int DeviceFormat::getFormatHeight() const { return m_formatHeight; }

// Get/set frame width.
void DeviceFormat::setFormatWidth(int value) { m_formatWidth = value; }

int DeviceFormat::getFormatWidth() const { return m_formatWidth; }

// Get/set number of bands.
void DeviceFormat::setFormatNumBands(int value) { m_formatNumBands = value; }

int DeviceFormat::getFormatNumBands() const { return m_formatNumBands; }

// Get/set frame type.
void DeviceFormat::setFormatFrameType(imaqkit::frametypes::FRAMETYPE value) {
    m_formatFrameType = value;
}

imaqkit::frametypes::FRAMETYPE DeviceFormat::getFormatFrameType() const {
    return m_formatFrameType;
}
