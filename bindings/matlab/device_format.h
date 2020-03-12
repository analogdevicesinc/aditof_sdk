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
#ifndef __DEMO_DEVICEFORMAT_HEADER__
#define __DEMO_DEVICEFORMAT_HEADER__

#include <mwadaptorimaq.h>

/**
 * Class DeviceFormat
 *
 * @brief: Keeps all format-related data together, including format name,
 *         frame type, height, width, and number of bands.
 *         Inherits from the imaqkit::IMAQInterface class.
 *         Objects of this class are instantiated and populated in the
 *         getAvailHW() call.
 */

class DeviceFormat : public imaqkit::IMAQInterface {

  public:
    // *******************************************************************
    // CONSTRUCTOR/DESTRUCTOR
    // *******************************************************************
    DeviceFormat(void);
    virtual ~DeviceFormat(void);

    // *******************************************************************
    // METHODS FOR ACCESSING VIDEO FORMAT INFORMATION.
    // *******************************************************************
    /// Retrieve height.
    int getFormatHeight() const;

    /// Retrieve width.
    int getFormatWidth() const;

    /// Retrieve number of bands.
    int getFormatNumBands() const;

    /// Retrieve frame type.
    imaqkit::frametypes::FRAMETYPE getFormatFrameType() const;

    // *******************************************************************
    // METHODS FOR SAVING VIDEO INFORMATION.
    // *******************************************************************
    /// Save video format height.
    void setFormatHeight(int value);

    /// Save video format width.
    void setFormatWidth(int value);

    /// Save number of bands.
    void setFormatNumBands(int value);

    /// Save frame type.
    void setFormatFrameType(imaqkit::frametypes::FRAMETYPE value);

  private:
    /// The height of the frame in pixels
    int m_formatHeight;

    /// The width of the frame in pixels
    int m_formatWidth;

    /// The number of bands of data returned
    int m_formatNumBands;

    /// The frame type
    imaqkit::frametypes::FRAMETYPE m_formatFrameType;
};

#endif
