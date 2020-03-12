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
#include "source_listener.h"

// TODO: delete:
#include <iostream>

void SourceListener::notify(imaqkit::IEnginePropInfo *propertyInfo,
                            void *newValue) {

    if (newValue) {
        // Get the ID of the new source requested by the user.
        m_source = *static_cast<const int *>(newValue);

        // Do not re-configure the property value unless the device is already
        // opened.
        if (m_parent->isOpen()) {
            // Apply the value to the hardware.
            applyValue();
        }
    }
}

void SourceListener::applyValue(void) {

    // If device cannot be configured while acquiring stop the device,
    // configure the source input, then restart the device.
    bool wasAcquiring = m_parent->isAcquiring();
    if (wasAcquiring) {
        m_parent->stop();
    }

    // ************************************************
    // TODO Include code here to update the device to the
    // source specified by the value of _source.
    // ************************************************

    // Restart the device if it was momentarily stopped to update the source.
    if (wasAcquiring) {
        m_parent->restart();
    }
}
