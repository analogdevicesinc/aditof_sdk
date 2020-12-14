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
#include "assert.h"

#include "prop_listener.h"

void PropListener::notify(imaqkit::IPropInfo *propertyInfo, void *newValue) {

    if (newValue) {
        // Store a handle to the imaqkit::IPropInfo object passed in.
        m_propInfo = propertyInfo;

        // Cast newValue to its appropriate type by checking the property type.
        switch (m_propInfo->getPropertyStorageType()) {

        // _lastDoubleValue contains the value for double properties.
        case imaqkit::propertytypes::DOUBLE:
            m_lastDoubleValue = *reinterpret_cast<double *>(newValue);
            break;

            // _lastIntValue contains value for integer properties.
            // For enumerated properties, _lastIntValue returns the ID number
            // associated with the property value.
        case imaqkit::propertytypes::INT:
            m_lastIntValue = *reinterpret_cast<int16_t *>(newValue);
            break;

            // _lastStrValue contains value for string properties.
        case imaqkit::propertytypes::STRING:
            m_lastStrValue = reinterpret_cast<char *>(newValue);
            break;

            // Since this demo adaptor only uses double, integer, or string
            // value properties, anything else should cause an assertion error.
        default:
            assert(false && "Unhandled property data type. Need to add a new "
                            "data type case.");
        }

        //	std::cout << _lastDoubleValue << " " << _lastIntValue << " "
        //		  << _lastStrValue << " \n";

        //	std::cout << _propInfo->getPropertyName() << "\n";

        // Do not re-configure the property value unless the device is already
        // opened.
        if (m_parent->isOpen() &&
            m_propInfo->getPropertyName() != aditof::TEMPERATURE_STR) {

            //	    std::cout << " applying value\n";

            applyValue();
        }
    }
}

void PropListener::applyValue() {

    // If device cannot be configured while acquiring data, stop the device,
    // configure the feature, then restart the device.
    bool wasAcquiring = m_parent->isAcquiring();
    if (wasAcquiring) {
        // Note: calling stop() will change the acquiring flag to false.
        // When the device tries to restart it invokes
        // DemoAdaptor::startCapture() which triggers notification for all
        // property listeners. Since the device is not acquiring data during
        // this second notification, the device will not stop and restart
        // again.
        m_parent->stop();
    }

    switch (m_propInfo->getPropertyIdentifier()) {
    case aditof::ADITOF_PROPERTY_MODE:
        m_parent->setMode(m_lastIntValue);
        break;
    case aditof::ADITOF_PROPERTY_FRAME_TYPE:
        m_parent->setDisplayedFrameType(m_lastIntValue);
        break;
    case aditof::ADITOF_PROPERTY_SMALL_SIGNAL:
        m_parent->setSmallSignalValue(m_lastIntValue);
        break;
    }

    // ************************************************
    // TODO Add code here to configure the property to its new value.
    // propName contains the name of the property to update.
    // The new value of that property is either stored in
    // _lastDoubleValue, _lastIntValue, or _lastStrValue depending on the
    // property type.
    // ************************************************

    // Restart the device if it was momentarily stopped to update the feature.
    if (wasAcquiring) {
        // Restart the device. This invokes DemoAdaptor::startCapture() which
        // invoke all property listeners.
        m_parent->restart();
    }
}
