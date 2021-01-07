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
#include "aditof/sensor_enumerator_factory.h"

#if defined(DRAGONBOARD) || defined(RASPBERRYPI) || defined(JETSON) ||         \
    defined(XAVIER) || defined(XAVIERNX) || defined(TOYBRICK)
#define TARGET
#endif

/* On target SDK will know only about TargetSensorEnumerator, while
// on remote, SDK will only know about UsbSensorEnumerator and
// (optionally) about NetworkSensorEnumerator. */
#ifdef TARGET
#include "connections/target/target_sensor_enumerator.h"
#else
#include "connections/usb/usb_sensor_enumerator.h"
#ifdef HAS_NETWORK
#include "connections/network/network_sensor_enumerator.h"
#endif
#endif

using namespace aditof;

std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildTargetSensorEnumerator() {
#ifdef TARGET
    return std::unique_ptr<SensorEnumeratorInterface>(
        new TargetSensorEnumerator());
#endif
    return nullptr;
}

std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildUsbSensorEnumerator() {
#ifndef TARGET
    return std::unique_ptr<SensorEnumeratorInterface>(
        new UsbSensorEnumerator());
#endif
    return nullptr;
}

std::unique_ptr<SensorEnumeratorInterface>
SensorEnumeratorFactory::buildNetworkSensorEnumerator(const std::string &ip) {
#ifndef TARGET
#ifdef HAS_NETWORK
    return std::unique_ptr<SensorEnumeratorInterface>(
        new NetworkSensorEnumerator(ip));
#endif
#endif
    return nullptr;
}
