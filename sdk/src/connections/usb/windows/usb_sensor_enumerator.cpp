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
#include "connections/usb/usb_sensor_enumerator.h"
#include "connections/usb/usb_depth_sensor.h"
#include "connections/usb/usb_storage.h"
#include "connections/usb/usb_temperature_sensor.h"
#include "connections/usb/usb_utils.h"
#include "connections/usb/windows/usb_windows_utils.h"
#include "connections/utils/connection_validator.h"
#include "utils.h"

#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif

#include <atlstr.h>
#include <memory>
#include <strmif.h>

using namespace aditof;

static aditof::Status getAvailableSensors(IMoniker *Moniker,
                                          std::string &advertisedSensorData) {
    using namespace aditof;

    IBaseFilter *pVideoInputFilter;

    HRESULT hr = Moniker->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                       (void **)&pVideoInputFilter);
    if (!SUCCEEDED(hr)) {
        LOG(WARNING) << "Failed to bind video input filter";
        return Status::GENERIC_ERROR;
    }

    uint16_t bufferLength;
    hr = UsbWindowsUtils::UvcExUnitReadBuffer(
        pVideoInputFilter, 4, -1, 0, reinterpret_cast<uint8_t *>(&bufferLength),
        sizeof(bufferLength));
    if (FAILED(hr)) {
        pVideoInputFilter->Release();
        LOG(WARNING)
            << "Failed to read size of buffer holding sensors info. Error: "
            << hr;
        return Status::GENERIC_ERROR;
    }
    std::unique_ptr<uint8_t[]> data(new uint8_t[bufferLength + 1]);
    hr = UsbWindowsUtils::UvcExUnitReadBuffer(pVideoInputFilter, 4, -1,
                                              sizeof(bufferLength), data.get(),
                                              bufferLength);
    if (FAILED(hr)) {
        pVideoInputFilter->Release();
        LOG(WARNING) << "Failed to read the content of buffer holding sensors "
                        "info. Error: "
                     << hr;
        return Status::GENERIC_ERROR;
    }

    pVideoInputFilter->Release();

    data[bufferLength] = '\0';
    advertisedSensorData = reinterpret_cast<char *>(data.get());

    return Status::OK;
}

UsbSensorEnumerator::~UsbSensorEnumerator() = default;

Status UsbSensorEnumerator::searchSensors() {
    using namespace std;
    Status status = Status::OK;

    LOG(INFO) << "Looking for USB connected sensors";

    HRESULT hr;

    hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

    std::string devName("ADI TOF DEPTH SENSOR");
    ICreateDevEnum *DevEnum = NULL;

    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&DevEnum));
    if (FAILED(hr)) {
        LOG(ERROR) << "Create Device Enumeration Failed" << std::endl;
        return Status::GENERIC_ERROR;
    }

    IEnumMoniker *EnumMoniker = NULL;
    hr = DevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
                                        &EnumMoniker, 0);

    if (hr != S_OK) {
        DevEnum->Release();
        LOG(ERROR) << "Device Enumeration Error" << std::endl;
        return Status::GENERIC_ERROR;
    }

    IMoniker *Moniker = NULL;
    ULONG cFetched;
    while (EnumMoniker->Next(1, &Moniker, &cFetched) == S_OK) {
        IPropertyBag *PropBag;
        hr = Moniker->BindToStorage(0, 0, IID_PPV_ARGS(&PropBag));

        if (SUCCEEDED(hr)) {
            VARIANT varName;
            VariantInit(&varName);
            hr = PropBag->Read(L"FriendlyName", &varName, 0);

            if (SUCCEEDED(hr)) {
                std::string str(static_cast<LPCTSTR>(CString(varName.bstrVal)));
                if (str.find(devName) != std::string::npos) {
                    SensorInfo sInfo;
                    sInfo.driverPath = str;

                    DLOG(INFO) << "Found USB capture device: " << str;

                    std::string advertisedSensorData;
                    status = getAvailableSensors(Moniker, advertisedSensorData);
                    DLOG(INFO) << "Received the following buffer with "
                                  "available sensors "
                                  "from target: "
                               << advertisedSensorData;
                    if (status == Status::OK) {
                        vector<string> sensorsPaths;
                        Utils::splitIntoTokens(advertisedSensorData, ';',
                                               sensorsPaths);
                        m_sensorsInfo.emplace_back(sInfo);

                        std::string versionString =
                            UsbUtils::getVersionString(sensorsPaths);
                        if (!isValidConnection(aditof::ConnectionType::USB,
                                               versionString)) {
                            LOG(ERROR) << "invalid connection string: "
                                       << versionString;
                            status = Status::GENERIC_ERROR;
                        }

                        m_storagesInfo =
                            UsbUtils::getStorageNamesAndIds(sensorsPaths);

                        m_temperatureSensorsInfo =
                            UsbUtils::getTemperatureSensorNamesAndIds(
                                sensorsPaths);
                    }
                }
            }
            VariantClear(&varName);
            PropBag->Release();
            PropBag = NULL;
        }

        Moniker->Release();
        Moniker = NULL;
    }

    EnumMoniker->Release();
    DevEnum->Release();

    return status;
}

Status UsbSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        auto sensor = std::make_shared<UsbDepthSensor>(sInfo.driverPath);
        depthSensors.emplace_back(sensor);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {
    storages.clear();

    for (const auto &nameAndId : m_storagesInfo) {
        auto storage =
            std::make_shared<UsbStorage>(nameAndId.first, nameAndId.second);
        storages.emplace_back(storage);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        &temperatureSensors) {

    temperatureSensors.clear();

    for (const auto &nameAndId : m_temperatureSensorsInfo) {
        auto tSensor = std::make_shared<UsbTemperatureSensor>(nameAndId.first,
                                                              nameAndId.second);
        temperatureSensors.emplace_back(tSensor);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getCameraTypeOnTarget(CameraType &cameraType) {
    // TO DO: get this information from target. Currently is fixed to 96TOF1
    cameraType = CameraType::AD_96TOF1_EBZ;
    return Status::OK;
}
