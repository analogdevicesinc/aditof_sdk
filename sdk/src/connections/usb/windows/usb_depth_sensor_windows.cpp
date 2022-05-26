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
#include "connections/usb/usb_depth_sensor.h"
#include "usb_windows_utils.h"

#include "device_utils.h"

#include <atlstr.h>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <unordered_map>

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct UsbDepthSensor::ImplData {
    UsbHandle handle;
    bool opened;
    std::unordered_map<std::string, CalibrationData> calibration_cache;
};

static std::wstring s2ws(const std::string &s) {
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, nullptr, 0);
    wchar_t *buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}

static aditof::Status getDevice(IBaseFilter **pVideoInputFilter,
                                const std::string &devName) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    BOOL done = FALSE;
    ICreateDevEnum *DevEnum = nullptr;

    hr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&DevEnum));
    if (FAILED(hr)) {
        LOG(WARNING) << "Create Device Enumeration Failed";
        return Status::GENERIC_ERROR;
    }

    IEnumMoniker *EnumMoniker = nullptr;
    hr = DevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
                                        &EnumMoniker, 0);

    if (hr != S_OK) {
        DevEnum->Release();
        LOG(WARNING) << "Device Enumeration Error";
        return Status::GENERIC_ERROR;
    }

    IMoniker *Moniker = nullptr;
    ULONG cFetched;
    while (!done && EnumMoniker->Next(1, &Moniker, &cFetched) == S_OK) {
        IPropertyBag *PropBag;
        hr = Moniker->BindToStorage(nullptr, nullptr, IID_PPV_ARGS(&PropBag));

        if (SUCCEEDED(hr)) {
            VARIANT varName;
            VariantInit(&varName);
            hr = PropBag->Read(L"FriendlyName", &varName, nullptr);

            if (SUCCEEDED(hr)) {
                std::string str(static_cast<LPCTSTR>(CString(varName.bstrVal)));
                if (str == devName) {
                    // We found it, so send it back to the caller
                    hr =
                        Moniker->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                              (void **)pVideoInputFilter);
                    if (!SUCCEEDED(hr)) {
                        LOG(WARNING) << "Failed to bind video input filter";
                    }
                    done = TRUE;
                }
            }
            VariantClear(&varName);
            PropBag->Release();
            PropBag = nullptr;
        }

        Moniker->Release();
        Moniker = nullptr;
    }

    EnumMoniker->Release();
    DevEnum->Release();

    return status;
}

static bool checkSingleByteFormat(GUID FormatType) {
    bool IsSingleByteFormat = true;

    if (FormatType == MEDIASUBTYPE_Y800 || FormatType == MEDIASUBTYPE_Y8 ||
        FormatType == MEDIASUBTYPE_GREY || FormatType == MEDIASUBTYPE_BY8) {
        IsSingleByteFormat = true;
    } else {
        IsSingleByteFormat = false;
    }

    return IsSingleByteFormat;
}

static void NukeDownstream(IBaseFilter *pBF, IGraphBuilder *pGraph) {
    IPin *pP, *pTo;
    ULONG u;
    IEnumPins *pins = nullptr;
    PIN_INFO pininfo;
    HRESULT hr = pBF->EnumPins(&pins);
    pins->Reset();
    while (hr == NOERROR) {
        hr = pins->Next(1, &pP, &u);
        if (hr == S_OK && pP) {
            pP->ConnectedTo(&pTo);
            if (pTo) {
                hr = pTo->QueryPinInfo(&pininfo);
                if (hr == NOERROR) {
                    if (pininfo.dir == PINDIR_INPUT) {
                        NukeDownstream(pininfo.pFilter, pGraph);
                        pGraph->Disconnect(pTo);
                        pGraph->Disconnect(pP);
                        pGraph->RemoveFilter(pininfo.pFilter);
                    }
                    pininfo.pFilter->Release();
                    pininfo.pFilter = nullptr;
                }
                pTo->Release();
            }
            pP->Release();
        }
    }
    if (pins)
        pins->Release();
}

static void destroyGraph(IGraphBuilder *pGraph) {
    HRESULT hr = 0;
    int i = 0;

    while (hr == NOERROR) {
        IEnumFilters *pEnum = nullptr;
        ULONG cFetched;

        // We must get the enumerator again every time because removing a filter
        // from the graph invalidates the enumerator. We always get only the
        // first filter from each enumerator.
        hr = pGraph->EnumFilters(&pEnum);

        IBaseFilter *pFilter = nullptr;

        if (pEnum->Next(1, &pFilter, &cFetched) == S_OK) {
            FILTER_INFO FilterInfo;
            memset(&FilterInfo, 0, sizeof(FilterInfo));
            hr = pFilter->QueryFilterInfo(&FilterInfo);
            FilterInfo.pGraph->Release();

            int count = 0;
            char buffer[255];
            memset(buffer, 0, 255 * sizeof(char));

            while (FilterInfo.achName[count] != 0x00) {
                buffer[count] = (char)FilterInfo.achName[count];
                count++;
            }

            hr = pGraph->RemoveFilter(pFilter);
            if (FAILED(hr)) {
                LOG(WARNING) << "SETUP: pGraph->RemoveFilter() failed.";
            }

            pFilter->Release();
            pFilter = nullptr;
        } else
            break;
        pEnum->Release();
        pEnum = nullptr;
        i++;
    }

    return;
}

UsbDepthSensor::UsbDepthSensor(const std::string &driverPath)
    : m_driverPath(driverPath), m_implData(new UsbDepthSensor::ImplData) {
    m_implData->handle.pMediaEvent = nullptr;
    m_implData->opened = false;

    m_sensorDetails.connectionType = aditof::ConnectionType::USB;
}

UsbDepthSensor::~UsbDepthSensor() {
    HRESULT HR = NOERROR;

    // Check to see if the graph is running, if so stop it.
    if (m_implData->handle.pControl) {
        HR = m_implData->handle.pControl->Pause();

        HR = m_implData->handle.pControl->Stop();
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }

    // Disconnect filters from capture device
    if (m_implData->handle.pVideoInputFilter) {
        NukeDownstream(m_implData->handle.pVideoInputFilter,
                       m_implData->handle.pGraph);
    }

    // Release and zero pointers to our filters etc
    if (m_implData->handle.pDestFilter) {
        m_implData->handle.pDestFilter->Release();
    }

    if (m_implData->handle.pVideoInputFilter) {
        m_implData->handle.pVideoInputFilter->Release();
    }

    if (m_implData->handle.pGrabberF) {
        m_implData->handle.pGrabberF->Release();
    }

    if (m_implData->handle.pGrabber) {
        m_implData->handle.pGrabber->Release();
    }

    if (m_implData->handle.pControl) {
        m_implData->handle.pControl->Release();
    }

    if (m_implData->handle.pMediaEvent) {
        m_implData->handle.pMediaEvent->Release();
    }

    if (m_implData->handle.streamConf) {
        m_implData->handle.streamConf->Release();
    }

    if (m_implData->handle.pAmMediaType) {
        if (m_implData->handle.pAmMediaType->cbFormat != 0) {
            CoTaskMemFree((PVOID)m_implData->handle.pAmMediaType->pbFormat);
            m_implData->handle.pAmMediaType->cbFormat = 0;
            m_implData->handle.pAmMediaType->pbFormat = nullptr;
        }
        if (m_implData->handle.pAmMediaType->pUnk != nullptr) {
            // Unecessary because pUnk should not be used, but safest.
            m_implData->handle.pAmMediaType->pUnk->Release();
            m_implData->handle.pAmMediaType->pUnk = nullptr;
        }
        CoTaskMemFree(m_implData->handle.pAmMediaType);
    }

    // Destroy the graph
    if (m_implData->handle.pGraph) {
        destroyGraph(m_implData->handle.pGraph);
    }

    // Release and zero our capture graph and our main graph
    if (m_implData->handle.pCaptureGraph) {
        m_implData->handle.pCaptureGraph->Release();
    }
    if (m_implData->handle.pGraph) {
        m_implData->handle.pGraph->Release();
    }
}

aditof::Status UsbDepthSensor::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    HRESULT hr;
    GUID CAPTURE_MODE = PIN_CATEGORY_CAPTURE;

    hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);

    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, nullptr,
                          CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2,
                          (void **)&(m_implData->handle.pCaptureGraph));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed CoCreateInstance(CLSID_CaptureGraphBuilder2)";
        return Status::GENERIC_ERROR;
    }

    hr = CoCreateInstance(CLSID_FilterGraph, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IGraphBuilder,
                          (void **)&(m_implData->handle.pGraph));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed CoCreateInstance(CLSID_FilterGraph)";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pCaptureGraph->SetFiltergraph(
        m_implData->handle.pGraph);
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed SetFiltergraph";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGraph->QueryInterface(
        IID_IMediaControl, (void **)&(m_implData->handle.pControl));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed QueryInterface(IID_IMediaControl)";
        return Status::GENERIC_ERROR;
    }

    status = getDevice(&m_implData->handle.pVideoInputFilter, m_driverPath);
    if (status != Status::OK) {
        return status;
    }

    std::wstring stemp = s2ws(m_driverPath);
    hr = m_implData->handle.pGraph->AddFilter(
        m_implData->handle.pVideoInputFilter, stemp.c_str());
    if (FAILED(hr)) {
        LOG(WARNING) << "ADI TOF Camera cannot be opened";
        return Status::GENERIC_ERROR;
    }

    IAMStreamConfig *streamConfTest = nullptr;
    hr = m_implData->handle.pCaptureGraph->FindInterface(
        &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video,
        m_implData->handle.pVideoInputFilter, IID_IAMStreamConfig,
        (void **)&streamConfTest);
    if (FAILED(hr)) {
        // TO DO: old IO library allowed this to fail. Investigate why.
        LOG(WARNING) << "Failed FindInterface(PIN_CATEGORY_PREVIEW)";
    } else {
        CAPTURE_MODE = PIN_CATEGORY_PREVIEW;
        streamConfTest->Release();
        streamConfTest = nullptr;
    }

    hr = m_implData->handle.pCaptureGraph->FindInterface(
        &CAPTURE_MODE, &MEDIATYPE_Video, m_implData->handle.pVideoInputFilter,
        IID_IAMStreamConfig, (void **)&(m_implData->handle.streamConf));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed FindInterface(CAPTURE_MODE)";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.streamConf->GetFormat(
        &(m_implData->handle.pAmMediaType));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed to get format from streamConf";
        return Status::GENERIC_ERROR;
    }

    hr = CoCreateInstance(CLSID_SampleGrabber, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IBaseFilter,
                          (void **)&(m_implData->handle.pGrabberF));
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not Create Sample Grabber - CoCreateInstance()";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGraph->AddFilter(m_implData->handle.pGrabberF,
                                              L"Sample Grabber");
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not add Sample Grabber - AddFilter()";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGrabberF->QueryInterface(
        IID_ISampleGrabber, (void **)&(m_implData->handle.pGrabber));
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not query SampleGrabber";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGrabber->SetOneShot(FALSE);
    hr = m_implData->handle.pGrabber->SetBufferSamples(TRUE);
    if (FAILED(hr)) {
        LOG(WARNING) << "Fail SetBuffer";
        return Status::GENERIC_ERROR;
    }

    AM_MEDIA_TYPE mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));

    mt.majortype = MEDIATYPE_Video;
    // Included conditional based format for Y16
    if (checkSingleByteFormat(m_implData->handle.pAmMediaType->subtype) ||
        (m_implData->handle.pAmMediaType->subtype == MEDIASUBTYPE_Y16)) {
        mt.subtype = m_implData->handle.pAmMediaType->subtype;
    } else
        mt.subtype = MEDIASUBTYPE_RGB24; // Making it RGB24, does conversion
                                         // from YUV to RGB Included conditional
                                         // based format for Y16 - end

    mt.formattype = FORMAT_VideoInfo;

    hr = m_implData->handle.pGrabber->SetMediaType(&mt);

    // NULL RENDERER//
    // used to give the video stream somewhere to go to.
    hr = CoCreateInstance(CLSID_NullRenderer, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IBaseFilter,
                          (void **)(&(m_implData->handle.pDestFilter)));
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not create filter - NullRenderer";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->handle.pGraph->AddFilter(m_implData->handle.pDestFilter,
                                              L"NullRenderer");
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not add filter - NullRenderer";
        return Status::GENERIC_ERROR;
    }

    // RENDER STREAM//
    // This is where the stream gets put together.
    hr = m_implData->handle.pCaptureGraph->RenderStream(
        &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video,
        m_implData->handle.pVideoInputFilter, m_implData->handle.pGrabberF,
        m_implData->handle.pDestFilter);

    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not connect pins - RenderStream()";
        return Status::GENERIC_ERROR;
    }

    // Try setting the sync source to null - and make it run as fast as possible
    IMediaFilter *pMediaFilter = nullptr;
    hr = m_implData->handle.pGraph->QueryInterface(IID_IMediaFilter,
                                                   (void **)&pMediaFilter);
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not get IID_IMediaFilter interface";
        return Status::GENERIC_ERROR;
    } else {
        pMediaFilter->SetSyncSource(nullptr);
        pMediaFilter->Release();
    }

    m_implData->handle.pCB = new SampleGrabberCallback();
    hr = m_implData->handle.pGrabber->SetCallback(m_implData->handle.pCB, 1);

    m_implData->opened = true;

    return status;
}

aditof::Status UsbDepthSensor::start() {
    using namespace aditof;
    Status status = Status::OK;

    // Nothing to do

    return status;
}

aditof::Status UsbDepthSensor::stop() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Stopping device";

    HRESULT hr = m_implData->handle.pControl->Stop();
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not stop graph";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    // Hardcored for now
    FrameDetails details;

    details.width = aditof::USB_FRAME_WIDTH;
    details.height = aditof::USB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height * 2; //TODO
    details.type = "depth_ir";
    types.push_back(details);

    details.width = aditof::USB_FRAME_WIDTH;
    details.height = aditof::USB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height; //TODO
    details.type = "depth";
    types.push_back(details);

    details.width = aditof::USB_FRAME_WIDTH;
    details.height = aditof::USB_FRAME_HEIGHT;
    details.fullDataWidth = details.width;
    details.fullDataHeight = details.height; //TODO
    details.type = "ir";
    types.push_back(details);

    // TO DO: Should get these details from the hardware/firmware

    return status;
}

aditof::Status
UsbDepthSensor::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr = m_implData->handle.streamConf->GetFormat(
        &(m_implData->handle.pAmMediaType));
    if (FAILED(hr)) {
        LOG(WARNING) << "failed 7";
        return Status::GENERIC_ERROR;
    }
    VIDEOINFOHEADER *pVih = reinterpret_cast<VIDEOINFOHEADER *>(
        m_implData->handle.pAmMediaType->pbFormat);
    HEADER(pVih)->biWidth = details.fullDataWidth;
    HEADER(pVih)->biHeight = details.fullDataHeight;

    hr = m_implData->handle.streamConf->SetFormat(
        m_implData->handle.pAmMediaType);

    if (FAILED(hr)) {
        LOG(WARNING) << "Could not set requested resolution (Frame Index)\n";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDepthSensor::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;

    ExUnitHandle handle;

    if (firmware == nullptr) {
        LOG(ERROR) << "Received firmware null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(size > 0);

    HRESULT hr = UsbWindowsUtils::UvcFindNodeAndGetControl(
        &handle, &m_implData->handle.pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    OAFilterState state;
    m_implData->handle.pControl->GetState(1, &state);
    if (state == _FilterState::State_Running) {
        hr = m_implData->handle.pControl->Pause();
    }

    size_t written_bytes = 0;
    BYTE buf[MAX_BUF_SIZE];

    while (written_bytes < size) {
        if ((size - written_bytes) > MAX_PACKET_SIZE) {
            memcpy(&buf[2], &firmware[written_bytes], MAX_PACKET_SIZE);
            buf[0] = 0x01;
            buf[1] = MAX_PACKET_SIZE;
            written_bytes += MAX_PACKET_SIZE;
        } else {
            memset(buf, 0, MAX_BUF_SIZE);
            buf[0] = 0x02;
            buf[1] = static_cast<BYTE>(size - written_bytes);
            memcpy(&buf[2], &firmware[written_bytes], size - written_bytes);
            written_bytes = size;
        }
        hr = UsbWindowsUtils::UvcExUnitSetProperty(&handle, 1, &buf[0],
                                                   MAX_BUF_SIZE);
        if (FAILED(hr)) {
            LOG(WARNING) << " Error in Programming AFE";
            return Status::GENERIC_ERROR;
        }
    }

    // RUN THE STREAM
    hr = m_implData->handle.pControl->Run();
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not start graph";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::getFrame(uint16_t *buffer,
                                        aditof::BufferInfo *bufferInfo) {
    using namespace aditof;
    Status status = Status::OK;

    if (buffer == nullptr) {
        LOG(ERROR) << "Received buffer null pointer";
        return Status::INVALID_ARGUMENT;
    }

    int retryCount = 0;
    HRESULT hr;

    unsigned short *tmpbuffer = nullptr;

    VIDEOINFOHEADER *pVi = reinterpret_cast<VIDEOINFOHEADER *>(
        m_implData->handle.pAmMediaType->pbFormat);
    int currentWidth = HEADER(pVi)->biWidth;
    int currentHeight = HEADER(pVi)->biHeight;

    tmpbuffer = (unsigned short *)malloc(currentWidth * currentHeight *
                                         sizeof(unsigned short));

    while (retryCount < 1000) {
        if (m_implData->handle.pCB->newFrame == 1) {
            long bufferSize = currentWidth * currentHeight * 2;
            hr = m_implData->handle.pGrabber->GetCurrentBuffer(
                (long *)&bufferSize, (long *)tmpbuffer);
            if (hr != S_OK) {
                LOG(WARNING) << "Incorrect Buffer Size allocated, Allocate "
                                "bigger buffer";
                continue;
            } else {
                EnterCriticalSection(&m_implData->handle.pCB->critSection);
                m_implData->handle.pCB->newFrame = false;
                LeaveCriticalSection(&m_implData->handle.pCB->critSection);
                break;
            }
        } else {
            Sleep(1);
            retryCount++;
        }
    }

    aditof::deinterleave((const char *)tmpbuffer, buffer,
                         currentWidth * currentHeight * 3 / 2, currentWidth,
                         currentHeight);

    free(tmpbuffer);

    return retryCount >= 1000 ? Status::GENERIC_ERROR : status;
}

aditof::Status UsbDepthSensor::readAfeRegisters(const uint16_t *address,
                                                uint16_t *data, size_t length) {
    using namespace aditof;

    if (address == nullptr) {
        LOG(ERROR) << "Received AfeRegisters address null pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (data == nullptr) {
        LOG(ERROR) << "Received AfeRegisters data null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(length > 0);

    ExUnitHandle handle;
    ULONG pageSize = 60;

    HRESULT hr = UsbWindowsUtils::UvcFindNodeAndGetControl(
        &handle, &m_implData->handle.pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    for (size_t j = 0; j < length; j++) {
        hr = UsbWindowsUtils::UvcExUnitSetProperty(
            &handle, 2, reinterpret_cast<const uint8_t *>(&address[j]),
            pageSize);
        if (FAILED(hr)) {
            LOG(WARNING)
                << "Failed to set property via UVC extension unit. Error: "
                << std::hex << hr;
            return Status::GENERIC_ERROR;
        }

        hr = UsbWindowsUtils::UvcExUnitGetProperty(
            &handle, 2, reinterpret_cast<uint8_t *>(&data[j]), pageSize);
        if (FAILED(hr)) {
            LOG(WARNING)
                << "Failed to get property via UVC extension unit. Error: "
                << std::hex << hr;
            return Status::GENERIC_ERROR;
        }
    }

    return Status::OK;
}

aditof::Status UsbDepthSensor::writeAfeRegisters(const uint16_t *address,
                                                 const uint16_t *data,
                                                 size_t length) {
    using namespace aditof;

    ExUnitHandle handle;

    if (address == nullptr) {
        LOG(ERROR) << "Received AfeRegisters address null pointer";
        return Status::INVALID_ARGUMENT;
    }

    if (data == nullptr) {
        LOG(ERROR) << "Received AfeRegisters data null pointer";
        return Status::INVALID_ARGUMENT;
    }

    assert(length > 0);

    HRESULT hr = UsbWindowsUtils::UvcFindNodeAndGetControl(
        &handle, &m_implData->handle.pVideoInputFilter);
    if (hr != S_OK) {
        LOG(WARNING) << "Failed to find node and get control. Error: "
                     << std::hex << hr;
        return Status::GENERIC_ERROR;
    }

    BYTE buf[MAX_BUF_SIZE];
    BYTE sampleCnt = 0;
    BYTE b = sizeof(uint16_t); // Size (in bytes) of an AFE register

    length *= 2 * sizeof(uint16_t);

    const BYTE *pAddr = reinterpret_cast<const BYTE *>(address);
    const BYTE *pData = reinterpret_cast<const BYTE *>(data);
    const BYTE *ptr = pAddr;
    bool pointingAtAddr = true;

    while (length) {
        memset(buf, 0, MAX_BUF_SIZE);
        buf[0] = length > MAX_PACKET_SIZE ? 0x01 : 0x02;
        buf[1] = length > MAX_PACKET_SIZE ? MAX_PACKET_SIZE
                                          : static_cast<BYTE>(length);
        for (int n = 0; n < buf[1]; ++n) {
            if ((sampleCnt / b) && (sampleCnt % b == 0)) {
                if (pointingAtAddr) {
                    pAddr = ptr;
                    ptr = pData;
                } else {
                    pData = ptr;
                    ptr = pAddr;
                }
                pointingAtAddr = !pointingAtAddr;
            }
            buf[2 + n] = *ptr++;
            ++sampleCnt;
        }
        length -= buf[1];

        hr = UsbWindowsUtils::UvcExUnitSetProperty(&handle, 1, &buf[0],
                                                   MAX_BUF_SIZE);
        if (FAILED(hr)) {
            LOG(WARNING)
                << "Failed to set property via UVC extension unit. Error: "
                << std::hex << hr;
            return Status::GENERIC_ERROR;
        }
    }

    return Status::OK;
}

aditof::Status
UsbDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status UsbDepthSensor::getHandle(void **handle) {
    if (m_implData->opened) {
        *handle = &m_implData->handle;
        return aditof::Status::OK;
    } else {
        *handle = nullptr;
        LOG(ERROR) << "Won't return the handle. Device hasn't been opened yet.";
        return aditof::Status::UNAVAILABLE;
    }
}

aditof::Status UsbDepthSensor::getName(std::string &sensorName) const {
    sensorName = m_sensorName;

    return aditof::Status::OK;
}
