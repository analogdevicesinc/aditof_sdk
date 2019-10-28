#include "usb_device.h"
#include "utils.h"
#include "windows_utils.h"

#include "device_utils.h"

#include <atlstr.h>
#include <glog/logging.h>
#include <unordered_map>

#define MAX_PACKET_SIZE 58
#define MAX_BUF_SIZE (MAX_PACKET_SIZE + 2)

static const GUID EXT_UNIT_GUID = {0xFFFFFFFF, 0xFFFF, 0xFFFF, 0xFF, 0xFF, 0xFF,
                                   0xFF,       0xFF,   0xFF,   0xFF, 0xFF};

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct UsbDevice::ImplData {
    ICaptureGraphBuilder2 *pCaptureGraph; // Capture graph builder object
    IGraphBuilder *pGraph;                // Graph builder object
    IMediaControl *pControl;              // Media control object
    IBaseFilter *pVideoInputFilter;       // Video Capture filter
    IBaseFilter *pGrabberF;
    IBaseFilter *pDestFilter;
    IAMStreamConfig *streamConf;
    ISampleGrabber *pGrabber; // Grabs frame
    AM_MEDIA_TYPE *pAmMediaType;
    IMediaEventEx *pMediaEvent;
    SampleGrabberCallback *pCB;
    GUID videoType;
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

UsbDevice::UsbDevice(const aditof::DeviceConstructionData &data)
    : m_devData(data), m_implData(new UsbDevice::ImplData) {
    m_implData->pMediaEvent = nullptr;
    m_deviceDetails.sensorType = aditof::SensorType::SENSOR_96TOF1;
}

UsbDevice::~UsbDevice() {
    HRESULT HR = NOERROR;

    // Check to see if the graph is running, if so stop it.
    if (m_implData->pControl) {
        HR = m_implData->pControl->Pause();

        HR = m_implData->pControl->Stop();
    }

    for (auto it = m_implData->calibration_cache.begin();
         it != m_implData->calibration_cache.begin(); ++it) {
        delete[] it->second.cache;
        it->second.cache = nullptr;
    }

    // Disconnect filters from capture device
    if (m_implData->pVideoInputFilter) {
        NukeDownstream(m_implData->pVideoInputFilter, m_implData->pGraph);
    }

    // Release and zero pointers to our filters etc
    if (m_implData->pDestFilter) {
        m_implData->pDestFilter->Release();
    }

    if (m_implData->pVideoInputFilter) {
        m_implData->pVideoInputFilter->Release();
    }

    if (m_implData->pGrabberF) {
        m_implData->pGrabberF->Release();
    }

    if (m_implData->pGrabber) {
        m_implData->pGrabber->Release();
    }

    if (m_implData->pControl) {
        m_implData->pControl->Release();
    }

    if (m_implData->pMediaEvent) {
        m_implData->pMediaEvent->Release();
    }

    if (m_implData->streamConf) {
        m_implData->streamConf->Release();
    }

    if (m_implData->pAmMediaType) {
        if (m_implData->pAmMediaType->cbFormat != 0) {
            CoTaskMemFree((PVOID)m_implData->pAmMediaType->pbFormat);
            m_implData->pAmMediaType->cbFormat = 0;
            m_implData->pAmMediaType->pbFormat = nullptr;
        }
        if (m_implData->pAmMediaType->pUnk != nullptr) {
            // Unecessary because pUnk should not be used, but safest.
            m_implData->pAmMediaType->pUnk->Release();
            m_implData->pAmMediaType->pUnk = nullptr;
        }
        CoTaskMemFree(m_implData->pAmMediaType);
    }

    // Destroy the graph
    if (m_implData->pGraph) {
        destroyGraph(m_implData->pGraph);
    }

    // Release and zero our capture graph and our main graph
    if (m_implData->pCaptureGraph) {
        m_implData->pCaptureGraph->Release();
    }
    if (m_implData->pGraph) {
        m_implData->pGraph->Release();
    }
}

aditof::Status UsbDevice::open() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Opening device";

    HRESULT hr;
    GUID CAPTURE_MODE = PIN_CATEGORY_CAPTURE;

    hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);

    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, nullptr,
                          CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2,
                          (void **)&(m_implData->pCaptureGraph));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed CoCreateInstance(CLSID_CaptureGraphBuilder2)";
        return Status::GENERIC_ERROR;
    }

    hr = CoCreateInstance(CLSID_FilterGraph, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IGraphBuilder, (void **)&(m_implData->pGraph));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed CoCreateInstance(CLSID_FilterGraph)";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->pCaptureGraph->SetFiltergraph(m_implData->pGraph);
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed SetFiltergraph";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->pGraph->QueryInterface(IID_IMediaControl,
                                            (void **)&(m_implData->pControl));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed QueryInterface(IID_IMediaControl)";
        return Status::GENERIC_ERROR;
    }

    status = getDevice(&m_implData->pVideoInputFilter, m_devData.driverPath);
    if (status != Status::OK) {
        return status;
    }

    std::wstring stemp = s2ws(m_devData.driverPath);
    hr = m_implData->pGraph->AddFilter(m_implData->pVideoInputFilter,
                                       stemp.c_str());
    if (FAILED(hr)) {
        LOG(WARNING) << "ADI TOF Camera cannot be opened";
        return Status::GENERIC_ERROR;
    }

    IAMStreamConfig *streamConfTest = nullptr;
    hr = m_implData->pCaptureGraph->FindInterface(
        &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video, m_implData->pVideoInputFilter,
        IID_IAMStreamConfig, (void **)&streamConfTest);
    if (FAILED(hr)) {
        // TO DO: old IO library allowed this to fail. Investigate why.
        LOG(WARNING) << "Failed FindInterface(PIN_CATEGORY_PREVIEW)";
    } else {
        CAPTURE_MODE = PIN_CATEGORY_PREVIEW;
        streamConfTest->Release();
        streamConfTest = nullptr;
    }

    hr = m_implData->pCaptureGraph->FindInterface(
        &CAPTURE_MODE, &MEDIATYPE_Video, m_implData->pVideoInputFilter,
        IID_IAMStreamConfig, (void **)&(m_implData->streamConf));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed FindInterface(CAPTURE_MODE)";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->streamConf->GetFormat(&(m_implData->pAmMediaType));
    if (FAILED(hr)) {
        LOG(WARNING) << "Failed to get format from streamConf";
        return Status::GENERIC_ERROR;
    }

    hr = CoCreateInstance(CLSID_SampleGrabber, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IBaseFilter, (void **)&(m_implData->pGrabberF));
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not Create Sample Grabber - CoCreateInstance()";
        return Status::GENERIC_ERROR;
    }

    hr =
        m_implData->pGraph->AddFilter(m_implData->pGrabberF, L"Sample Grabber");
    if (FAILED(hr)) {
        LOG(WARNING) << "Could not add Sample Grabber - AddFilter()";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->pGrabberF->QueryInterface(
        IID_ISampleGrabber, (void **)&(m_implData->pGrabber));
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not query SampleGrabber";
        return Status::GENERIC_ERROR;
    }

    hr = m_implData->pGrabber->SetOneShot(FALSE);
    hr = m_implData->pGrabber->SetBufferSamples(TRUE);
    if (FAILED(hr)) {
        LOG(WARNING) << "Fail SetBuffer";
        return Status::GENERIC_ERROR;
    }

    AM_MEDIA_TYPE mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));

    mt.majortype = MEDIATYPE_Video;
    // Included conditional based format for Y16
    if (checkSingleByteFormat(m_implData->pAmMediaType->subtype) ||
        (m_implData->pAmMediaType->subtype == MEDIASUBTYPE_Y16)) {
        mt.subtype = m_implData->pAmMediaType->subtype;
    } else
        mt.subtype = MEDIASUBTYPE_RGB24; // Making it RGB24, does conversion
                                         // from YUV to RGB Included conditional
                                         // based format for Y16 - end

    mt.formattype = FORMAT_VideoInfo;

    hr = m_implData->pGrabber->SetMediaType(&mt);

    // NULL RENDERER//
    // used to give the video stream somewhere to go to.
    hr = CoCreateInstance(CLSID_NullRenderer, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IBaseFilter,
                          (void **)(&(m_implData->pDestFilter)));
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not create filter - NullRenderer";
        return Status::GENERIC_ERROR;
    }

    hr =
        m_implData->pGraph->AddFilter(m_implData->pDestFilter, L"NullRenderer");
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not add filter - NullRenderer";
        return Status::GENERIC_ERROR;
    }

    // RENDER STREAM//
    // This is where the stream gets put together.
    hr = m_implData->pCaptureGraph->RenderStream(
        &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video, m_implData->pVideoInputFilter,
        m_implData->pGrabberF, m_implData->pDestFilter);

    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not connect pins - RenderStream()";
        return Status::GENERIC_ERROR;
    }

    // Try setting the sync source to null - and make it run as fast as possible
    IMediaFilter *pMediaFilter = nullptr;
    hr = m_implData->pGraph->QueryInterface(IID_IMediaFilter,
                                            (void **)&pMediaFilter);
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not get IID_IMediaFilter interface";
        return Status::GENERIC_ERROR;
    } else {
        pMediaFilter->SetSyncSource(nullptr);
        pMediaFilter->Release();
    }

    m_implData->pCB = new SampleGrabberCallback();
    hr = m_implData->pGrabber->SetCallback(m_implData->pCB, 1);

    return status;
}

aditof::Status UsbDevice::start() {
    using namespace aditof;
    Status status = Status::OK;

    // Nothing to do

    return status;
}

aditof::Status UsbDevice::stop() {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Stopping device";

    HRESULT hr = m_implData->pControl->Stop();
    if (FAILED(hr)) {
        LOG(WARNING) << "ERROR: Could not stop graph";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
UsbDevice::getAvailableFrameTypes(std::vector<aditof::FrameDetails> &types) {
    using namespace aditof;
    Status status = Status::OK;

    // Hardcored for now
    FrameDetails details;

    details.width = 640;
    details.height = 960;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "depth_ir";
    types.push_back(details);

    details.width = 668;
    details.height = 750;
    details.cal_data.offset = 0;
    details.cal_data.gain = 1;
    details.type = "raw";
    types.push_back(details);

    // TO DO: Should get these details from the hardware/firmware

    return status;
}

aditof::Status UsbDevice::setFrameType(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr = m_implData->streamConf->GetFormat(&(m_implData->pAmMediaType));
    if (FAILED(hr)) {
        LOG(WARNING) << "failed 7";
        return Status::GENERIC_ERROR;
    }
    VIDEOINFOHEADER *pVih =
        reinterpret_cast<VIDEOINFOHEADER *>(m_implData->pAmMediaType->pbFormat);
    HEADER(pVih)->biWidth = details.width;
    HEADER(pVih)->biHeight = details.height;

    hr = m_implData->streamConf->SetFormat(m_implData->pAmMediaType);

    if (FAILED(hr)) {
        LOG(WARNING) << "Could not set requested resolution (Frame Index)\n";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status UsbDevice::program(const uint8_t *firmware, size_t size) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    size_t written_bytes = 0;
    IKsControl *pKsUnk;
    IKsNodeControl *pUnk;
    GUID guidNodeType;
    BYTE buf[MAX_BUF_SIZE];
    IKsTopologyInfo *pKsTopologyInfo = nullptr;

    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    } else {
        return Status::GENERIC_ERROR;
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                              (VOID **)&pKsUnk);

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        KSP_NODE s;
                        ULONG ulBytesReturned;

                        OAFilterState state;
                        m_implData->pControl->GetState(1, &state);
                        if (state == _FilterState::State_Running) {
                            hr = m_implData->pControl->Pause();
                        }

                        s.Property.Set = EXT_UNIT_GUID;
                        s.Property.Id = 1;
                        s.Property.Flags =
                            KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
                        s.NodeId = i;

                        while (written_bytes < size) {
                            if ((size - written_bytes) > MAX_PACKET_SIZE) {
                                memcpy(&buf[2], &firmware[written_bytes],
                                       MAX_PACKET_SIZE);
                                buf[0] = 0x01;
                                buf[1] = MAX_PACKET_SIZE;

                                hr = pKsUnk->KsProperty(
                                    (PKSPROPERTY)&s, sizeof(s), (LPVOID)&buf[0],
                                    MAX_BUF_SIZE, &ulBytesReturned);

                                if (FAILED(hr)) {
                                    LOG(WARNING) << " Error in Programming AFE";
                                    return Status::GENERIC_ERROR;
                                }
                                written_bytes += MAX_PACKET_SIZE;
                            } else {
                                memset(buf, 0, MAX_BUF_SIZE);
                                buf[0] = 0x02;
                                buf[1] =
                                    static_cast<BYTE>(size - written_bytes);
                                memcpy(&buf[2], &firmware[written_bytes],
                                       size - written_bytes);

                                s.Property.Set = EXT_UNIT_GUID;
                                s.Property.Id = 1;
                                s.Property.Flags = KSPROPERTY_TYPE_SET |
                                                   KSPROPERTY_TYPE_TOPOLOGY;
                                s.NodeId = i;
                                hr = pKsUnk->KsProperty(
                                    (PKSPROPERTY)&s, sizeof(s), (LPVOID)&buf[0],
                                    MAX_BUF_SIZE, &ulBytesReturned);

                                if (FAILED(hr)) {
                                    LOG(WARNING) << " Error in Programming AFE";
                                    return Status::GENERIC_ERROR;
                                }
                                written_bytes = size;
                            }
                        }
                    }

                    // RUN THE STREAM
                    hr = m_implData->pControl->Run();
                    if (FAILED(hr)) {
                        LOG(WARNING) << "ERROR: Could not start graph";
                        return Status::GENERIC_ERROR;
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::getFrame(uint16_t *buffer) {
    using namespace aditof;
    Status status = Status::OK;

    int retryCount = 0;
    HRESULT hr;
    int i, j;
    unsigned int offset[2];
    unsigned int offset_idx;

    unsigned short *tmpbuffer = nullptr;

    VIDEOINFOHEADER *pVi =
        reinterpret_cast<VIDEOINFOHEADER *>(m_implData->pAmMediaType->pbFormat);
    int currentWidth = HEADER(pVi)->biWidth;
    int currentHeight = HEADER(pVi)->biHeight;

    tmpbuffer = (unsigned short *)malloc(currentWidth * currentHeight *
                                         sizeof(unsigned short));

    while (retryCount < 1000) {
        if (m_implData->pCB->newFrame == 1) {
            long bufferSize = currentWidth * currentHeight * 2;
            hr = m_implData->pGrabber->GetCurrentBuffer((long *)&bufferSize,
                                                        (long *)tmpbuffer);
            if (hr != S_OK) {
                LOG(WARNING) << "Incorrect Buffer Size allocated, Allocate "
                                "bigger buffer";
                continue;
            } else {
                EnterCriticalSection(&m_implData->pCB->critSection);
                m_implData->pCB->newFrame = false;
                LeaveCriticalSection(&m_implData->pCB->critSection);
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

aditof::Status UsbDevice::readEeprom(uint32_t address, uint8_t *data,
                                     size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    IKsControl *pKsUnk = nullptr;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;
    uint8_t packet[MAX_BUF_SIZE];

    IKsTopologyInfo *pKsTopologyInfo = nullptr;
    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        KSP_NODE s;
                        ULONG ulBytesReturned;
                        size_t readBytes = 0;
                        size_t readlength = 0;
                        size_t addr = address;

                        while (readBytes < length) {
                            *((uint32_t *)&packet[0]) = addr;
                            readlength = length - readBytes < MAX_BUF_SIZE
                                             ? length - readBytes
                                             : MAX_BUF_SIZE;
                            packet[4] = static_cast<uint8_t>(readlength);

                            // This set property will send the address of the
                            // EEPROM
                            s.Property.Set = EXT_UNIT_GUID;
                            s.Property.Id = 5;
                            s.Property.Flags =
                                KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
                            s.NodeId = i;
                            hr = pKsUnk->KsProperty(
                                (PKSPROPERTY)&s, sizeof(s), (LPVOID)&packet[0],
                                (long)MAX_BUF_SIZE, &ulBytesReturned);
                            if (FAILED(hr)) {
                                LOG(WARNING) << "Error in updating eeprom "
                                                "address and read size";
                                return Status::GENERIC_ERROR;
                            }

                            // This get property will get the value at address
                            // in EEPROM
                            s.Property.Set = EXT_UNIT_GUID;
                            s.Property.Id = 5;
                            s.Property.Flags =
                                KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
                            s.NodeId = i;
                            hr = pKsUnk->KsProperty(
                                (PKSPROPERTY)&s, sizeof(s), (LPVOID)packet,
                                (long)MAX_BUF_SIZE, &ulBytesReturned);
                            if (FAILED(hr)) {
                                LOG(WARNING) << "Error in reading eeprom "
                                                "address and read size";
                                return Status::GENERIC_ERROR;
                            }
                            memcpy(&data[readBytes], packet, readlength);
                            readBytes += readlength;
                            addr += readlength;
                        }
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::writeEeprom(uint32_t address, const uint8_t *data,
                                      size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    IKsControl *pKsUnk = nullptr;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;
    uint8_t packet[MAX_BUF_SIZE];
    uint16_t writelength = 3;

    IKsTopologyInfo *pKsTopologyInfo = nullptr;
    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        KSP_NODE s;
                        ULONG ulBytesReturned;
                        size_t writeLen = 0;
                        size_t writtenBytes = 0;

                        while (writtenBytes < length) {
                            *((uint32_t *)&packet[0]) = address;
                            writeLen = length - writtenBytes > MAX_BUF_SIZE - 5
                                           ? MAX_BUF_SIZE - 5
                                           : length - writtenBytes;
                            packet[4] = static_cast<uint8_t>(writeLen);
                            memcpy(&packet[5], data + writtenBytes, writeLen);

                            // This set property will send the address and data
                            // to EEPROM
                            s.Property.Set = EXT_UNIT_GUID;
                            s.Property.Id = 6;
                            s.Property.Flags =
                                KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
                            s.NodeId = i;
                            hr = pKsUnk->KsProperty(
                                (PKSPROPERTY)&s, sizeof(s), (LPVOID)packet,
                                (long)MAX_BUF_SIZE, &ulBytesReturned);
                            if (FAILED(hr)) {
                                LOG(WARNING) << "Error in updating eeprom "
                                                "address and read size";
                                return Status::GENERIC_ERROR;
                            }
                            writtenBytes += writeLen;
                            address += writeLen;
                        }
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::readAfeRegisters(const uint16_t *address,
                                           uint16_t *data, size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    IKsControl *pKsUnk = nullptr;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;
    uint16_t pageSize = 60;

    IKsTopologyInfo *pKsTopologyInfo = nullptr;

    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        KSP_NODE s;
                        ULONG ulBytesReturned;

                        for (size_t j = 0; j < length; j++) {
                            // This set property will send the address of the
                            // AFE register
                            s.Property.Set = EXT_UNIT_GUID;
                            s.Property.Id = 2;
                            s.Property.Flags =
                                KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
                            s.NodeId = i;
                            hr = pKsUnk->KsProperty(
                                (PKSPROPERTY)&s, sizeof(s), (LPVOID)&address[j],
                                (long)pageSize, &ulBytesReturned);
                            if (FAILED(hr)) {
                                LOG(WARNING) << "Error in updating afe address "
                                                "and read size";
                                return Status::GENERIC_ERROR;
                            }

                            // This get property will get the value of AFE
                            // register from cypress CX3
                            s.Property.Set = EXT_UNIT_GUID;
                            s.Property.Id = 2;
                            s.Property.Flags =
                                KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
                            s.NodeId = i;
                            hr = pKsUnk->KsProperty(
                                (PKSPROPERTY)&s, sizeof(s), (LPVOID)&data[j],
                                (long)pageSize, &ulBytesReturned);
                            if (FAILED(hr)) {
                                LOG(WARNING) << "Error in reading afe address "
                                                "and read size";
                                return Status::GENERIC_ERROR;
                            }
                        }
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::writeAfeRegisters(const uint16_t *address,
                                            const uint16_t *data,
                                            size_t length) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    IKsControl *pKsUnk = nullptr;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;
    BYTE buf[MAX_BUF_SIZE];
    BYTE sampleCnt = 0;

    IKsTopologyInfo *pKsTopologyInfo = nullptr;

    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        length *= 2 * sizeof(uint16_t);
                        while (length) {
                            KSP_NODE s;
                            ULONG ulBytesReturned;

                            memset(buf, 0, MAX_BUF_SIZE);
                            buf[0] = length > MAX_PACKET_SIZE ? 0x01 : 0x02;
                            buf[1] = length > MAX_PACKET_SIZE
                                         ? MAX_PACKET_SIZE
                                         : static_cast<BYTE>(length);
                            for (int i = 0; i < buf[1]; i += 4) {
                                *(uint16_t *)(buf + 2 + i) = address[sampleCnt];
                                *(uint16_t *)(buf + 4 + i) = data[sampleCnt];
                                sampleCnt++;
                            }
                            length -= buf[1];

                            s.Property.Set = EXT_UNIT_GUID;
                            s.Property.Id = 1;
                            s.Property.Flags =
                                KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
                            s.NodeId = i;
                            hr = pKsUnk->KsProperty(
                                (PKSPROPERTY)&s, sizeof(s), (LPVOID)&buf[0],
                                MAX_BUF_SIZE, &ulBytesReturned);

                            if (FAILED(hr)) {
                                LOG(WARNING)
                                    << "Error in Programming AFE register";
                                return Status::GENERIC_ERROR;
                            }
                        }
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::readAfeTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    IKsControl *pKsUnk = nullptr;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;

    IKsTopologyInfo *pKsTopologyInfo = nullptr;

    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    } else {
        return Status::GENERIC_ERROR;
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        KSP_NODE s;
                        ULONG ulBytesReturned;

                        uint16_t readlength =
                            8; // length is length of array, multiply by 2 as
                               // each alement in array is 2 bytes

                        s.Property.Set = EXT_UNIT_GUID;
                        s.Property.Id = 3;
                        s.Property.Flags =
                            KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
                        s.NodeId = i;
                        float integerTemperature[2];
                        hr = pKsUnk->KsProperty((PKSPROPERTY)&s, sizeof(s),
                                                (LPVOID)&integerTemperature,
                                                (long)readlength,
                                                &ulBytesReturned);
                        if (FAILED(hr)) {
                            LOG(WARNING) << "Error in reading temperature";
                            return Status::GENERIC_ERROR;
                        }
                        temperature = integerTemperature[0];
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::readLaserTemp(float &temperature) {
    using namespace aditof;
    Status status = Status::OK;

    HRESULT hr;
    DWORD uiNumNodes;
    IKsControl *pKsUnk = nullptr;
    IKsNodeControl *pUnk = nullptr;
    GUID guidNodeType;

    IKsTopologyInfo *pKsTopologyInfo = nullptr;

    if (m_implData->pVideoInputFilter != nullptr) {
        hr = m_implData->pVideoInputFilter->QueryInterface(
            __uuidof(IKsTopologyInfo), (VOID **)&pKsTopologyInfo);

        if (!SUCCEEDED(hr)) {
            LOG(WARNING) << "setVideoSetting - QueryInterface Error";
            m_implData->pVideoInputFilter->Release();
            m_implData->pVideoInputFilter = nullptr;

            return Status::GENERIC_ERROR;
        }
    } else {
        return Status::GENERIC_ERROR;
    }

    // get nodes number in usb video device capture filter
    if (pKsTopologyInfo->get_NumNodes(&uiNumNodes) == S_OK) {
        // go thru all nodes searching for the node of the
        // KSNODETYPE_DEV_SPECIFIC type, node of this type - represents
        // extension unit of the USB device
        for (UINT i = 0; i < uiNumNodes + 1; i++) {
            if (pKsTopologyInfo->get_NodeType(i, &guidNodeType) == S_OK) {
                if (guidNodeType == KSNODETYPE_DEV_SPECIFIC) {
                    // create node instance
                    hr = pKsTopologyInfo->CreateNodeInstance(
                        i, __uuidof(IKsNodeControl), (void **)&pUnk);

                    // get IKsControl interface from node
                    if (hr == S_OK) {
                        hr = pUnk->QueryInterface(__uuidof(IKsControl),
                                                  (VOID **)&pKsUnk);
                    }

                    // trying to read first control of the extension unit
                    if (hr == S_OK) {
                        KSP_NODE s;
                        ULONG ulBytesReturned;

                        uint16_t readlength =
                            8; // length is length of array, multiply by 2 as
                               // each alement in array is 2 bytes

                        s.Property.Set = EXT_UNIT_GUID;
                        s.Property.Id = 3;
                        s.Property.Flags =
                            KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
                        s.NodeId = i;
                        float integerTemperature[2];
                        hr = pKsUnk->KsProperty((PKSPROPERTY)&s, sizeof(s),
                                                (LPVOID)integerTemperature,
                                                (long)readlength,
                                                &ulBytesReturned);
                        if (FAILED(hr)) {
                            LOG(WARNING) << "Error in reading temperature";
                            return Status::GENERIC_ERROR;
                        }
                        temperature = integerTemperature[1];
                    }
                }
            }
        }
    }

    if (pKsTopologyInfo) {
        pKsTopologyInfo->Release();
    }

    return status;
}

aditof::Status UsbDevice::setCalibrationParams(const std::string &mode,
                                               float gain, float offset,
                                               int range) {
    const int16_t pixelMaxValue = (1 << 12) - 1; // 4095
    CalibrationData calib_data;
    calib_data.mode = mode;
    calib_data.gain = gain;
    calib_data.offset = offset;
    calib_data.cache = aditof::Utils::buildCalibrationCache(
        gain, offset, pixelMaxValue, range);
    m_implData->calibration_cache[mode] = calib_data;

    return aditof::Status::OK;
}

aditof::Status UsbDevice::applyCalibrationToFrame(uint16_t *frame,
                                                  const std::string &mode) {

    float gain = m_implData->calibration_cache[mode].gain;
    float offset = m_implData->calibration_cache[mode].offset;

    VIDEOINFOHEADER *pVi =
        reinterpret_cast<VIDEOINFOHEADER *>(m_implData->pAmMediaType->pbFormat);
    unsigned int width = HEADER(pVi)->biWidth;
    unsigned int height = HEADER(pVi)->biHeight;

    aditof::Utils::calibrateFrame(m_implData->calibration_cache[mode].cache,
                                  frame, width, height);

    return aditof::Status::OK;
}

aditof::Status UsbDevice::getDetails(aditof::DeviceDetails &details) const {
    details = m_deviceDetails;
    return aditof::Status::OK;
}
