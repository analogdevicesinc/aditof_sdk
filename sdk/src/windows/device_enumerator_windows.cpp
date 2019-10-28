#include "device_enumerator_impl.h"
#include "windows_utils.h"

#include <glog/logging.h>

#include <atlstr.h>

aditof::Status DeviceEnumeratorImpl::findDevices(
    std::vector<aditof::DeviceConstructionData> &devices) {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Looking for USB connected devices";

    HRESULT hr;

    hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

    std::string devName("ADI TOF DEPTH SENSOR");
    ICreateDevEnum *DevEnum = NULL;

    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&DevEnum));
    if (FAILED(hr)) {
        std::cout << "Create Device Enumeration Failed" << std::endl;
        return Status::GENERIC_ERROR;
    }

    IEnumMoniker *EnumMoniker = NULL;
    hr = DevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
                                        &EnumMoniker, 0);

    if (hr != S_OK) {
        DevEnum->Release();
        std::cout << "Device Enumeration Error" << std::endl;
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
                    DeviceConstructionData devData;
                    devData.deviceType = DeviceType::USB;
                    devData.driverPath = str;
                    devices.emplace_back(devData);
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
