#include "ethernet_device.h"
#include "local_device.h"
#include "usb_device.h"

#include <aditof/device_factory.h>

using namespace aditof;

std::unique_ptr<DeviceInterface>
DeviceFactory::buildDevice(const aditof::DeviceConstructionData &data) {
    using namespace aditof;

    switch (data.deviceType) {
    case DeviceType::USB: {
        return std::unique_ptr<DeviceInterface>(new UsbDevice(data));
    }
    case DeviceType::ETHERNET: {
        return std::unique_ptr<DeviceInterface>(new EthernetDevice(data));
    }
    case DeviceType::LOCAL: {
        return std::unique_ptr<DeviceInterface>(new LocalDevice(data));
    }
    }

    return nullptr;
}
