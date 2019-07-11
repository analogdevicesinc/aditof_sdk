#include "ethernet_device.h"
#include "local_device.h"
#include "usb_device.h"

#include <aditof/device_factory.h>

DeviceInterface *
DeviceFactory::buildDevice(const aditof::DeviceConstructionData &data) {
    using namespace aditof;

    switch (data.deviceType) {
    case DeviceType::USB: {
        return new UsbDevice(data);
    }
    case DeviceType::ETHERNET: {
        return new EthernetDevice(data);
    }
    case DeviceType::LOCAL: {
        return new LocalDevice(data);
    }
    }

    return nullptr;
}
