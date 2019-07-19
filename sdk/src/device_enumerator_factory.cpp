#include "aditof/device_enumerator_factory.h"
#include "aditof/device_enumerator_ethernet.h"
#include "aditof/device_enumerator_impl.h"

std::unique_ptr<DeviceEnumeratorInterface>
DeviceEnumeratorFactory::buildDeviceEnumerator() {
    return std::unique_ptr<DeviceEnumeratorInterface>(new DeviceEnumeratorImpl);
}

std::unique_ptr<DeviceEnumeratorInterface>
DeviceEnumeratorFactory::buildDeviceEnumeratorEthernet(const std::string &ip) {

    return std::unique_ptr<DeviceEnumeratorInterface>(
        new DeviceEnumeratorEthernet(ip));
}
