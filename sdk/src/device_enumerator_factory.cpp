#include "device_enumerator_factory.h"
#include "device_enumerator_impl.h"

std::unique_ptr<DeviceEnumeratorInterface>
DeviceEnumeratorFactory::buildDeviceEnumerator() {
    return std::unique_ptr<DeviceEnumeratorInterface>(new DeviceEnumeratorImpl);
}
