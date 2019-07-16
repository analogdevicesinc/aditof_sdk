#include "device_enumerator_ethernet.h"

DeviceEnumeratorEthernet::DeviceEnumeratorEthernet(const std::string &ip)
    : m_ip(ip) {}

aditof::Status DeviceEnumeratorEthernet::findDevices(
    std::vector<aditof::DeviceConstructionData> & /*devices*/) {
    using namespace aditof;
    Status status = Status::OK;

    return status;
}
