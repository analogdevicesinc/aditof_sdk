#ifndef DEVICE_ENUMERATOR_ETHERNET_H
#define DEVICE_ENUMERATOR_ETHERNET_H

#include <aditof/device_enumerator_interface.h>

#include <string>

class DeviceEnumeratorEthernet : public aditof::DeviceEnumeratorInterface {
  public:
    DeviceEnumeratorEthernet(const std::string &ip);
    ~DeviceEnumeratorEthernet();

  public: // implements DeviceEnumeratorInterface
    virtual aditof::Status
    findDevices(std::vector<aditof::DeviceConstructionData> &devices);

  private:
    std::string m_ip;
};

#endif // DEVICE_ENUMERATOR_ETHERNET_H
