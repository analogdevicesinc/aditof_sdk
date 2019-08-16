#include "system_impl.h"
#include "aditof/camera.h"
#include "aditof/device_construction_data.h"
#include "aditof/device_enumerator_factory.h"
#include "aditof/device_factory.h"
#include "camera_impl.h"

#include <glog/logging.h>

SystemImpl::SystemImpl()
    : m_enumerator(DeviceEnumeratorFactory::buildDeviceEnumerator()) {}

SystemImpl::~SystemImpl() {
    for (auto &cam : m_cameras) {
        delete cam;
    }
}

aditof::Status SystemImpl::initialize() {
    using namespace aditof;
    Status status = Status::OK;

    std::vector<aditof::DeviceConstructionData> devsData;
    m_enumerator->findDevices(devsData);

    for (const auto &data : devsData) {
        DeviceInterface *device = DeviceFactory::buildDevice(data);
        aditof::Camera *camera = new Camera(device);
        m_cameras.push_back(camera);
    }

    LOG(INFO) << "System initialized";

    return status;
}

aditof::Status
SystemImpl::getCameraList(std::vector<aditof::Camera *> &cameraList) const {
    using namespace aditof;
    Status status = Status::OK;

    cameraList = m_cameras;

    return status;
}

aditof::Status
SystemImpl::getCameraListAtIp(std::vector<aditof::Camera *> &cameraList,
                              const std::string &ip) const {
    using namespace aditof;
    Status status = Status::OK;

    cameraList.clear();

    std::vector<aditof::DeviceConstructionData> devsData;
    auto ethernetEnumerator =
        DeviceEnumeratorFactory::buildDeviceEnumeratorEthernet(ip);
    status = ethernetEnumerator->findDevices(devsData);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get find devices on target with ip: " << ip;
        return status;
    }

    for (const auto &data : devsData) {
        DeviceInterface *device = DeviceFactory::buildDevice(data);
        aditof::Camera *camera = new Camera(device);
        cameraList.push_back(camera);
    }

    return Status::OK;
}
