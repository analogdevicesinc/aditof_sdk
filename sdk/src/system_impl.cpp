#include "system_impl.h"
#include "camera.h"
#include "camera_impl.h"
#include "device_construction_data.h"
#include "device_enumerator_factory.h"
#include "device_factory.h"

#include <glog/logging.h>

SystemImpl::SystemImpl() {
    m_enumerator = DeviceEnumeratorFactory::buildDeviceEnumerator();
}

SystemImpl::~SystemImpl() = default;

aditof::Status SystemImpl::initialize() {
    using namespace aditof;
    Status status = Status::OK;

    std::vector<aditof::DeviceConstructionData> devsData;
    m_enumerator->findDevices(devsData);
    for (const auto &data : devsData) {
        std::unique_ptr<DeviceInterface> device =
            DeviceFactory::buildDevice(data);
        std::unique_ptr<CameraImpl> cameraImpl(
            new CameraImpl(std::move(device)));
        auto camera = std::make_shared<Camera>(std::move(cameraImpl));
        m_cameras.push_back(camera);
    }

    LOG(INFO) << "System initialized";

    return status;
}

aditof::Status SystemImpl::getCameraList(
    std::vector<std::shared_ptr<aditof::Camera>> &cameraList) const {
    using namespace aditof;
    Status status = Status::OK;

    cameraList = m_cameras;

    return status;
}
