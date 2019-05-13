#ifndef CAMERA_IMPL_H
#define CAMERA_IMPL_H

#include "calibration.h"
#include "camera_definitions.h"
#include "status_definitions.h"

#include <memory>
#include <string>
#include <vector>

namespace aditof {
class Frame;
}

class DeviceInterface;

class CameraImpl {
  public:
    CameraImpl(std::unique_ptr<DeviceInterface> device);
    ~CameraImpl();

  public: // from Camera
    aditof::Status initialize();
    aditof::Status start();
    aditof::Status stop();
    aditof::Status setMode(const std::string &mode,
                           const std::string &modeFilename);
    aditof::Status
    getAvailableModes(std::vector<std::string> &availableModes) const;
    aditof::Status setFrameType(const std::string &frameType);
    aditof::Status
    getAvailableFrameTypes(std::vector<std::string> &availableFrameTypes) const;
    aditof::Status requestFrame(std::shared_ptr<aditof::Frame> frame,
                                aditof::FrameUpdateCallback cb);
    aditof::Status getDetails(aditof::CameraDetails &details) const;

    std::shared_ptr<DeviceInterface> getDevice();

  private:
    aditof::CameraDetails m_details;
    std::shared_ptr<DeviceInterface> m_device;
    bool m_devStarted;
    Calibration m_calibration;
};

#endif // CAMERA_IMPL_H
