#ifndef CAMERA_96TOF1_H
#define CAMERA_96TOF1_H

#include "calibration.h"

#include <memory>

#include <aditof/camera.h>
#include <aditof/camera_96tof1_specifics.h>

class Camera96Tof1 : public aditof::Camera {
  public:
    Camera96Tof1(std::unique_ptr<aditof::DeviceInterface> device);
    ~Camera96Tof1();

  public: // implements Camera
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
    aditof::Status requestFrame(aditof::Frame *frame,
                                aditof::FrameUpdateCallback cb);
    aditof::Status getDetails(aditof::CameraDetails &details) const;
    std::shared_ptr<aditof::CameraSpecifics> getSpecifics();
    std::shared_ptr<aditof::DeviceInterface> getDevice();

  private:
    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::CameraSpecifics> m_specifics;
    std::shared_ptr<aditof::DeviceInterface> m_device;
    bool m_devStarted;
    Calibration m_calibration;

  public:
    friend class aditof::Camera96Tof1Specifics;
};

#endif // CAMERA_96TOF1_H
