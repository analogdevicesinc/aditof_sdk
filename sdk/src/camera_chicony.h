#ifndef CAMERA_CHICONY_H
#define CAMERA_CHICONY_H

#include <memory>

#include <aditof/camera.h>
#include <aditof/camera_chicony_specifics.h>

class CameraChicony : public aditof::Camera {
  public:
    CameraChicony(std::unique_ptr<aditof::DeviceInterface> device);
    ~CameraChicony();

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

  public:
    friend class aditof::CameraChiconySpecifics;
};

#endif // CAMERA_CHICONY_H
