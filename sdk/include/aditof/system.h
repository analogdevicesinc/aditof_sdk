#ifndef SYSTEM_H
#define SYSTEM_H

#include "sdk_exports.h"
#include "status_definitions.h"

#include <memory>
#include <string>
#include <vector>

class SystemImpl;

namespace aditof {

class Camera;

/**
 * @class System
 * @brief The TOF system that manages the cameras.
 */
class SDK_API System {
  public:
    /**
     * @brief Constructor
     */
    System();

    /**
     * @brief Destructor
     */
    ~System();

    // Make System movable and non-copyable
    /**
     * @brief Move constructor
     */
    System(System &&op) noexcept;

    /**
     * @brief Move assignment
     */
    System &operator=(System &&op) noexcept;

  public:
    /**
     * @brief Initializes the system. Detects available TOF cameras and stores
     * them internally. The System object owns the cameras, which will be
     * destroyed the objects gets out of scope.
     * @return Status
     */
    Status initialize();

    /**
     * @brief Populates the given list with Camera objects that correspond to
     * the available cameras.
     * @param[out] cameraList - A container to be set with the available cameras
     * @return Status
     */
    Status
    getCameraList(std::vector<std::shared_ptr<Camera>> &cameraList) const;

    /**
     * @brief Populates the given list with Camera objects that correspond to
     * the available cameras from the remote target.
     * @param[out] cameraList - A container to be set with the available cameras
     * @param ip - The IP of the remote target
     * @return Status
     */
    Status getCameraListAtIp(std::vector<std::shared_ptr<Camera>> &cameraList,
                             const std::string &ip) const;

  private:
    std::unique_ptr<SystemImpl> m_impl;
};

} // namespace aditof

#endif // SYSTEM_H
