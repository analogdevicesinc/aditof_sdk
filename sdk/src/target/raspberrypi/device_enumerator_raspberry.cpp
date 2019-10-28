#include "device_enumerator_impl.h"

#include <dirent.h>
#include <glog/logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h>

aditof::Status DeviceEnumeratorImpl::findDevices(
    std::vector<aditof::DeviceConstructionData> &devices) {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Looking for devices on the target";

    // TO DO: Do we still need to do this?
    // Find all video device paths
    std::vector<std::string> videoPaths;
    const std::string videoDirPath("/dev/");
    const std::string videoBaseName("video");

    DIR *dirp = opendir(videoDirPath.c_str());
    struct dirent *dp;
    while ((dp = readdir(dirp))) {
        if (!strncmp(dp->d_name, videoBaseName.c_str(),
                     videoBaseName.length())) {
            std::string fullvideoPath = videoDirPath + std::string(dp->d_name);
            videoPaths.emplace_back(fullvideoPath);
        }
    }
    closedir(dirp);

    for (const auto &video : videoPaths) {
        std::string devPath;

        if (devPath.empty()) {
            continue;
        }
    }
    // TO DO: Don't guess the device, find a way to identify it so we are sure
    // we've got the right device and is compatible with the SDK
    DeviceConstructionData devData;
    devData.deviceType = DeviceType::LOCAL;
    devData.driverPath = "/dev/video0";
    devices.emplace_back(devData);

    DLOG(INFO) << "Looking at: " << devData.driverPath
               << " for an eligible TOF camera";

    return status;
}
