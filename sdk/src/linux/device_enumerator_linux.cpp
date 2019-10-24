#include "device_enumerator_impl.h"
#include "utils_linux.h"

#include <dirent.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <sys/stat.h>

aditof::Status DeviceEnumeratorImpl::findDevices(
    std::vector<aditof::DeviceConstructionData> &devices) {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Looking for USB connected devices";

    const char *path = "/dev/";
    DIR *d;

    d = opendir(path);
    if (!d) {
        LOG(WARNING) << "Failed to open dir at path: " << path;
        return Status::UNREACHABLE;
    }

    struct dirent *dir;
    char sset[] = "video";
    while ((dir = readdir(d)) != nullptr) {
        if (strspn(sset, (dir->d_name)) != 5) {
            continue;
        }
        std::string driverPath(path);
        driverPath += dir->d_name;

        struct stat st;
        if (-1 == stat(driverPath.c_str(), &st)) {
            LOG(WARNING) << "Cannot identify '" << driverPath
                         << "' error: " << errno << "(" << strerror(errno)
                         << ")";
            continue;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << driverPath << " is no device";
            continue;
        }

        int fd = open(driverPath.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (-1 == fd) {
            LOG(WARNING) << "Cannot open '" << driverPath
                         << "' error: " << errno << "(" << strerror(errno)
                         << ")";
            continue;
        }

        struct v4l2_capability cap;
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
            if (EINVAL == errno) {
                LOG(WARNING) << driverPath << " is not V4L2 device";
                close(fd);
                continue;
            } else {
                LOG(WARNING) << "VIDIOC_QUERYCAP";
            }
        }

        if (strncmp(reinterpret_cast<const char *>(cap.card),
                    "ADI TOF DEPTH SENSOR", 20) != 0) {
            close(fd);
            continue;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            LOG(WARNING) << driverPath << " is no video capture device";
            close(fd);
            continue;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            LOG(WARNING) << driverPath << " does not support streaming i/o";
            close(fd);
            continue;
        }

        close(fd);

        DeviceConstructionData devData;
        devData.deviceType = DeviceType::USB;
        devData.driverPath = driverPath;
        devices.emplace_back(devData);
    }

    closedir(d);

    return status;
}
