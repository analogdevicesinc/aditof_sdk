/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "device_enumerator_impl.h"
#include "utils_linux.h"

#include <dirent.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <sys/stat.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

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

        // Skip device which does not support VIDIOC_G_FMT
        struct v4l2_format fmt;
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
            close(fd);
            continue;
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
