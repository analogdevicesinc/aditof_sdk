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
#include "connections/usb/usb_sensor_enumerator.h"
#include "connections/usb/linux/usb_linux_utils.h"
#include "connections/usb/usb_depth_sensor.h"
#include "connections/usb/usb_storage.h"
#include "connections/usb/usb_utils.h"
#include "utils.h"

#include <dirent.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/videodev2.h>
#include <memory>
#include <sys/stat.h>

using namespace std;
using namespace aditof;

static aditof::Status getAvailableSensors(int fd,
                                          string &advertisedSensorData) {
    uint16_t bufferLength;

    int ret = UsbLinuxUtils::uvcExUnitReadBuffer(
        fd, 4, 0, reinterpret_cast<uint8_t *>(&bufferLength),
        sizeof(bufferLength));
    if (ret < 0) {
        LOG(WARNING)
            << "Failed to read size of buffer holding sensors info. Error: "
            << ret;
        return aditof::Status::GENERIC_ERROR;
    }

    unique_ptr<uint8_t[]> data(new uint8_t[bufferLength + 1]);
    ret = UsbLinuxUtils::uvcExUnitReadBuffer(fd, 4, sizeof(bufferLength),
                                             data.get(), bufferLength);
    if (ret < 0) {
        LOG(WARNING) << "Failed to read the content of buffer holding sensors "
                        "info. Error: "
                     << ret;
        return aditof::Status::GENERIC_ERROR;
    }

    data[bufferLength] = '\0';
    advertisedSensorData = reinterpret_cast<char *>(data.get());

    return aditof::Status::OK;
}

UsbSensorEnumerator::~UsbSensorEnumerator() = default;

Status UsbSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    LOG(INFO) << "Looking for USB connected sensors";

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
        string driverPath(path);
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
        if (-1 == UsbLinuxUtils::xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
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
        if (-1 == UsbLinuxUtils::xioctl(fd, VIDIOC_G_FMT, &fmt)) {
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
        string advertisedSensorData;
        getAvailableSensors(fd, advertisedSensorData);
        close(fd);

        vector<string> sensorsPaths;
        Utils::splitIntoTokens(advertisedSensorData, ';', sensorsPaths);

        SensorInfo sInfo;
        sInfo.driverPath = driverPath;
        int sensorType = UsbUtils::getDepthSensoType(sensorsPaths);
        switch (sensorType) {
        case 0: {
            sInfo.sensorType = SensorType::SENSOR_ADDI9036;
            break;
        }
        default: {
            if (sensorType == -1) {
                LOG(WARNING) << "Failed to indetify sensorType";
            } else {
                LOG(WARNING) << "Unkown sensorType";
            }
        }
        } //switch (sensorType)

        m_sensorsInfo.emplace_back(sInfo);

        m_storagesInfo = UsbUtils::getStorageNames(sensorsPaths);
    }

    closedir(d);

    return status;
}

Status UsbSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        auto sensor = std::make_shared<UsbDepthSensor>(sInfo.sensorType,
                                                       sInfo.driverPath);
        depthSensors.emplace_back(sensor);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {

    storages.clear();

    for (const auto &name : m_storagesInfo) {
        auto storage = std::make_shared<UsbStorage>(name);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        & /*temperatureSensors*/) {
    // TO DO: implement this

    return Status::OK;
}
