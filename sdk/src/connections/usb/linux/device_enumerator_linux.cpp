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
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <sys/stat.h>

using namespace std;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static void split_into_tokens(const string &s, const char delimiter,
                              vector<string> &tokens) {
    string::size_type start = 0;
    for (string::size_type end = 0;
         (end = s.find(delimiter, end)) != string::npos; ++end) {
        tokens.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(s.substr(start));
}

static aditof::Status getAvailableSensors(int fd,
                                          string &advertisedSensorData) {
    struct uvc_xu_control_query cq;
    uint8_t packet[MAX_BUF_SIZE];
    size_t bytesCount = MAX_BUF_SIZE;
    size_t readBytes = 0;
    size_t readLength = 0;
    size_t addr = 0;

    while (readBytes < bytesCount) {
        readLength = bytesCount - readBytes < MAX_BUF_SIZE
                         ? bytesCount - readBytes
                         : MAX_BUF_SIZE;

        uint32_t *packet_ptr = reinterpret_cast<uint32_t *>(packet);
        packet_ptr[0] = addr;
        packet[4] = MAX_BUF_SIZE;

        // This set property will send the address of sensors information buffer to be read
        CLEAR(cq);
        cq.query = UVC_SET_CUR; // bRequest
        cq.data = static_cast<unsigned char *>(packet);
        cq.size = MAX_BUF_SIZE; // MAX_BUF_SIZE;
        cq.unit = 0x03;         // wIndex
        cq.selector = 4;        // Value for information about available sensors

        if (-1 == xioctl(fd, UVCIOC_CTRL_QUERY, &cq)) {
            LOG(WARNING) << "Error in sending address to device, error: "
                         << errno << "(" << strerror(errno) << ")";
            return aditof::Status::GENERIC_ERROR;
        }

        // This get property will read from the sensors information buffer
        CLEAR(cq);
        cq.query = UVC_GET_CUR; // bRequest
        cq.data = static_cast<unsigned char *>(packet);
        cq.size = MAX_BUF_SIZE; // MAX_BUF_SIZE;
        cq.unit = 0x03;         // wIndex
        cq.selector = 4;        // Value for information about available sensors

        if (-1 == xioctl(fd, UVCIOC_CTRL_QUERY, &cq)) {
            LOG(WARNING) << "Error in reading data from device, error: "
                         << errno << "(" << strerror(errno) << ")";
            return aditof::Status::GENERIC_ERROR;
        }

        int nbSizeBytes = sizeof(uint16_t);
        if (addr == 0) {
            // Read the length of the entire sensors information buffer
            int nbSizeBytes = sizeof(uint16_t);
            uint16_t *sizePtr = reinterpret_cast<uint16_t *>(packet);
            advertisedSensorData.resize(*sizePtr + 1);
            bytesCount = *sizePtr + nbSizeBytes;

            // Re-evaluate the number of bytes to read
            readLength = bytesCount - readBytes < MAX_BUF_SIZE
                             ? bytesCount - readBytes
                             : MAX_BUF_SIZE;

            advertisedSensorData.insert(
                addr, reinterpret_cast<const char *>(packet + nbSizeBytes),
                readLength - nbSizeBytes);
        } else {
            advertisedSensorData.insert(addr - nbSizeBytes,
                                        reinterpret_cast<const char *>(packet),
                                        readLength);
        }

        readBytes += readLength;
        addr += readLength;
    }

    return aditof::Status::OK;
}

aditof::Status DeviceEnumeratorImpl::findDevices(
    vector<aditof::DeviceConstructionData> &devices) {
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
        string advertisedSensorData;
        getAvailableSensors(fd, advertisedSensorData);
        close(fd);

        DeviceConstructionData devData;
        devData.connectionType = ConnectionType::USB;
        devData.driverPath = driverPath;

        vector<string> sensorsPaths;
        split_into_tokens(advertisedSensorData, ';', sensorsPaths);

        for (const auto &path : sensorsPaths) {
            vector<string> keyValueStr;
            split_into_tokens(path, '=', keyValueStr);
            if (keyValueStr[0] == "EEPROM_NAME") {
                EepromConstructionData ecd;
                ecd.driverName = keyValueStr[1];
                devData.eeproms.push_back(ecd);
            } else if (keyValueStr[0] == "EEPROM_PATH") {
                auto &ecd = devData.eeproms.back();
                ecd.driverPath = keyValueStr[1];
            }
        }

        devices.emplace_back(devData);
    }

    closedir(d);

    return status;
}
