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
#include "connections/mipi/target_sensor_enumerator.h"
#include "target_definitions.h"

#include <dirent.h>
#include <glog/logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

using namespace aditof;

namespace local {

aditof::Status findDevicePathsAtMedia(const std::string &media,
                                      std::string &dev_name,
                                      std::string &subdev_name) {
    using namespace aditof;
    using namespace std;

    char *buf;
    int size = 0;
    int idx = 0;

    /* Run media-ctl to get the video processing pipes */
    char cmd[64];
    sprintf(cmd, "sudo media-ctl -d %s -p", media.c_str());
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        LOG(WARNING) << "Error running media-ctl";
        return Status::GENERIC_ERROR;
    }

    /* Read the media-ctl output stream */
    buf = (char *)malloc(128 * 1024);
    while (!feof(fp)) {
        fread(&buf[size], 1, 1, fp);
        size++;
    }
    pclose(fp);
    buf[size] = '\0';

    /* Get the list of entities from output buffer */
    string str(buf);
    free(buf);
    vector<string> entities;
    size_t pos = str.find("entity");
    while (pos != string::npos) {
        size_t pos1 = str.find("entity", pos + 1);
        if (pos1 != string::npos) {
            string str1(str, pos, pos1 - pos);
            entities.push_back(str1);
        } else {
            string str1(str, pos, size - pos);
            entities.push_back(str1);
            break;
        }
        pos = pos1;
    }

    /* Find the names of the associated device and subdevice */
    string fstr("<- \"addi903x");
    for (idx = 0; idx < entities.size(); idx++) {
        size_t found = entities[idx].find(fstr);
        size_t found1 = 0;
        if (found != string::npos) {
            found = entities[idx].find("Source", found);
            if (found == string::npos) {
                found = entities[idx].find("device node name ");
                if (found == string::npos) {
                    break;
                }
                found1 = entities[idx].find("pad", found);
                if (found1 == string::npos) {
                    break;
                }
                dev_name = entities[idx].substr(
                    found + strlen("device node name "),
                    found1 - found - strlen("device node name ") - 2);
                break;
            }
            found = entities[idx].find("ENABLED", found);
            if (found != string::npos) {
                found = entities[idx].rfind("\"", found);
                found1 = entities[idx].rfind("\"", found - 1);
                string str1(entities[idx], found1, found - found1);
                fstr = "<- " + str1;
                continue;
            }
        }
    }
    fstr = "addi903x";
    for (; idx < entities.size(); idx++) {
        size_t found = entities[idx].find(fstr);
        size_t found1 = 0;
        if (found == string::npos) {
            continue;
        }

        found = entities[idx].find("device node name ");
        if (found == string::npos) {
            break;
        }
        found1 = entities[idx].find("pad", found);
        if (found1 == string::npos) {
            break;
        }
        subdev_name = entities[idx].substr(found + strlen("device node name "),
                                           found1 - found -
                                               strlen("device node name ") - 2);
        break;
    }

    return Status::OK;
}

}; // namespace local

//aditof::Status DeviceEnumeratorImpl::findDevices(
//    std::vector<aditof::DeviceConstructionData> &devices) {
//    using namespace aditof;
//    Status status = Status::OK;

//    LOG(INFO) << "Looking for devices on the target";

//    // Find all media device paths
//    std::vector<std::string> mediaPaths;
//    const std::string mediaDirPath("/dev/");
//    const std::string mediaBaseName("media");

//    DIR *dirp = opendir(mediaDirPath.c_str());
//    struct dirent *dp;
//    while ((dp = readdir(dirp))) {
//        if (!strncmp(dp->d_name, mediaBaseName.c_str(),
//                     mediaBaseName.length())) {
//            std::string fullMediaPath = mediaDirPath + std::string(dp->d_name);
//            mediaPaths.emplace_back(fullMediaPath);
//        }
//    }
//    closedir(dirp);

//    // Identify any eligible time of flight cameras
//    for (const auto &media : mediaPaths) {

//        DLOG(INFO) << "Looking at: " << media << " for an eligible TOF camera";

//        std::string devPath;
//        std::string subdevPath;

//        status = local::findDevicePathsAtMedia(media, devPath, subdevPath);
//        if (status != Status::OK) {
//            LOG(WARNING) << "failed to find device paths at media: " << media;
//            return status;
//        }

//        if (devPath.empty() || subdevPath.empty()) {
//            continue;
//        }

//        DeviceConstructionData devData;
//        devData.connectionType = ConnectionType::LOCAL;
//        devData.driverPath = devPath + ";" + subdevPath;

//        // Check if EEPROM is available
//        struct stat st;
//        if (stat(EEPROM_DEV_PATH, &st) == 0) {
//            EepromConstructionData eData;
//            eData.driverName = EEPROM_NAME;
//            eData.driverPath = EEPROM_DEV_PATH;
//            devData.eeproms.emplace_back(eData);
//        }

//        devices.emplace_back(devData);
//    }

//    return status;
//}

Status TargetSensorEnumerator::searchSensors() {
    // TO DO: implement this

    return Status::OK;
}
