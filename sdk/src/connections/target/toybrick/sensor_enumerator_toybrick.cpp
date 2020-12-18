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
#include "connections/target/target_sensor_enumerator.h"
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

aditof::Status TargetSensorEnumerator::searchSensors() {
    Status status = Status::OK;

    LOG(INFO) << "Looking for sensors on the target";

    // TO DO: Do we still need to do this?
    // Find all media device paths
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
        std::string devPath = "/dev/video2";
	
        if (devPath.empty()) {
            continue;
        }

        DLOG(INFO) << "Considering: "  << video << " an eligible TOF camera";
        // TO DO: Don't guess the device, find a way to identify it so we are sure
        // we've got the right device and is compatible with the SDK
        SensorInfo sInfo;
        sInfo.sensorType = SensorType::SENSOR_ADDI9036;
        sInfo.driverPath = "/dev/video2";
//	sInfo.subDevPath = "/dev/v4l-subdev0";
        m_sensorsInfo.emplace_back(sInfo);

    }

    return status;
}
