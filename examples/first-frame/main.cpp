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
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <chrono>
#include <ctime>
#include <glog/logging.h>
#include <iostream>
#include <sys/time.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <sys/time.h>

using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

#include <fstream>
#include <iostream>

using namespace aditof;

FILE *fp = fopen("fps.txt", "a+");
FILE *fp2 = fopen("frame.txt", "a+");

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraList(cameras);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type available!";
        return 0;
    }
    status = camera->setFrameType("depth");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "no camera modes available!";
        return 0;
    }
    status = camera->setMode(modes.front());
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    aditof::Frame frame;
    int j = 0;
    while (j < 10) {
        auto t1 =
            duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();

        status = camera->requestFrame(&frame);
        auto t2 =
            duration_cast<milliseconds>(system_clock::now().time_since_epoch())
                .count();

        std::cout << "FPS: " << 1 / (double)(t2 - t1) * 1000 << std::endl;
        fprintf(fp, "%f, ", 1 / (double)(t2 - t1) * 1000);


        if (j == 9) {
                    uint16_t *data1;
        status = frame.getData(FrameDataType::DEPTH, &data1);
        FrameDetails fDetails;
        frame.getDetails(fDetails);
            for (unsigned int i = 1; i <= fDetails.width * fDetails.height;
                 ++i) {
                // std::cout << data1[i] << " ";
                fprintf(fp2, "%d, ", data1[i]);
                if(i%20==0)
                    fprintf(fp2, "\n");
            }
        }

        j++;
    }

    return 0;
}
