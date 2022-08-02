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
#ifndef DEPTHIMAGE_MSG_H
#define DEPTHIMAGE_MSG_H

#include <aditof/frame.h>

#include "aditof_sensor_msg.h"
#include "aditof_utils.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//rainbow color map
#define RED 0
#define INDIGO 275

//saturation and value, for hsv
#define SAT 1.0
#define VAL 1.0

typedef struct Rgba8Color {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
} Rgba8Color;

typedef struct Rgb32Color {
    double r;
    double g;
    double b;
} Rgb32Color;

class DepthImageMsg : public AditofSensorMsg {
  public:
    DepthImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                  aditof::Frame *frame, std::string encoding, ros::Time tStamp);

    /**
     * @brief Each message corresponds to one frame
     */
    sensor_msgs::Image msg;

    /**
     * @brief Will be assigned a value from the list of strings in include/sensor_msgs/image_encodings.h
     */
    std::string imgEncoding;

    /**
     * @brief Converts the frame data to a message
     */
    void FrameDataToMsg(const std::shared_ptr<aditof::Camera> &camera,
                        aditof::Frame *frame, ros::Time tStamp);

    /**
     * @brief Assigns values to the message fields concerning metadata
     */
    void setMetadataMembers(int width, int height, ros::Time tStamp);

    /**
     * @brief Assigns values to the message fields concerning the point data
     */
    void setDataMembers(const std::shared_ptr<aditof::Camera> &camera,
                        uint16_t *frameData);

    /**
     * @brief Converts depth data to RGBA8 color
     */
    void dataToRGBA8(uint16_t min_int, uint16_t max_int, uint16_t *data);

    /**
     * @brief Converts pixel value from HSV to RGBA
     */
    Rgba8Color HSVtoRGBA8(double hue, double sat, double val);
    /**
     * @brief Publishes a message
     */
    void publishMsg(const ros::Publisher &pub);

    void setDepthDataFormat(int value);
    int getDepthDataFormat();

  private:
    DepthImageMsg();
    int m_depthDataFormat; //MONO16 - 0, RGBA8 - 1
};

#endif // DEPTHIMAGE_MSG_H
