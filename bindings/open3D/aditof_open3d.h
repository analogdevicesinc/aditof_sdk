#ifndef ADITOF_OPEN3D_H
#define ADITOF_OPEN3D_H

#include <Open3D/Open3D.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>

using namespace open3d;

namespace aditof {
Status fromFrameToDepthImg(Frame &frame, int camera_range,
                           geometry::Image &image) {
    FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height) / 2;
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *depthData;
    frame.getData(FrameDataType::DEPTH, &depthData);

    if (depthData == nullptr) {
        return Status::GENERIC_ERROR;
    }

    image.Prepare(frameWidth, frameHeight, 1, 1);
    for (int i = 0; i < frameHeight * frameWidth; i++) {
        uint8_t *p =
            static_cast<uint8_t *>(image.data_.data() + i * sizeof(uint8_t));
        uint16_t value = *(depthData + i) * 255.0 / camera_range;
        *p = static_cast<uint8_t>(value <= 255 ? value : 255);
        p++;
    }

    return Status::OK;
}

Status fromFrameToIRImg(Frame &frame, int camera_range,
                        geometry::Image &image) {
    FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    const int frameHeight = static_cast<int>(frameDetails.height) / 2;
    const int frameWidth = static_cast<int>(frameDetails.width);

    uint16_t *irData;
    frame.getData(FrameDataType::IR, &irData);

    if (irData == nullptr) {
        return Status::GENERIC_ERROR;
    }

    image.Prepare(frameWidth, frameHeight, 1, 1);
    for (int i = 0; i < frameHeight * frameWidth; i++) {
        uint8_t *p =
            static_cast<uint8_t *>(image.data_.data() + i * sizeof(uint8_t));
        uint16_t value = *(irData + i) * 255.0 / camera_range;
        *p = static_cast<uint8_t>(value <= 255 ? value : 255);
        p++;
    }

    return Status::OK;
}
} // namespace aditof

#endif // ADITOF_OPEN3D_H
