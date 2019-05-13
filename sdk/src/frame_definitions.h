#ifndef FRAME_DEFINITIONS_H
#define FRAME_DEFINITIONS_H

#include <string>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum FrameDataType
 * @brief Types of data that a frame can contain
 */
enum class FrameDataType {
    RAW,   //!< Raw information
    DEPTH, //!< Depth information
    IR,    //!< Infrared information
};

/**
 * @struct FrameCalData
 * @brief Stores the calibration data to be applied to the frame.
 */
struct FrameCalData {
    float offset; //!< Offset information
    float gain;   //!< Gain information
};

/**
 * @struct FrameDetails
 * @brief Describes the properties of a frame.
 */
struct FrameDetails {
    /**
     * @brief The width of the frame.
     */
    unsigned int width;

    /**
     * @brief The height of the frame.
     */
    unsigned int height;

    /**
     * @brief The type of the frame. Can be one of the types provided by the
     * camera.
     */
    std::string type;

    /**
     * @brief Calibration data for the frame
     */
    struct FrameCalData cal_data;
};

} // namespace aditof

#endif // FRAME_DEFINITIONS_H
