#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

namespace aditof {
class Utils {
  public:
    static uint16_t *getCalibrationData(float gain, float offset,
                                        int16_t maxPixelValue);
    static void applyCalibrationToFrame(uint16_t *calibrationData,
                                        uint16_t *frame, unsigned int width,
                                        unsigned int height);
};
} // namespace aditof

#endif
