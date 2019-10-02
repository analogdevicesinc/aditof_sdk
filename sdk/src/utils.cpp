#include "utils.h"

uint16_t *aditof::Utils::buildCalibrationCache(float gain, float offset,
                                               int16_t maxPixelValue) {
    uint16_t *cache = new uint16_t[maxPixelValue + 1];
    for (int16_t current = 0; current <= maxPixelValue; ++current) {
        cache[current] =
            static_cast<int16_t>(static_cast<float>(current) * gain + offset);
    }
    return cache;
}

void aditof::Utils::calibrateFrame(uint16_t *calibrationData, uint16_t *frame,
                                   unsigned int width, unsigned int height) {
    uint16_t *cache = calibrationData;

    unsigned int size = width * height / 2;

    uint16_t *end = frame + (size - size % 8);
    uint16_t *framePtr = frame;

    for (; framePtr < end; framePtr += 8) {
        *framePtr = *(cache + *framePtr);
        *(framePtr + 1) = *(cache + *(framePtr + 1));
        *(framePtr + 2) = *(cache + *(framePtr + 2));
        *(framePtr + 3) = *(cache + *(framePtr + 3));
        *(framePtr + 4) = *(cache + *(framePtr + 4));
        *(framePtr + 5) = *(cache + *(framePtr + 5));
        *(framePtr + 6) = *(cache + *(framePtr + 6));
        *(framePtr + 7) = *(cache + *(framePtr + 7));
    }

    end += (size % 8);

    for (; framePtr < end; framePtr++) {
        *framePtr = *(cache + *framePtr);
    }
}
