#ifndef DEVICE_UTILS_H
#define DEVICE_UTILS_H

#include <cstddef>
#include <inttypes.h>

namespace aditof {
static void deinterleave(const char *source, uint16_t *destination,
                         size_t source_len, size_t dest_width,
                         size_t dest_height) {

    size_t offset[2] = {0, dest_height * dest_width / 2};
    size_t offset_idx = 0;
    size_t j = 0;

    if (dest_width == 668) {
        for (size_t i = 0; i < source_len; i += 3) {
            if ((i != 0) & (i % (336 * 3) == 0)) {
                j -= 4;
            }

            destination[j] =
                (((unsigned short)*(((unsigned char *)source) + i)) << 4) |
                (((unsigned short)*(((unsigned char *)source) + i + 2)) &
                 0x000F);
            j++;

            destination[j] =
                (((unsigned short)*(((unsigned char *)source) + i + 1)) << 4) |
                ((((unsigned short)*(((unsigned char *)source) + i + 2)) &
                  0x00F0) >>
                 4);
            j++;
        }
    } else {
        for (size_t i = 0; i < source_len; i += 3) {

            offset_idx = ((j / dest_width) % 2);

            destination[offset[offset_idx]] =
                (((unsigned short)*(((unsigned char *)source) + i)) << 4) |
                (((unsigned short)*(((unsigned char *)source) + i + 2)) &
                 0x000F);
            offset[offset_idx]++;

            destination[offset[offset_idx]] =
                (((unsigned short)*(((unsigned char *)source) + i + 1)) << 4) |
                ((((unsigned short)*(((unsigned char *)source) + i + 2)) &
                  0x00F0) >>
                 4);
            offset[offset_idx]++;

            j += 2;
        }
    }
}
} // namespace aditof

#endif // DEVICE_UTILS_H
