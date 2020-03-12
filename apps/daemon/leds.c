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
#include "leds.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USER_LED_PATH "/sys/class/leds/apq8016-sbc:green:user"

void set_user_led_brightness(int led_id, unsigned char brightness) {

    char path[128];

    snprintf(path, sizeof(path), USER_LED_PATH "%d/brightness", led_id);

    FILE *fp = fopen(path, "w");
    if (!fp) {
        fprintf(
            stderr,
            "Failed to open file when settings led %d brightness. Reason: %s\n",
            led_id, strerror(errno));
        return;
    }

    char buf[8];
    snprintf(buf, sizeof(buf), "%i", brightness);
    size_t buf_len = strlen(buf);
    int ret = fwrite(buf, sizeof(char), buf_len, fp);
    if (ret != buf_len) {
        fprintf(stderr, "Failed to write brightness: %d to file. Reason: %s\n",
                brightness, strerror(errno));
    }

    fclose(fp);
}

int get_user_led_brightness(int led_id) {

    char path[128];

    snprintf(path, sizeof(path), USER_LED_PATH "%d/brightness", led_id);

    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(
            stderr,
            "Failed to open file when settings led %d brightness. Reason: %s\n",
            led_id, strerror(errno));
        return -1;
    }

    char buf[8];

    int ret = fread(buf, sizeof(char), sizeof(buf), fp);

    if (ret <= 0) {
        fprintf(stderr, "Failed to read brightness from file. Reason: %s\n",
                strerror(errno));
    }

    fclose(fp);

    int brightness = atoi(buf);

    return brightness;
}
