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
