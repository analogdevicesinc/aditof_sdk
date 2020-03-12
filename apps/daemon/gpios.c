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
#include "gpios.h"

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define GPIO_PATH "/sys/class/gpio"

int write_int_to_file(int n, const char *filename) {

    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "Failed to open file %s. Reason: %s\n", filename,
                strerror(errno));
        return -1;
    }

    char buf[8];
    snprintf(buf, sizeof(buf), "%i", n);
    size_t buf_len = strlen(buf);
    int ret = fwrite(buf, sizeof(char), buf_len, fp);
    if (ret != buf_len) {
        fprintf(stderr, "Failed to write int: %d to file. Reason: %s\n", n,
                strerror(errno));
    }

    fclose(fp);

    return 0;
}

size_t write_string_to_file(const char *string, const char *filename) {
    
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "Failed to open file %s. Reason: %s\n", filename,
                strerror(errno));
        return 0;
    }
    
    size_t len = strlen(string);
    int ret = fwrite(string, sizeof(char), len, fp);
    if (ret != len) {
        fprintf(stderr, "Failed to write string: %s to file. Reason: %s\n", string,
                strerror(errno));
    }

    fclose(fp);

    return ret;
}

void gpio_export(struct gpio *gpio) {
    write_int_to_file(gpio->id, GPIO_PATH "/export");
}

void gpio_unexport(struct gpio *gpio) {
    write_int_to_file(gpio->id, GPIO_PATH "/unexport");
}

void gpio_init(struct gpio *gpio, int id, bool direction) {
    char tmp_path[128];
    size_t tmp_path_len;

    gpio->id = id;
    gpio->value = -1;

    snprintf(tmp_path, sizeof(tmp_path), GPIO_PATH "/gpio%i", id);
    tmp_path_len = strlen(tmp_path) + 1;
    gpio->location_path = malloc(tmp_path_len);
    memcpy(gpio->location_path, tmp_path, tmp_path_len);

    snprintf(tmp_path, sizeof(tmp_path), GPIO_PATH "/gpio%i/value", id);
    tmp_path_len = strlen(tmp_path) + 1;
    gpio->value_path = malloc(tmp_path_len);
    memcpy(gpio->value_path, tmp_path, tmp_path_len);

    snprintf(tmp_path, sizeof(tmp_path), GPIO_PATH "/gpio%i/direction", id);
    tmp_path_len = strlen(tmp_path) + 1;
    gpio->direction_path = malloc(tmp_path_len);
    memcpy(gpio->direction_path, tmp_path, tmp_path_len);

    gpio_export(gpio);

    if (direction)
        write_string_to_file("in", gpio->direction_path);
    else
        write_string_to_file("out", gpio->direction_path);
}

void gpio_destroy(struct gpio *gpio) {

    gpio_unexport(gpio);

    gpio->id = -1;
    gpio->value = -1;

    free(gpio->location_path);
    gpio->location_path = NULL;

    free(gpio->value_path);
    gpio->value_path = NULL;

    free(gpio->direction_path);
    gpio->direction_path = NULL;
}

void gpio_set_edge(struct gpio *gpio, const char *edge) {
    static const char *edge_fn = "edge";
    size_t edge_fn_len = strlen(gpio->location_path) + strlen(edge_fn) + 2;
    char *edge_filename = malloc(edge_fn_len);
    snprintf(edge_filename, edge_fn_len, "%s/%s", gpio->location_path, edge_fn);

    FILE *fp = fopen(edge_filename, "w");
    free(edge_filename);
    if (!fp) {
        fprintf(stderr, "Failed to open file when setting edge. Reason: %s\n",
                strerror(errno));
        return;
    }

    size_t size = strlen(edge);
    int ret = fwrite(edge, sizeof(char), size, fp);
    if (ret != size) {
        fprintf(stderr, "Failed to write edge: %s to file. Reason: %s\n", edge,
                strerror(errno));
    }

    fclose(fp);
}

void gpio_set_value(struct gpio *gpio, int value)
{
    write_int_to_file(value, gpio->value_path);
}
