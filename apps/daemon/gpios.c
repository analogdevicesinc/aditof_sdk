#include "gpios.h"

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define GPIO_PATH "/sys/class/gpio"

size_t write_int_to_file(int n, FILE *fp) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%i", n);
    size_t buf_len = strlen(buf);
    int ret = fwrite(buf, sizeof(char), buf_len, fp);
    if (ret != buf_len) {
        fprintf(stderr, "Failed to write int: %d to file. Reason: %s\n", n,
                strerror(errno));
    }
}

void gpio_init(struct gpio *gpio, int id) {
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
}

void gpio_free(struct gpio *gpio) {
    gpio->id = -1;
    gpio->value = -1;

    free(gpio->location_path);
    gpio->location_path = NULL;

    free(gpio->value_path);
    gpio->value_path = NULL;
}

void gpio_export(struct gpio *gpio) {
    FILE *fp = fopen(GPIO_PATH "/export", "w");
    if (!fp) {
        fprintf(stderr, "Failed to open file when exporting gpio. Reason: %s\n",
                strerror(errno));
        return;
    }
    write_int_to_file(gpio->id, fp);
    fclose(fp);
}

void gpio_unexport(struct gpio *gpio) {
    FILE *fp = fopen(GPIO_PATH "/unexport", "w");
    if (!fp) {
        fprintf(stderr,
                "Failed to open file when un-exporting gpio. Reason: %s\n",
                strerror(errno));
        return;
    }
    write_int_to_file(gpio->id, fp);
    fclose(fp);
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
