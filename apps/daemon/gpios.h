#ifndef GPIOS_H
#define GPIOS_H

#include <stdbool.h>

struct gpio {
    int id;
    int value;
    bool direction;
    char *location_path;
    char *value_path;
    char *direction_path;
};

void gpio_init(struct gpio *gpio, int id, bool direction);
void gpio_destroy(struct gpio *gpio);
void gpio_set_edge(struct gpio *gpio, const char *edge);
void gpio_set_value(struct gpio *gpio, int value);

#endif /* GPIOS_H */
