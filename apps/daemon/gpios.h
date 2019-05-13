#ifndef GPIOS_H
#define GPIOS_H

struct gpio {
    int id;
    int value;
    char *location_path;
    char *value_path;
};

void gpio_init(struct gpio *gpio, int id);
void gpio_free(struct gpio *gpio);
void gpio_export(struct gpio *gpio);
void gpio_unexport(struct gpio *gpio);
void gpio_set_edge(struct gpio *gpio, const char *edge);

#endif /* GPIOS_H */
