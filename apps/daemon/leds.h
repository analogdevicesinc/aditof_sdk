#ifndef LEDS_H
#define LEDS_H

void set_user_led_brightness(int led_id, unsigned char brightness);
int get_user_led_brightness(int led_id);

#endif /* LEDS_H */
