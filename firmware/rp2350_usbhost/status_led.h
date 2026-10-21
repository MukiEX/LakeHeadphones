#ifndef STATUS_LED_H
#define STATUS_LED_H
#include <stdint.h>
/* WS2812 status LED on PIO2 (PIO0/PIO1 belong to PIO-USB). No-op without a pin. */
void status_led_init(void);
void status_led_rgb(uint8_t r, uint8_t g, uint8_t b);   /* skipped if unchanged */
#endif
