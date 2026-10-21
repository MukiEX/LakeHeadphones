#include "status_led.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#ifndef LHP_WS2812_PIN
#ifdef PICO_DEFAULT_WS2812_PIN
#define LHP_WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
#define LHP_WS2812_PIN -1
#endif
#endif
/* The RP2350-USB-A's LED takes R,G,B byte order (as found on the GunCon 3 adapter). */
#ifndef LHP_WS2812_RGB_ORDER
#define LHP_WS2812_RGB_ORDER 1
#endif
#define LED_PIO pio2

static int s_sm = -1;
static uint32_t s_last = 0xFFFFFFFFu;

void status_led_init(void)
{
#if LHP_WS2812_PIN >= 0
    uint offset = pio_add_program(LED_PIO, &ws2812_program);
    s_sm = (int)pio_claim_unused_sm(LED_PIO, true);
    ws2812_program_init(LED_PIO, (uint)s_sm, offset, LHP_WS2812_PIN, 800000, false);
#endif
}

void status_led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_sm < 0) return;
#if LHP_WS2812_RGB_ORDER
    const uint32_t w = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
#else
    const uint32_t w = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
#endif
    if (w == s_last) return;
    if (pio_sm_is_tx_fifo_full(LED_PIO, (uint)s_sm)) return;   /* never block */
    s_last = w;
    pio_sm_put(LED_PIO, (uint)s_sm, w << 8u);
}
