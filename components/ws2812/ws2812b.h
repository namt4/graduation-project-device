#ifndef __WS2812B_H
#define __WS2812B_H
#include <stdint.h>

void ws2812_init(uint32_t gpio_pin, uint32_t led_number);
void ws2812_write_led(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);
void set_brightness(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t brightness);
void ws2812_set_refresh(void);
void ws2812_set_pixel(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);
void ws2812_clear_led(void);

#endif