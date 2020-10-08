/*
 * Tyler Nguyen 2020
 */
#ifndef LEDDRIVERLIB_LEDDRIVER_H
#define LEDDRIVERLIB_LEDDRIVER_H

#include <stdint.h>



/*
 * Color handlers
 */

typedef enum LEDDRIVER_COLORS {
    LEDDRIVER_COLORS_WHITE=0b111, // RGB
    LEDDRIVER_COLORS_RED=0b100, // R
    LEDDRIVER_COLORS_YELLOW=0b110, // RG
    LEDDRIVER_COLORS_GREEN=0b010, // G
    LEDDRIVER_COLORS_BLUE=0b001, // B
    LEDDRIVER_COLORS_CYAN=0b011, // GB
    LEDDRIVER_COLORS_PURPLE=0b101, // RB
    LEDDRIVER_COLORS_BLACK=0b000 // OFF
} LEDDRIVER_COLORS;

typedef struct leddriver_color_t {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} leddriver_color_t;

/*
 * Functions
 */

/**
 * Illuminate the RGB LED to a specific color and brightness
 * @param color LEDDRIVER_COLORS color enum
 * @param brightness uint8_t of brightnes. 0=off, 255=full
 * @param redpin Output pin of red LED
 * @param greenpin Output pin of green LED
 * @param bluepin Output pin of blue LED
 */
static void leddriver_led_illuminate(LEDDRIVER_COLORS color, uint8_t brightness, uint8_t redpin, uint8_t greenpin, uint8_t bluepin);

/**
 * Turn all RGB LEDs off
 * @param redpin Output pin of red LED
 * @param greenpin Output pin of green LED
 * @param bluepin Output pin of blue LED
 */
static void leddriver_led_off(uint8_t redpin, uint8_t greenpin, uint8_t bluepin);

/**
 * Get a prepared color type from the respective color enum
 * @param color LEDDRIVER_COLORS color enum
 * @return leddriver_color_t color struct with RGB elements
 */
static leddriver_color_t _get_color(LEDDRIVER_COLORS color);

#endif //LEDDRIVERLIB_LEDDRIVER_H
