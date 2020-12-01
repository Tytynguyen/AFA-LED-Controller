/*
 * Tyler Nguyen 2020
* Handles driving the LEDs
 */
#ifndef LEDDRIVERLIB_LEDDRIVER_H
#define LEDDRIVERLIB_LEDDRIVER_H

#include <stdint.h>
#include "../arduino/Arduino.h"
#include <Arduino.h>

/*
 * Color handlers
 */

typedef enum LEDDRIVER_BRIGHTNESS {
    LEDDRIVER_BRIGHTNESS_OFF = 63,
    LEDDRIVER_BRIGHTNESS_LOW = 127,
    LEDDRIVER_BRIGHTNESS_MEDIUM = 191,
    LEDDRIVER_BRIGHTNESS_HIGH = 255
};

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

/**
 * Stores a single RGB LED which can be illuminated.
 * Can be used with single color LEDs by setting non-existant pins to 255
 */
class LED{
    leddriver_color_t color;
    uint8_t brightness;

    uint8_t redpin;
    uint8_t greenpin;
    uint8_t bluepin;

public:
    // Constructor

    /**
     * Construct an LED object to be used
     * @param redpin Analog out pin for red LED. 255 if non-existant
     * @param greenpin Analog out pin for green LED. 255 if non-existant
     * @param bluepin Analog out pin for blue LED. 255 if non-existant
     */
    LED(uint8_t redpin, uint8_t greenpin, uint8_t bluepin);

/**
 * Illuminate the RGB LED to a specific color and brightness
 * @param color LEDDRIVER_COLORS color enum
 * @param brightness uint8_t of brightnes. 0=off, 255=full
 * @param redpin Output pin of red LED
 * @param greenpin Output pin of green LED
 * @param bluepin Output pin of blue LED
 */
    void illuminate(LEDDRIVER_COLORS color, uint8_t brightness);

    /**
 * Illuminate the RGB LED to a specific color and brightness
 * @param red Red value (0-255)
 * @param green Green value (0-255)
 * @param blue Blue value (0-255)
 * @param brightness (0-255)
 * @param redpin Output pin of red LED
 * @param greenpin Output pin of green LED
 * @param bluepin Output pin of blue LED
 */
    void illuminate(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);

    /**
 * Illuminate all RGBs to LEDDRIVER_BRIGHTNESS_OFF,
 * @param redpin Output pin of red LED
 * @param greenpin Output pin of green LED
 * @param bluepin Output pin of blue LED
 */
    void off();
};




/**
 * Get a prepared color type from the respective color enum
 * @param color LEDDRIVER_COLORS color enum
 * @return leddriver_color_t color struct with RGB elements
 */
static leddriver_color_t _get_color(LEDDRIVER_COLORS color);

#endif //LEDDRIVERLIB_LEDDRIVER_H
