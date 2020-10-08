/*
 * Tyler Nguyen 2020
 */
#include <stdint.h>
#include "leddriver.h"
#include <arduino.h>

leddriver_color_t _get_color(LEDDRIVER_COLORS color) {
    uint8_t val[3]; // RGB

    for(uint8_t i = 0; i<3 ; i++){
        if(color & 1<<i){
            val[2-i] = UINT8_MAX;
        } else{
            val[2-i] = 0;
        }
    }
    return leddriver_color_t{val[0], val[1], val[2]};
}

void leddriver_led_illuminate(LEDDRIVER_COLORS color, uint8_t brightness, uint8_t redpin, uint8_t greenpin,
                                         uint8_t bluepin) {
    leddriver_color_t color_type = _get_color(color);

    analogWrite(redpin, color_type.red&brightness);
    analogWrite(greenpin, color_type.green&brightness);
    analogWrite(bluepin, color_type.blue&brightness);
}

void leddriver_led_off(uint8_t redpin, uint8_t greenpin, uint8_t bluepin){
    analogWrite(redpin, 0);
    analogWrite(greenpin, 0);
    analogWrite(bluepin, 0);
}
