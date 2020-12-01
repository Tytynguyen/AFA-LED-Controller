/*
 * Tyler Nguyen 2020
 * Handles driving the LEDs
 */

#include "leddriver.h"


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

LED::LED(uint8_t redpin = 255, uint8_t greenpin = 255, uint8_t bluepin = 255) {
    this->redpin = redpin;
    this->greenpin = greenpin;
    this->bluepin = bluepin;
}

void LED::illuminate(LEDDRIVER_COLORS color, uint8_t brightness) {
    this->color = _get_color(color);
    this->brightness = brightness;

    analogWrite(this->redpin, this->color.red & this->brightness);
    analogWrite(this->greenpin, this->color.green & this->brightness);
    analogWrite(this->bluepin, this->color.blue & this->brightness);
}

void LED::illuminate(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
    this->color = leddriver_color_t{red, green, blue};
    this->brightness = brightness;

    analogWrite(this->redpin, this->color.red & this->brightness);
    analogWrite(this->greenpin, this->color.green & this->brightness);
    analogWrite(this->bluepin, this->color.blue & this->brightness);
}

void LED::off(){
    this->brightness = LEDDRIVER_BRIGHTNESS_OFF;

    analogWrite(this->redpin, this->color.red & this->brightness);
    analogWrite(this->greenpin, this->color.green & this->brightness);
    analogWrite(this->bluepin, this->color.blue & this->brightness);
}
