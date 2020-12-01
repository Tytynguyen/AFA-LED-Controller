/*  
 *  Tyler Nguyen 2020
 *  LED Controller Board for Autonomous Flying Ambulance LED Indicators
*/
#include "./include/mavlink/common/mavlink.h"
#include "./include/leddriver/leddriver.cpp"
#include "./include/mavlinkcommhandler/mavlinkcommhandler.cpp"

#define DEBUG

// CONFIG
const uint16_t REQUEST_DATA_RATE = 1000; // units: ms

// Timers
long previous_hb = 0;

MavlinkCommHandler comm_handler(Serial, 57600);

void setup() {
  // Change analogWrite resolution to 255, which is default for Uno. Needed for LEDDRIVER library.
  #if defined(ESP8266)
    analogWriteRange(255);
  #endif
}

void loop() {
  if (millis() - previous_hb > REQUEST_DATA_RATE) {
    comm_handler.send_heartbeat();
    previous_hb = millis();
   }
   
    comm_handler.comm_RX();
}
