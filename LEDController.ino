/*  
 *  Tyler Nguyen 2020
 *  LED Controller Board for Autonomous Flying Ambulance LED Indicators
*/
#include "mavlinklib/common/mavlink.h"
#include "leddriverlib/leddriver.cpp"

#define DEBUG

// System Info
const uint8_t SYSTEM_ID = 1; // ID of the target system (flight controller)
const uint8_t TARGET_COMPONENT_ID = 1; // ID of the target component (flight controller)

// LED Controller Component Info
const uint8_t COMPONENT_ID = 2; // The ID of sender (i.e. this component)
const uint8_t COMPONENT_TYPE = MAV_TYPE_ONBOARD_CONTROLLER; // Companion board
const uint8_t COMPONENT_AUTOPILOT = MAV_AUTOPILOT_INVALID; // Non flight controller
const uint8_t COMPONENT_MODE = MAV_MODE_PREFLIGHT; // Booting up TODO: Test this vs. MAV_MODE_FLAG_SAFETY_ARMED
const uint32_t COMPONENT_CUSTOM_MODE = 0; // Custom mode to be defined by user
const uint8_t COMPONENT_STATE = MAV_STATE_BOOT; // Booting up

// LED pins
const uint8_t LED_RED = 14;
const uint8_t LED_GREEN = 12;
const uint8_t LED_BLUE = 13;


// CONFIG
const uint16_t REQUEST_DATA_RATE = 1000; // units: ms
const uint16_t LED_BLINK_RATE = 1000; // units:ms (on/off timing)


// Timers
long previous_hb = 0;
long previous_blink = 0;

// MAV State (when applicable: True = ready, False = not preset/healthy/enabled)
uint8_t led_brightness = 3;
bool armed = false;
bool gps = false;
bool ahrs = false;
bool batt = false;
bool rc_receiver = false;



void setup() {
  // Change analogWrite resolution to 255, such as for Uno. Needed for LEDDRIVER library.
  #if defined(ESP8266)
    analogWriteRange(255);
  #endif
  
  // MAVlink connection to Pixhawk
  Serial.begin(57600);
}

void loop() {
  if (millis() - previous_hb > REQUEST_DATA_RATE) {
    MAV_Send_Heartbeat();
    MAV_Request_Brightness();
   }

  Drive_LEDs();
  Comm_Receive();

 
}

/*
 * Drive the LEDs based upon the state of the AFA
 */
void Drive_LEDs(){
  if(led_brightness == 0){
        leddriver_led_off(LED_RED, LED_GREEN, LED_BLUE);
  }
  
  uint8_t adj_brightness = led_brightness*85;


if (armed){ // Solid color
  if (gps){ // Armed and GPS = Solid Green
    leddriver_led_illuminate(LEDDRIVER_COLORS_GREEN, adj_brightness, LED_RED, LED_GREEN, LED_BLUE);
  } else { // Armed and no GPS = Solid Blue
        leddriver_led_illuminate(LEDDRIVER_COLORS_BLUE, adj_brightness, LED_RED, LED_GREEN, LED_BLUE);
  }
  return;
}

if(!ahrs || !rc_receiver){ // ERROR = blinking red.
  if (millis() - previous_blink < LED_BLINK_RATE){
      leddriver_led_illuminate(LEDDRIVER_COLORS_RED, adj_brightness, LED_RED, LED_GREEN, LED_BLUE);
  }else{
    leddriver_led_off(LED_RED, LED_GREEN, LED_BLUE);
  }

  if (millis() - previous_blink > LED_BLINK_RATE*2){
    previous_blink = millis();
  }
  return;
}

if (!batt){ // Low Battery = Solid Orange
  leddriver_led_illuminate(LEDDRIVER_COLORS_YELLOW, adj_brightness, LED_RED, LED_GREEN, LED_BLUE);
  return;
}

if (!armed){ // Ready = Blinking color
  if (gps){ // Ready and GPS = Blinking Green
    if (millis() - previous_blink < LED_BLINK_RATE){
      leddriver_led_illuminate(LEDDRIVER_COLORS_GREEN, adj_brightness, LED_RED, LED_GREEN, LED_BLUE);
    }else{
      leddriver_led_off(LED_RED, LED_GREEN, LED_BLUE);
    }
  
    if (millis() - previous_blink > LED_BLINK_RATE*2){
      previous_blink = millis();
    }
    return;
    
  } else { // Ready and no GPS = Blinking Blue
    if (millis() - previous_blink < LED_BLINK_RATE){
      leddriver_led_illuminate(LEDDRIVER_COLORS_BLUE, adj_brightness, LED_RED, LED_GREEN, LED_BLUE);
    }else {
      leddriver_led_off(LED_RED, LED_GREEN, LED_BLUE);
    }
  
    if (millis() - previous_blink > LED_BLINK_RATE*2){
      previous_blink = millis();
    }
    return;
  }
}

}

/*
   Send a heartbeat to the network
*/
void MAV_Send_Heartbeat() {
  // Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message and send to buffer
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, COMPONENT_TYPE,
                             COMPONENT_AUTOPILOT, COMPONENT_MODE, COMPONENT_CUSTOM_MODE, COMPONENT_STATE);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Transmit heartbeat
  Serial.write(buf, len);
  previous_hb = millis();
}

/*
   Request brightness param using param_request_read (one-shot)
*/
void MAV_Request_Brightness() {
  // Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  /**
    @brief Pack a param_request_read message
    @param system_id ID of this system
    @param component_id ID of this component (e.g. 200 for IMU)
    @param msg The MAVLink message to compress the data into

    @param target_system  System ID
    @param target_component  Component ID
    @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars
      and WITHOUT null termination (NULL) byte if the length is exactly 16 chars
    @param param_index  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
  */
  mavlink_msg_param_request_read_pack(SYSTEM_ID, COMPONENT_ID, &msg, SYSTEM_ID, TARGET_COMPONENT_ID, "NTF_LED_BRIGHT\0", -1);

  // Send to buffer and transmit
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

/* ArduPilot v4.0+
 * Initialize incoming data streams from flight controller using MAV_CMD_SET_MESSAGE_INTERVAL
 * TODO: Test this
 */
void MAV_Init_Data_Streams() {
  // Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request brightness parameter
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                SYSTEM_ID, TARGET_COMPONENT_ID, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                MAVLINK_MSG_ID_PARAM_REQUEST_READ, REQUEST_DATA_RATE*1000, 0, 0, 0, 0, 0);
}

/*
   Receive a message.
   TODO: Maybe protect against a loss of connection by saving the ongoing message elsewhere?
*/
void Comm_Receive() {
  mavlink_message_t msg; // Message buffer
  mavlink_status_t status; // Parsing status

  while (Serial.available() > 0) {
    uint8_t c = Serial.read();

    // Try to parse new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message if fully received, otherwise break
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // read heartbeat if from flight controller
            if (msg.sysid == SYSTEM_ID & msg.compid == TARGET_COMPONENT_ID) {
              mavlink_heartbeat_t heartbeat;
              mavlink_msg_heartbeat_decode(&msg, &heartbeat);

              armed = (heartbeat.base_mode - 1 == MAV_MODE_STABILIZE_ARMED ||
                       heartbeat.base_mode - 1 == MAV_MODE_MANUAL_ARMED ||
                       heartbeat.base_mode - 1 == MAV_MODE_GUIDED_ARMED ||
                       heartbeat.base_mode - 1 == MAV_MODE_AUTO_ARMED ||
                       heartbeat.base_mode - 1 == MAV_MODE_TEST_ARMED);

#ifdef DEBUG
//              Serial.println("\nHEARTBEAT");
//              Serial.println(msg.sysid);
//              Serial.println(msg.compid);
//              Serial.println(heartbeat.type);
//              Serial.println(heartbeat.autopilot);
//              Serial.println(heartbeat.base_mode);
//              Serial.println(heartbeat.custom_mode);
//              Serial.println(heartbeat.system_status);
//              Serial.println(heartbeat.mavlink_version);
//              Serial.println("------ Fin -------");

              Serial.print("Armed: ");
              Serial.println(armed);
#endif
            }
            break;

          case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
            {
              mavlink_sys_status_t sys_status;
              mavlink_msg_sys_status_decode(&msg, &sys_status);

              // Check for flight readiness (Could combine into one statement with all parts

              gps = MAV_SYS_STATUS_SENSOR_GPS & sys_status.onboard_control_sensors_present &
                    sys_status.onboard_control_sensors_enabled &
                    sys_status.onboard_control_sensors_health;

              ahrs = MAV_SYS_STATUS_AHRS & sys_status.onboard_control_sensors_present &
                     sys_status.onboard_control_sensors_enabled &
                     sys_status.onboard_control_sensors_health;


              batt = MAV_SYS_STATUS_SENSOR_BATTERY & sys_status.onboard_control_sensors_present &
                     sys_status.onboard_control_sensors_enabled &
                     sys_status.onboard_control_sensors_health;
                     
              rc_receiver = MAV_SYS_STATUS_SENSOR_RC_RECEIVER & sys_status.onboard_control_sensors_present &
                     sys_status.onboard_control_sensors_enabled &
                     sys_status.onboard_control_sensors_health;
            }
#ifdef DEBUG
            Serial.print("GPS: ");
            Serial.println(gps);
            Serial.print("AHRS: ");
            Serial.println(ahrs);
            Serial.print("BATT: ");
            Serial.println(batt);
            Serial.print("RC_RECEIVER: ");
            Serial.println(rc_receiver);
#endif

          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            mavlink_param_value_t param;
            mavlink_msg_param_value_decode(&msg, &param);
            
            if(param.param_value < 4){
                led_brightness = param.param_value;
            }

#ifdef DEBUG
//            Serial.println("\nPX PARAM_VALUE");
//            Serial.println(param.param_value);
//            Serial.println(param.param_count);
//            Serial.println(param.param_index);
//            Serial.println(param.param_id);
//            Serial.println(param.param_type);
//            Serial.println("------ Fin -------");
              Serial.print("LED BRIGHTNESS: ");
              Serial.println(led_brightness);
#endif
          }
          break;

        default:
          break;
      }
    }
  }
}
