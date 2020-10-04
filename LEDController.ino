#include "mavlinklib/common/mavlink.h"

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

// CONFIG
const uint16_t REQUEST_DATA_RATE = 100; // units: us
const uint16_t HB_RATE = 1000; // units: ms
const uint16_t REQUEST_BRIGHTNESS_RATE = 100; // units: us


// Timers
long previous_hb = 0;

// MAV State
bool armed = false;

  
void setup() {
// MAVlink connection to Pixhawk
Serial.begin(57600);
}

void loop() {
  long current_time = millis();
  
  if(current_time - previous_hb > HB_RATE){
    MAV_Send_Heartbeat();
    MAV_Request_Brightness();
  }

  Comm_Receive();
}

/*
 * Send a heartbeat to the network
 */
void MAV_Send_Heartbeat(){
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
 * Request brightness param using param_request_read (one-shot)
 */
void MAV_Request_Brightness(){
  // Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  /**
 * @brief Pack a param_request_read message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_index  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
 */
  mavlink_msg_param_request_read_pack(SYSTEM_ID, COMPONENT_ID, &msg, SYSTEM_ID, TARGET_COMPONENT_ID, "NTF_LED_BRIGHT\0", -1);

  // Send to buffer and transmit
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

// ArduPilot v4.0+
// Initialize incoming data streams from flight controller using MAV_CMD_SET_MESSAGE_INTERVAL
// TODO: Test this
void MAV_Init_Data_Streams(){  
  // Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request brightness parameter
/**
 * @brief Pack a command_long message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System which should execute the command
 * @param target_component  Component which should execute the command, 0 for all components
 * @param command  Command ID (of command to send).
 * @param confirmation  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * 
 * @param param1  Parameter 1 (for the specific command).
 * @param param2  Parameter 2 (for the specific command).
 * @param param3  Parameter 3 (for the specific command).
 * @param param4  Parameter 4 (for the specific command).
 * @param param5  Parameter 5 (for the specific command).
 * @param param6  Parameter 6 (for the specific command).
 * @param param7  Parameter 7 (for the specific command).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
  SYSTEM_ID, TARGET_COMPONENT_ID, MAV_CMD_SET_MESSAGE_INTERVAL, 0, 
  MAVLINK_MSG_ID_PARAM_REQUEST_READ, REQUEST_BRIGHTNESS_RATE,0,0,0,0,0);
}

/*
 * Receive a message.
 * TODO: Maybe protect against a loss of connection by saving the ongoing message elsewhere?
 */
void Comm_Receive() {
  mavlink_message_t msg; // Message buffer
  mavlink_status_t status; // Parsing status

  while(Serial.available()>0) {
    uint8_t c = Serial.read();

    // Try to parse new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message if fully received, otherwise break
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // read heartbeat
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);

            armed = (heartbeat.base_mode-1 == MAV_MODE_STABILIZE_ARMED || 
              heartbeat.base_mode-1 == MAV_MODE_MANUAL_ARMED ||
              heartbeat.base_mode-1 == MAV_MODE_GUIDED_ARMED ||
              heartbeat.base_mode-1 == MAV_MODE_AUTO_ARMED ||
              heartbeat.base_mode-1 == MAV_MODE_TEST_ARMED);
            
            #ifdef DEBUG
            if(msg.sysid == 1 & msg.compid == 1){
              Serial.println("\nHEARTBEAT");
              Serial.println(msg.sysid);
              Serial.println(msg.compid);
              Serial.println(heartbeat.type);
              Serial.println(heartbeat.autopilot);
              Serial.println(heartbeat.base_mode);
              Serial.println(heartbeat.custom_mode);
              Serial.println(heartbeat.system_status);
              Serial.println(heartbeat.mavlink_version);
              Serial.println("------ Fin -------");

              Serial.println("\nArmed: ");
              Serial.println(armed);
            #endif

          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);

          #ifdef DEBUG
            Serial.println("\nPX PARAM_VALUE");
            Serial.println(param_value.param_value);
            Serial.println(param_value.param_count);
            Serial.println(param_value.param_index);
            Serial.println(param_value.param_id);
            Serial.println(param_value.param_type);
            Serial.println("------ Fin -------");
          #endif

//            // Handle brightness parameter
//            if(param_value.param_index == MAVLINK_MSG_ID_PARAM_VALUE){
//              // HANDLE LEDS HERE
//            }
          }
          break;
        
       default:
          break;
      }
    }
  }
}
