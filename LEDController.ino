#include "mavlinklib/common/mavlink.h"

//System Info
uint8_t SYSTEM_ID = 1; //ID of the target system (flight controller)
uint8_t TARGET_COMPONENT_ID = 1; //ID of the target component (flight controller)

//LED Controller Component Info
uint8_t COMPONENT_ID = 2; //The ID of sender (i.e. this component)
uint8_t COMPONENT_TYPE = MAV_TYPE_ONBOARD_CONTROLLER; //Companion board
uint8_t COMPONENT_AUTOPILOT = MAV_AUTOPILOT_INVALID; //Non flight controller
uint8_t COMPONENT_MODE = MAV_MODE_PREFLIGHT; //Booting up TODO: Test this vs. MAV_MODE_FLAG_SAFETY_ARMED
uint32_t COMPONENT_CUSTOM_MODE = 0; //Custom mode to be defined by user
uint8_t COMPONENT_STATE = MAV_STATE_BOOT; //Booting up

//CONFIG
uint16_t REQUEST_DATA_RATE = 100; //units: us
uint16_t HB_RATE = 1000; //units: ms
uint16_t REQUEST_BRIGHTNESS_RATE = 100; //units: us

//Timers
long previous_hb = 0;
  
void setup() {
//MAVlink connection to Pixhawk
Serial.begin(57600);
}

void loop() {
  long current_time = millis();
  
  if(current_time - previous_hb > HB_RATE){
    MAV_Send_Heartbeat();
  }
}

//Send a heartbeat to the network
void MAV_Send_Heartbeat(){
  //Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  //Pack the message and send to buffer
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, COMPONENT_TYPE, 
  COMPONENT_AUTOPILOT, COMPONENT_MODE, COMPONENT_CUSTOM_MODE, COMPONENT_STATE);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Transmit heartbeat
  Serial.write(buf, len);
  previous_hb = millis();
}

//Initialize incoming data streams from flight controller
void MAV_Init_Data_Streams(){  
  //Message buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Request brightness parameter
  MAV_CMD_SET_MESSAGE_INTERVAL;
  uint16_t len = mavlink_msg_command_int_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
  SYSTEM_ID, TARGET_COMPONENT_ID, MAV_FRAME_LOCAL_NED, MAV_CMD_SET_MESSAGE_INTERVAL, 
  1, 0, MAVLINK_MSG_ID_PARAM_REQUEST_READ, REQUEST_BRIGHTNESS_RATE, 0, 0, 0, 0, 0);
}

//Receive incoming data
void Comm_Receive() {
  mavlink_message_t msg; //Message buffer
  mavlink_status_t status; //Parsing status

  while(Serial.available()>0) {
    uint8_t c = Serial.read();

    //Try to parse new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      //Handle message if fully received
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);

//            mySerial.println("PX PARAM_VALUE");
//            mySerial.println(param_value.param_value);
//            mySerial.println(param_value.param_count);
//            mySerial.println(param_value.param_index);
//            mySerial.println(param_value.param_id);
//            mySerial.println(param_value.param_type);
//            mySerial.println("------ Fin -------");

//            //Handle brightness parameter
//            if(param_value.param_index == MAVLINK_MSG_ID_PARAM_VALUE){
//              //HANDLE LEDS HERE
//            }
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
          }
          break;
        
       default:
          break;
      }
    }
  }
}
