/*
 * Tyler Nguyen 2020
 * Handles communication with MAV through MAVLINK
 */

#include "mavlinkcommhandler.h"

MavlinkCommHandler::MavlinkCommHandler(HardwareSerial& serialstream = Serial, int baud = 57600) :
comm_serial(serialstream), drone_state(millis()){
    comm_serial.begin(baud);
    this->comm_serial = serialstream;
}

/**
 * Send a single heartbeat on mavlink connection.
 * @return The number of bytes to be sent to serial
 */
uint16_t MavlinkCommHandler::send_heartbeat() const {
    // Message buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message and send to buffer
    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, COMPONENT_TYPE,
                               COMPONENT_AUTOPILOT, COMPONENT_MODE, COMPONENT_CUSTOM_MODE, COMPONENT_STATE);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Transmit heartbeat
    comm_serial.write(buf, len);

    return len;
}

/**
 * Receive, parse, and handle data on the comm channel. Call this as often as possible
 * @return True if received new full message, False if no new message
 */
bool MavlinkCommHandler::comm_RX() {
    mavlink_message_t msg; // Message buffer
    mavlink_status_t status; // Parsing status

    while (comm_serial.available() > 0) {
        uint8_t c = comm_serial.read();

        // Try to parse new message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            handle_RX_msg(millis(), msg);
            return true;
        }
        return false;
    }
}

/**
 * Handles a received mavlink message and updates UAV state accordingly. Called from comm_RX
 * @param RX_time Time that full message was received
 * @param msg Mavlink message data
 */
void MavlinkCommHandler::handle_RX_msg(long RX_time, mavlink_message_t &msg) {
    // LED Controller will choose the drone with the lowest system ID as the message target
    if (msg.sysid != SYSTEM_ID) {
        // Ignore messages that aren't from an aerial vehicle TODO: Handle routing
        switch (msg.sysid) {
            case MAV_TYPE_ANTENNA_TRACKER:{ return; }
            case MAV_TYPE_GCS:{ return; }
            case MAV_TYPE_ONBOARD_CONTROLLER:{ return; }
            case MAV_TYPE_GIMBAL:{ return; }
            case MAV_TYPE_ADSB:{ return; }
            case MAV_TYPE_CAMERA:{ return; }
            case MAV_TYPE_CHARGING_STATION:{ return; }
            case MAV_TYPE_FLARM:{ return; }
            case MAV_TYPE_SERVO:{ return; }
        }

        // If first contact, or if the received message comes from a drone with a lower system ID, set as target
        // Otherwise, ignore message entirely
        if (SYSTEM_ID == 0 || msg.sysid < SYSTEM_ID){
            SYSTEM_ID = msg.sysid;

            #ifdef DEBUG
                comm_serial.print("\nSYSTEM_ID set to: ");
                comm_serial.println(SYSTEM_ID);
            #endif
        } else {
            return;
        }
    }

    // If the message comes from the target system and the flight controller (component 1), then
    // handle the message appropriately, according to what it contains
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
        {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                drone_state.update_state(RX_time,heartbeat);

                #ifdef DEBUG
                    comm_serial.println("<<<\nRX new heartbeat!");
                    comm_serial.print("armed: ");
                    comm_serial.println(drone_state.armed());
                    comm_serial.println(">>>");
                #endif
            }
            break;

            case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
            {
                mavlink_sys_status_t sys_status;
                mavlink_msg_sys_status_decode(&msg, &sys_status);
                drone_state.update_state(RX_time, sys_status);

                #ifdef DEBUG
                    comm_serial.println("<<<\nRX new status!");
                    comm_serial.print("gps: ");
                    comm_serial.println(drone_state.gps_ready());
                    comm_serial.print("ahrs: ");
                    comm_serial.println(drone_state.ahrs_ready());
                    comm_serial.print("batt: ");
                    comm_serial.println(drone_state.batt_ready());
                    comm_serial.print("RC_rx: ");
                    comm_serial.println(drone_state.rc_receiver_ready());
                    comm_serial.println(">>>");
                #endif

            }
        break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
        {
            mavlink_param_value_t param;
            mavlink_msg_param_value_decode(&msg, &param);

            #ifdef DEBUG
                comm_serial.println("<<<\nRX new param!");
                comm_serial.print("id: ");
                comm_serial.println(param.param_id);
                comm_serial.print("index: ");
                comm_serial.println(param.param_index);
                comm_serial.print("value: ");
                comm_serial.println(param.param_value);
                comm_serial.println(">>>");
            #endif
        }
        break;

        default:
            break;
    }
}

uint16_t MavlinkCommHandler::request_param(const char param_id, int16_t param_index = -1) const {
    return request_param(param_id,param_index, this->get_system_id(), this->get_target_component_id());
}

uint16_t MavlinkCommHandler::request_param(const char param_id, int16_t param_index, uint8_t target_system, uint8_t target_component) const {
    // Message buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_request_read_pack(SYSTEM_ID, COMPONENT_ID, &msg, target_system, target_component, &param_id, param_index);

    // Send to buffer and transmit
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    comm_serial.write(buf, len);
    return len;
}
