/**
 * Tyler Nguyen 2020
 * Handles communication with MAV through MAVLINK
 */

//#ifndef LEDCONTROLLER_MAVLINKCOMMHANDLER_H
//#define LEDCONTROLLER_MAVLINKCOMMHANDLER_H

#include <stdint.h>
#include "../mavlink/common/mavlink.h" // Mavlink headers
#include "uavstate.h"
#include "../arduino/Arduino.h"
#include "parametermanager.h"
#include <Arduino.h>

#define DEBUG

class MavlinkCommHandler{
    HardwareSerial& comm_serial;
    UAVState drone_state;

    // System Info
    uint8_t SYSTEM_ID = 0; // ID of the target system (flight controller) TBD
    uint8_t TARGET_COMPONENT_ID = 1; // ID of the target component (flight controller)


    // LED Controller Component Info
    const uint8_t COMPONENT_ID = 2; // The ID of sender (i.e. this component)
    const uint8_t COMPONENT_TYPE = MAV_TYPE_ONBOARD_CONTROLLER; // Companion board
    const uint8_t COMPONENT_AUTOPILOT = MAV_AUTOPILOT_INVALID; // Non flight controller
    const uint8_t COMPONENT_MODE = MAV_MODE_PREFLIGHT; // Booting up
    const uint32_t COMPONENT_CUSTOM_MODE = 0; // Custom mode to be defined by user
    const uint8_t COMPONENT_STATE = MAV_STATE_BOOT; // Booting up

    // Timers
    long hb_TX_time = 0;

public:
    // Constructor
    /**
     * Initializes the Mavlink communications handler with a serial stream to listen and post to. Should be the serial
     * over which the LED Controller board is communicating to the Pixhawk.
     * @param serialstream Initialized serial stream. Default: Serial
     * @param baud Baudrate of serial stream. Default: 57600
     */
    MavlinkCommHandler(HardwareSerial& serialstream, int baud);

    /**
     * Send a single heartbeat over comm_serial
     * @return The number of bytes to be sent to serial
     */
    uint16_t send_heartbeat() const;

    /**
     * Receive, parse, and handle data on the comm channel. Call this as often as possible
     * @return True if received new full message, False if no new message
     */
    bool comm_RX();

    /**
     * Request the value of a specific parameter from the flight controller.
     * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars
     * and WITHOUT null termination (NULL) byte if the length is exactly 16 chars
     * @param param_index Send -1 to use the param ID field as identifier (else the param id will be ignored) DEFAULT:-1
     * @return The number of bytes to sent to serial
     */
    uint16_t request_param(char param_id, int16_t param_index) const;


    /**
     * Request the value of a specific parameter from the given target system and component.
     * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars
     * and WITHOUT null termination (NULL) byte if the length is exactly 16 chars
     * @param param_index Send -1 to use the param ID field as identifier (else the param id will be ignored) DEFAULT:-1
     * @param target_system System ID to request parameter from. Default: SYSTEM_ID
     * @param target_component Component ID to request parameter from. Default: 1
     * @return The number of bytes sent to serial
     */
    uint16_t request_param(char param_id, int16_t param_index, uint8_t target_system, uint8_t target_component) const;

    // Accessors
    uint8_t get_system_id() const  {return this->SYSTEM_ID;}
    uint8_t get_target_component_id() const {return this->TARGET_COMPONENT_ID;}

private:
    /**
     * Handles a received mavlink message and updates UAV state accordingly. Called from comm_RX
     * @param RX_time Time that full message was received
     * @param msg Mavlink message data
     */
    void handle_RX_msg(long RX_time, mavlink_message_t& msg);


    //send heartbeat continuously

    //Reroute heartbeats, calls, etc.





};
//#endif //LEDCONTROLLER_MAVLINKCOMMHANDLER_H
