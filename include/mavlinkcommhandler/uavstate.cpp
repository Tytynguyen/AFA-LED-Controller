/*
 * Tyler Nguyen 2020
 * Holds UAV state info
 */

#include "uavstate.h"

// Constructors
UAVState::UAVState(long time){
    this->rx_time = time;
}
UAVState::UAVState(long time, mavlink_heartbeat_t hb, mavlink_sys_status_t sys_status) {
    this->rx_time = time;
    this->UAV_hb = hb;
    this->UAV_status = sys_status;
}

// Setters
void UAVState::update_state(long RX_time, mavlink_heartbeat_t hb) {
    this->rx_time = RX_time;
    this->UAV_hb = hb;
}

void UAVState::update_state(long RX_time, mavlink_sys_status_t status) {
    this->rx_time = RX_time;
    this->UAV_status = status;
}

// Accessors
const long UAVState::time() const { return this->rx_time; }

const bool UAVState::armed() const {
    return (this->UAV_hb.base_mode - 1 == MAV_MODE_MANUAL_ARMED ||
            this->UAV_hb.base_mode - 1 == MAV_MODE_TEST_ARMED ||
            this->UAV_hb.base_mode - 1 == MAV_MODE_STABILIZE_ARMED ||
            this->UAV_hb.base_mode - 1 == MAV_MODE_GUIDED_ARMED ||
            this->UAV_hb.base_mode - 1 == MAV_MODE_AUTO_ARMED
    );
}

const bool UAVState::gps_ready() const {
    return (MAV_SYS_STATUS_SENSOR_GPS &
            this->UAV_status.onboard_control_sensors_present &
            this->UAV_status.onboard_control_sensors_enabled &
            this->UAV_status.onboard_control_sensors_health);
}

const bool UAVState::ahrs_ready() const {
    return (MAV_SYS_STATUS_AHRS &
            this->UAV_status.onboard_control_sensors_present &
            this->UAV_status.onboard_control_sensors_enabled &
            this->UAV_status.onboard_control_sensors_health);
}

const bool UAVState::batt_ready() const {
    return (MAV_SYS_STATUS_SENSOR_BATTERY &
            this->UAV_status.onboard_control_sensors_present &
            this->UAV_status.onboard_control_sensors_enabled &
            this->UAV_status.onboard_control_sensors_health);
}

const bool UAVState::rc_receiver_ready() const {
    return (MAV_SYS_STATUS_SENSOR_RC_RECEIVER &
            this->UAV_status.onboard_control_sensors_present &
            this->UAV_status.onboard_control_sensors_enabled &
            this->UAV_status.onboard_control_sensors_health);
}



