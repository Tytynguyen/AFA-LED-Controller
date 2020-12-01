/*
 * Tyler Nguyen 2020
 * Holds UAV state info
 */

#ifndef LEDCONTROLLER_UAVSTATE_H
#define LEDCONTROLLER_UAVSTATE_H

#include "../mavlink/common/mavlink.h" // Mavlink headers

class UAVState{
    long rx_time;
    mavlink_heartbeat_t UAV_hb;
    mavlink_sys_status_t UAV_status;

public:
// Constructors
    UAVState(long time);
    UAVState(long time, mavlink_heartbeat_t hb, mavlink_sys_status_t sys_status);

// Setters
    void update_state(long RX_time, mavlink_heartbeat_t hb);
    void update_state(long RX_time, mavlink_sys_status_t status);

// Accessors
    const long time() const;

    // Armed states
    const bool armed() const; // TRUE for any armed state

    // Global Positioning System (GPS) states
    const bool gps_ready() const; // TRUE iff GPS is present, enabled, and healthy

    //Altitude Heading Reference System (AHRS) states
    const bool ahrs_ready() const; // TRUE iff AHRS is present, enabled, and healthy

    //Battery states
    const bool batt_ready() const; // TRUE iff BATT is present, enabled, and healthy

    //Radio Control (RC) Receiver states
    const bool rc_receiver_ready() const; // TRUE iff RC RECEIVER is present, enabled, and healthy

};

#endif //LEDCONTROLLER_UAVSTATE_H
