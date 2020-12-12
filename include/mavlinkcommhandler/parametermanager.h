/*
 * Tyler Nguyen 2020
 * Handles the creation, usage, storage, and sending of parameters to be stored and shared over a Mavlink connection
 */

#ifndef LEDCONTROLLER_PARAMETERMANAGER_H
#define LEDCONTROLLER_PARAMETERMANAGER_H


#include <stdint.h>
#include <unordered_map>
#include <vector>
#include "../mavlink/common/mavlink.h" // Mavlink headers
#include "parametermanager.h"

/**
 * List out all the parameters into an enum.  This way they automatically get assigned a unique ID at compile time.
 * By adding P_COUNT at the end, we automatically update at compile time how many parameters we have
 */
typedef enum params_enum {
    // Parameter names
    PID_MAN_INPUT = 0,

    P_COUNT,            // This records the total number of parameters

} params_enum;

class ParameterManager {

private:
    /**
     * Initialize parameters with human-readable names and default float value.
     * Note: Human-readable names should be less than or equal to 17 bytes in size, and should be null-terminated
     */
    void param_init_all() {
        param_init(PID_MAN_INPUT, "PID_MAN_INPUT", 1.4);
    }

/**
 * Initialize a single parameter with a human-readable name and float value. Should only be used at initialization
 * @param PARAM_NAME Parameter index, specified by params_enum
 * @param human_readable_name Human-readable null-terminated parameter name. Must be char array of less than or equal to 17 bytes
 * @param value Float value to initialize parameter
 */
    void param_init(params_enum PARAM_NAME, const char human_readable_name[], float value);

    /**
     * This struct can be used to hold the names and values of all the parameters we currently have.
     * The size is automatically set using the P_COUNT enum value
     */
    struct params_struct {
        char p_name[P_COUNT][17];	// Human-readable names for the parameters that will appear in Mission Planner
        float p_value[P_COUNT];
    };

    // params holds all of the stored parameter human-readable names and values.
    params_struct params;

public:
    /**
     * Set the internal value of a parameter to a specific param_union_extended
     * @param PARAM_NAME Parameter index, specified by params_enum
     * @param float value to set the param to. TODO: Implement mavlink_param_union_t to handle other types
     * @return Float value of new parameter value
     */
    float set_param_value(params_enum PARAM_NAME, float value);

    /**
     * Retrieves the parameter value
     * @param PARAM_NAME Parameter index, specified by params_enum
     * @return Float value of parameter
     */
    float get_param_value(params_enum PARAM_NAME) const;

    /**
     * Retrieve the human-readable null-terminated name for a parameter
     * @param PARAM_NAME Parameter index, specified by params_enum
     * @param destination Pointer to the destination array of at least 17 bytes to store the human-readable name
     * @return Destination is returned
     */
    char* get_param_name(params_enum PARAM_NAME, char *destination) const;


/////////////////////////
//   Mavlink Handlers  //
/////////////////////////

public:
    ParameterManager();
    
}; // ParameterManager



#endif //LEDCONTROLLER_PARAMETERMANAGER_H
