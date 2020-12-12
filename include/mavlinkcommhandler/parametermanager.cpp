/*
 * Tyler Nguyen 2020
 * Handles the creation, usage, storage, and sending of parameters to be stored and shared over a Mavlink connection
 */

#include "parametermanager.h"


float ParameterManager::set_param_value(params_enum PARAM_NAME, float value) {
    this->params.p_value[PARAM_NAME] = value;
    return get_param_value(PARAM_NAME);
}

float ParameterManager::get_param_value(params_enum PARAM_NAME) const {
    return this->params.p_value[PARAM_NAME];
}



void ParameterManager::param_init(params_enum PARAM_NAME, const char *human_readable_name, float value) {
    if(strlen(human_readable_name)>16){
        throw "Human-readable name must be less or equal to 17 characters long, including null character.";
    }
    strcpy(params.p_name[PARAM_NAME], human_readable_name);
    set_param_value(PARAM_NAME, value);
}

ParameterManager::ParameterManager() {
    this->param_init_all();
}

char *ParameterManager::get_param_name(params_enum PARAM_NAME, char *destination) const {
    return strcpy(destination, params.p_name[PARAM_NAME]);
}
