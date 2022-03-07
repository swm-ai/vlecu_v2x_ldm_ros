#include "ros/ros.h"
#include "swm_ldm/st_SWMLDM.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>
#include <ncurses.h>

#include "swm_define.h"
#include "swm_emergency_vehicle_alert.h"
#include "swm_monitor_window_proc.h"

const char* get_response_type(u8 response_type)
{
    switch (response_type)
    {
    case RESPONSE_TYPE_NOT_IN_USE_OR_NOT_EQUIPPED : // - notInUseOrNotEquipped (0)
        return "not in use or not equipped";
        break;
    case RESPONSE_TYPE_EMERGENCY : // - emergency (1)
        return "emergency";
        break;
    case RESPONSE_TYPE_NON_EMERGENCY : // - nonEmergency (2)
        return "non emergency";
        break;
    case RESPONSE_TYPE_PURSUIT : // - pursuit (3)
        return "pursuit";
        break;
    case RESPONSE_TYPE_STATIONARY : // - stationary (4)
        return "stationary";
        break;
    case RESPONSE_TYPE_SLOW_MOVING : // - slowMoving (5)
        return "slow moving";
        break;
    case RESPONSE_TYPE_STOP_AND_GO_MOVEMENT : // - stopAndGoMovement (6)
        return "stop and go movement";
        break;
    default :
        break;
    }

    return "unknown";
}

const char* get_sirent_state(u8 siren_use)
{
    switch (siren_use)
    {
    case ITIS_SIREN_STATE_UNAVAILABLE : // - unavailable (0)
        return "unavailable";
        break;
    case ITIS_SIREN_STATE_NOT_IN_USE : // - notInUse (1)
        return "not in use";
        break;
    case ITIS_SIREN_STATE_IN_USE : // - inUse (2)
        return "in use";
        break;
    case ITIS_SIREN_STATE_RESERVED : // - reserved (3)
        return "reserved";
        break;
    default:
        break;
    }
    
    return "unknown";
}

const char* get_light_bar_state(u8 lights_use)
{
    switch (lights_use)
    {
    case ITIS_LIGHT_BAR_STATE_UNAVAILABLE : // - unavailable (0)
        return "unavailable";
        break;
    case ITIS_LIGHT_BAR_STATE_NOT_IN_USE : // - notInUse (1)
        return "not in use";
        break;
    case ITIS_LIGHT_BAR_STATE_IN_USE : // - inUse (2)
        return "in use";
        break;
    case ITIS_LIGHT_BAR_STATE_YELLOW_CAUTION_LIGHTS : // - yellowCautionLights (3)
        return "yellow caution lights";
        break;
    case ITIS_LIGHT_BAR_STATE_SCHOOL_BUS_LIGHTS : // - schooldBusLights (4)
        return "school bus lights";
        break;
    case ITIS_LIGHT_BAR_STATE_ARROW_SIGNS_ACTIVE : // - arrowSignsActive (5)
        return "arrow signs active";
        break;
    case ITIS_LIGHT_BAR_STATE_SLOW_MOVING_VEHICLE : // - slowMovingVehicle (6)
        return "slow moving vehicle";
        break;
    case ITIS_LIGHT_BAR_STATE_FREQ_STOPS : // - freqStops (7) 
        return "freq stops";
        break;
    default:
        break;
    }

    return "unknown";
}

const char* get_multiple_vehicle_type(u8 multi)
{
    switch (multi)
    {
    case MULTI_VEHICLE_RESPONSE_TYPE_UNAVAILABLE : // - unavailable (0)
        return "unavailable";
        break;
    case MULTI_VEHICLE_RESPONSE_TYPE_SINGLE_VEHICLE : // - singleVehicle (1)
        return "single vehicle";
        break;
    case MULTI_VEHICLE_RESPONSE_TYPE_MULTI_VEHICLE : // - multiVehicle (2)
        return "multi vehicle";
        break;
    case MULTI_VEHICLE_RESPONSE_TYPE_RESERVED : // - reserved (3)
        return "reserved";
        break;
    default :
        break;
    }

    return "unknown";
}
