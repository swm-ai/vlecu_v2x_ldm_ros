/*
 * swm_emergency_vehicle_alert.h
 *
 *  Created on: July 15, 2021
 *      Author: yhcho
 */

#ifndef SWM_EMERGENCY_VEHICLE_ALERT_H_
#define SWM_EMERGENCY_VEHICLE_ALERT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// ResponseType 
#define RESPONSE_TYPE_NOT_IN_USE_OR_NOT_EQUIPPED    0 // - notInUseOrNotEquipped (0)
#define RESPONSE_TYPE_EMERGENCY                     1 // - emergency (1)
#define RESPONSE_TYPE_NON_EMERGENCY                 2 // - nonEmergency (2)
#define RESPONSE_TYPE_PURSUIT                       3 // - pursuit (3)
#define RESPONSE_TYPE_STATIONARY                    4 // - stationary (4)
#define RESPONSE_TYPE_SLOW_MOVING                   5 // - slowMoving (5)
#define RESPONSE_TYPE_STOP_AND_GO_MOVEMENT          6 // - stopAndGoMovement (6)

// VehicleGroupAffected
#define ITIS_VEHICLE_TYPE_ALL_VEHICLES              9217 // - all-vehicles (9217)
#define ITIS_VEHICLE_TYPE_BICYCLES                  9218 // - bicycles (9218)
#define ITIS_VEHICLE_TYPE_MOTORCYCLES               9219 // - motorcycles (9219)
#define ITIS_VEHICLE_TYPE_CARS                      9220 // - cars (9220)
// - light-vehicles (9221)
// - cars-and-light-vehicles (9222)
// - cars-with-trailers (9223),
// - cars-with-recreational-trailers (9224)
// - vehicles-with-trailers (9225)
// - heavy-vehicles (9226)
// - trucks (9227)
// - buses (9228)
// - articulated-buses (9229)
#define ITIS_VEHICLE_TYPE_SCHOOL_BUSES              9230 // - school-buses (9230)
// - vehicles-with-semi-trailers (9231)
// - vehicles-with-double-trailers (9232)
// - high-profile-vehicles (9233)
// - wide-vehicles (9234)
// - long-vehicles (9235)
// - hazardous-loads (9236)
// - exceptional-loads (9237)
// - abnormal-loads (9238)
#define ITIS_VEHICLE_TYPE_CONVOYS                   9239 // - convoys (9239)
// - maintenance-vehicles (9240)
// - delivery-vehicles (9241)
// - vehicles-with-even-numbered-license-plates (9242)
// - vehicles-with-odd-numbered-license-plates (9243)
// - vehicles-with-parking-permits (9244)
// - vehicles-with-catalytic-converters (9245)
// - vehicles-without-catalytic-converters (9246)
// - gas-powered-vehicles (9247)
// - diesel-powered-vehicles (9248)
// - lPG-vehicles (9249)
#define ITIS_VEHICLE_TYPE_MILITARY_CONVOYS          9250 // - military-convoys (9250)
#define ITIS_VEHICLE_TYPE_MILITARY_VEHICLES         9251 // - military-vehicles (9251)

// SirenInUse
#define ITIS_SIREN_STATE_UNAVAILABLE     0 // - unavailable (0)
#define ITIS_SIREN_STATE_NOT_IN_USE      1 // - notInUse (1)
#define ITIS_SIREN_STATE_IN_USE          2 // - inUse (2)
#define ITIS_SIREN_STATE_RESERVED        3 // - reserved (3)

// LightbarInUse
#define ITIS_LIGHT_BAR_STATE_UNAVAILABLE                0 // - unavailable (0)
#define ITIS_LIGHT_BAR_STATE_NOT_IN_USE                 1 // - notInUse (1)
#define ITIS_LIGHT_BAR_STATE_IN_USE                     2 // - inUse (2)
#define ITIS_LIGHT_BAR_STATE_YELLOW_CAUTION_LIGHTS      3 // - yellowCautionLights (3)
#define ITIS_LIGHT_BAR_STATE_SCHOOL_BUS_LIGHTS          4 // - schooldBusLights (4)
#define ITIS_LIGHT_BAR_STATE_ARROW_SIGNS_ACTIVE         5 // - arrowSignsActive (5)
#define ITIS_LIGHT_BAR_STATE_SLOW_MOVING_VEHICLE        6 // - slowMovingVehicle (6)
#define ITIS_LIGHT_BAR_STATE_FREQ_STOPS                 7 // - freqStops (7) 

// MultiVehicleResponse
#define MULTI_VEHICLE_RESPONSE_TYPE_UNAVAILABLE         0 // - unavailable (0)
#define MULTI_VEHICLE_RESPONSE_TYPE_SINGLE_VEHICLE      1 // - singleVehicle (1)
#define MULTI_VEHICLE_RESPONSE_TYPE_MULTI_VEHICLE       2 // - multiVehicle (2)
#define MULTI_VEHICLE_RESPONSE_TYPE_RESERVED            3 // - reserved (3)

extern const char* get_response_type(u8 response_type);
extern const char* get_sirent_state(u8 siren_use);
extern const char* get_light_bar_state(u8 lights_use);
extern const char* get_multiple_vehicle_type(u8 multi);

#ifdef __cplusplus
}
#endif

#endif /* SWM_EMERGENCY_VEHICLE_ALERT_H_ */
