/*
 * swm_road_side_alert.h
 *
 *  Created on: May 24, 2021
 *      Author: yhcho
 */

#ifndef SWM_ROAD_SIDE_ALERT_H_
#define SWM_ROAD_SIDE_ALERT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	ITIS_CODE_ROAD_CONSTRUCTION				1025	// road-construction
#define	ITIS_CODE_EMERGENCY_VEHICLES_ON_ROADWAY	1796	// emergency-vehicles-on-roadway
#define	ITIS_CODE_VISIBILITY_BLOCKED			5404	// visibility-blocked
#define	ITIS_CODE_ALLOW_EMERGENCY_VEHICLE_TO_PASS		7438	// allow-emergency-vehicles-to-pass
#define	ITIS_CODE_CLEAR_A_LANE_FOR_EMERGENCY_VEHICLES	7439	// clear-a-lane-for-emergency-vehicles

#define ITIS_CODE_OBSTRUCTION_ON_ROADWAY        1281
#define ITIS_CODE_VEHICLE_TRAVELING_WRONG_WAY   1793
#define ITIS_CODE_PEOPLE_ON_ROADWAY             1286
#define ITIS_CODE_DISABLE_VEHICLE               534
#define ITIS_CODE_U_TURN                        7751
#define ITIS_CODE_LONG_QUEUES                   262
#define ITIS_CODE_VEHICLE_ON_FIRE               540
#define ITIS_CODE_PEDESTRAINS                   9486
#define ITIS_CODE_BICYCLISTS                    9487
#define ITIS_CODE_PARKING                       4120
#define ITIS_CODE_DRIVE_CAREFULLY               7169
#define ITIS_CODE_CROWDED                       1545
#define ITIS_CODE_OVERCROWDED                   1546
#define ITIS_CODE_APPROACH_WITH_CARE            7171

extern const char* get_road_side_alert(u16 itis);
extern const char* get_extent_string(u16 extent);
extern void output_road_side_alert_string(u16 itis);

#ifdef __cplusplus
}
#endif

#endif /* SWM_ROAD_SIDE_ALERT_H_ */
