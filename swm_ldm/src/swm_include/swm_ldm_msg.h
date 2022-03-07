/*
 * swm_ldm_msg.h
 *
 *  Created on: May 17, 2021
 *      Author: yhcho
 */

#ifndef SWM_LDM_MSG_H_
#define SWM_LDM_MSG_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <mqueue.h>
#include "swm_define.h"

#ifdef __cplusplus
extern "C" {
#endif

// MAP message
#define PROFILE_TYPE_POSITION                   0
#define PROFILE_TYPE_PATH_CONTROL               1
#define PROFILE_TYPE_NODE                       2
#define PROFILE_TYPE_LINK                       3
#define PROFILE_TYPE_DRIVE_WAY_SECTION          4
#define PROFILE_TYPE_SAFETY_SIGN                5
#define PROFILE_TYPE_SURFACE_LINE_MARK          6
#define PROFILE_TYPE_SURFACE_MARK               7
#define PROFILE_TYPE_TRAFFIC_LIGHT              8
#define PROFILE_TYPE_VEHICLE_PROTECTION_SAFETY  9
#define PROFILE_TYPE_SPEED_BUMP                 10
#define PROFILE_TYPE_POST_POINT                 11
#define PROFILE_TYPE_SD_NODE                    12
#define PROFILE_TYPE_SD_LINK                    13

// V2X receiving message
#define PROFILE_TYPE_TRAFFIC_SIGNAL             14
#define PROFILE_TYPE_ROAD_SIDE_ALERT            15
#define PROFILE_TYPE_EMERGENCY_VEHICLE_INFO     16
#define PROFILE_TYPE_ROADWAY_CROSSING_PEDESTRIAN_INFO   17
#define PROFILE_TYPE_PEDESTRIAN_RISK_INFO       18
#define PROFILE_TYPE_LANE_QUEUE_INFORMATION     19
//#define PROFILE_TYPE_PROBE_VEHICLE_DATA         20      // To be deleted
#define PROFILE_TYPE_BASIC_SAFETY               21
#define PROFILE_TYPE_CFM                        22
#define PROFILE_TYPE_CSM                        23
#define PROFILE_TYPE_CLCM                       24
#define PROFILE_TYPE_EMERGENCY_VEHICLE_ALERT    25

// V2X sending message
#define PROFILE_TYPE_CAV_DAT                    30
#define PROFILE_TYPE_EVA                        31 

#define PROFILE_HEADER_LENGTH                   32
#define PROFILE_LINK_LENGTH                     132


// 32 bytes
typedef struct _LDM_MSG_HEADER {
    long long qTimeStamp;   // Global Time Stamp
    u32 unMsgSize;          // message size //  Message Header + Payload(Message)
    u32 unMsgType;          // message type enum
    u32 unPathId;           // path id of profile
    u8 unLinkId[12];        // link id nearested by profile
    //u8* data;             // message body
} st_LDMMsgHeader;

#ifdef __cplusplus
}
#endif

#endif /* SWM_LDM_MSG_H_ */
