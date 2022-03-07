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
#include "swm_road_side_alert.h"
#include "swm_monitor_window_proc.h"

const char* get_road_side_alert(u16 itis)
{
    switch (itis)
    {
    case ITIS_CODE_ROAD_CONSTRUCTION:
		return "road-construction";
		break;
	case ITIS_CODE_VISIBILITY_BLOCKED:
		return "visibility-blocked";
		break;
    case ITIS_CODE_OBSTRUCTION_ON_ROADWAY:
        return "obstruction on roadway";
        break;
    case ITIS_CODE_VEHICLE_TRAVELING_WRONG_WAY:
        return "vehicle traveling wrong way";
        break;
    case ITIS_CODE_PEOPLE_ON_ROADWAY:
        return "people on roadway";
        break;
    case ITIS_CODE_DISABLE_VEHICLE:
        return "disable vehicle";
        break;
    case ITIS_CODE_U_TURN:
        return "u-turn";
        break;
    case ITIS_CODE_LONG_QUEUES:
        return "long queues";
        break;
    case ITIS_CODE_EMERGENCY_VEHICLES_ON_ROADWAY:
        return "emergency vehicles on roadway";
        break;
    case ITIS_CODE_VEHICLE_ON_FIRE:
        return "vehicle on fire";
        break;
    case ITIS_CODE_PEDESTRAINS:
        return "pedestrains";
        break;
    case ITIS_CODE_BICYCLISTS:
        return "bicyclists";
        break;
    case ITIS_CODE_PARKING:
        return "parking";
        break;
    case ITIS_CODE_DRIVE_CAREFULLY:
        return "drive carefully";
        break;
    case ITIS_CODE_CROWDED:
        return "crowded";
        break;
    case ITIS_CODE_OVERCROWDED:
        return "over crowded";
        break;
    case ITIS_CODE_APPROACH_WITH_CARE:
        return "approach with care";
        break;
	case ITIS_CODE_ALLOW_EMERGENCY_VEHICLE_TO_PASS:
		return "allow emergency vehicles to pass";
		break;
	case ITIS_CODE_CLEAR_A_LANE_FOR_EMERGENCY_VEHICLES:
		return "clear a lane for emergency vehicles";
		break;
    default:
        break;
    }
    return "unknown";
}

const char* get_extent_string(u16 extent)
{
    switch (extent)
    {
    case 0 : // useInstantlyOnly     (0),
        return "use instantly only";
        break;
    case 1 : // useFor3meters        (1),
        return "3 meters";
        break;
    case 2 : // useFor10meters       (2),
        return "10 meters";
        break;
    case 3 : // useFor50meters       (3),
        return "50 meters";
        break;
    case 4 : // useFor100meters      (4),
        return "100 meters";
        break;
    case 5 : // useFor500meters      (5),
        return "500 meters";
        break;
    case 6 : // useFor1000meters     (6),
        return "1 kilometers";
        break;
    case 7 : // useFor5000meters     (7),
        return "5 kilometers";
        break;
    case 8 : // useFor10000meters    (8),
        return "10 kilometers";
        break;
    case 9 : // useFor50000meters    (9),
        return "50 kilometers";
        break;
    case 10 : // useFor100000meters   (10),
        return "100 kilometers";
        break;
    case 11 : // useFor500000meters   (11),
        return "500 kilometers";
        break;
    case 12 : // useFor1000000meters  (12),
        return "1000 kilometers";
        break;
    case 13 : // useFor5000000meters  (13),
        return "5000 kilometers";
        break;
    case 14 : // useFor10000000meters (14), 
        return "10,000 kilometers";
        break;
    case 15 : // forever              (15)  -- very wide area
        return "very wide area";
        break;
    default:
        break;
    }
    return "unknown";
}

void output_road_side_alert_string(u16 itis)
{
    switch (itis)
    {
    case ITIS_CODE_OBSTRUCTION_ON_ROADWAY:
        cprintf(COLOR_RED, "obstruction on roadway");
        break;
    case ITIS_CODE_VEHICLE_TRAVELING_WRONG_WAY:
        cprintf(COLOR_RED, "vehicle traveling wrong way");
        break;
    case ITIS_CODE_PEOPLE_ON_ROADWAY:
        cprintf(COLOR_RED, "people on roadway");
        break;
    case ITIS_CODE_DISABLE_VEHICLE:
        cprintf(COLOR_RED, "disable vehicle");
        break;
    case ITIS_CODE_U_TURN:
        cprintf(COLOR_RED, "u-turn");
        break;
    case ITIS_CODE_LONG_QUEUES:
        cprintf(COLOR_RED, "long queues");
        break;
    case ITIS_CODE_EMERGENCY_VEHICLES_ON_ROADWAY:
        cprintf(COLOR_RED, "emergency vehicles on roadway");
        break;
    case ITIS_CODE_VEHICLE_ON_FIRE:
        cprintf(COLOR_RED, "vehicle on fire");
        break;
    case ITIS_CODE_PEDESTRAINS:
        cprintf(COLOR_RED, "pedestrains");
        break;
    case ITIS_CODE_BICYCLISTS:
        cprintf(COLOR_YELLOW, "bicyclists");
        break;
    case ITIS_CODE_PARKING:
        cprintf(COLOR_YELLOW, "parking");
        break;
    case ITIS_CODE_DRIVE_CAREFULLY:
        cprintf(COLOR_RED, "drive carefully");
        break;
    case ITIS_CODE_CROWDED:
        cprintf(COLOR_YELLOW, "crowded");
        break;
    case ITIS_CODE_OVERCROWDED:
        cprintf(COLOR_YELLOW, "over crowded");
        break;
    case ITIS_CODE_APPROACH_WITH_CARE:
        cprintf(COLOR_YELLOW, "approach with care");
        break;
    default:
        break;
    }
}
