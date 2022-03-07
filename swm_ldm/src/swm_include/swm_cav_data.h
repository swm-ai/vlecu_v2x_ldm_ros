/*
 * swm_cav_data.h
 *
 *  Created on: May 18, 2021
 *      Author: yhcho
 */

#ifndef SWM_CAV_DATA_H_
#define SWM_CAV_DATA_H_

#include <string>
#include <utility>
#include <tuple>
#include <map>
#include "swm_define.h"
#include "swm_ldm/st_SWMLDM.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAV_DATA_MAX 		(32)

typedef struct _CV_RES{
	unsigned char	vid;
	unsigned char	response;
	unsigned short	count;
}st_CVRES;

typedef struct _CONVOY{
	unsigned char	vid;
	unsigned short	status;
	unsigned char	join_req;
	unsigned char	leave_notice;
	st_CVRES		accept_res;
	unsigned char	chg_req;
	unsigned char	pre_lane;
	unsigned char	chg_lane;
	unsigned char	headway;
	st_CVRES		chg_res;
	unsigned char	chg_notice;
}st_CONVOY;

typedef struct _CAV_DATA{
	unsigned short 	count;
	unsigned int 	sec;
	unsigned short 	msec;
// size 8 / 8

	char 			id[4];
// size 4 / 12

	int 			latitude;		// *
	int 			longitude;		// *
// size 8 / 20

	unsigned short	elevation;		// *
	unsigned char	transmission;		// *
	unsigned short 	speed;
	unsigned short 	heading;		// *
// size 7 / 27

	short			angle;
	short 			acc_long;
	short 			acc_lat;		// *
	char		 	acc_vert;
	short 			acc_yaw;		// *
// size 8 / 35

	unsigned char	wheelbrakes;
	unsigned char	traction;
	unsigned char	abs;
	unsigned char	scs;
	unsigned char	brakeboost;
	unsigned char	auxbrakes;
// size 6 / 41
	st_CONVOY		tConvoy;
// = 41 * 14699
}st_CAVData;

typedef struct _CAVData_Lists{
	st_CAVData data;
	struct _CAVData_Lists *next;
}st_CAVDataLists;

#define CAVMSG_SET			(1)
#define CAVMSG_SEND			(0)

typedef struct _TR_SIGNAL {
	unsigned char	traffic_light_id[16];
	unsigned char	event_state;
	unsigned int	end_time;
	int				traffic_light_type;
	unsigned char	set_flag;	// use one byte from fromlink direction of trafficsignal
} st_TR_SIGNAL;

using namespace std;

/** traffic_light_id, link_direction */
typedef	std::pair<std::string, int>							traffic_and_direction_info_t;
/** link_type, lane_no, r_link_id, l_link_id, max_speed */
typedef	std::tuple<int, int, std::string, std::string, int>	a2_link_info_t;
/** std::pair<traffic_light_id, traffic_type> */
typedef std::pair<std::string, int>							traffic_light_info_t;
/** std::pair<direction_type, max_speed> */
typedef std::pair<int, int>									intersection_link_info_t;
/** std::pair<link_type, max_speed> */
typedef std::pair<int, int>									road_link_info_t;


/** link_id(key), std::pair<link_type, max_speed> */
typedef std::map<std::string, road_link_info_t>				road_link_list_t;

/** ref_link_id(key), std::pair<traffic_light_id, traffic_type> */
typedef std::map<std::string, traffic_light_info_t>			traffic_light_list_t;
/** link_id(key), std::pair<direction_type, max_speed> */
typedef std::map<std::string, intersection_link_info_t>		intersection_link_list_t;
/** link_id(key), tuple<link_type, lane_no, r_link_id, l_link_id, max_speed> */
typedef std::map<std::string, a2_link_info_t>				a2_link_list_t;

/** map datas on each path */
typedef	struct	_MAP_DATA {
	/** intesrection links */
	intersection_link_list_t	intersection_link_list;
	/** a2 links on road */
	a2_link_list_t				road_link_list;
	/** traffic lights */
	traffic_light_list_t		traffic_light_list;
} st_MAP_DATA;

/**
 * whole map data divided by path id
 * path_id(key), st_MAP_DATA
 */
typedef	std::map<u32, st_MAP_DATA*>	hdmap_table_t;


/** traffic_id(key), st_TR_SIGNAL */
typedef std::map<std::string, st_TR_SIGNAL*> 				traffic_signal_list_t;


// fromlink direction of trafficsignal
#define	TYPE_LINK_STR		1001	// STRIGHT
#define	TYPE_LINK_LEFT		1002	// LEFT
#define	TYPE_LINK_RIGHT		1004	// RIGHT
#define	TYPE_LINK_UTURN		1008	// U-TURN

// traffic_light_type
#define	TYPE_TRAFFIC_LIGHT_STR_ONLY		101	// code 1
#define	TYPE_TRAFFIC_LIGHT_STR_LEFT		102	// code 2
#define	TYPE_TRAFFIC_LIGHT_LEFT_ONLY	201	// code 1 (real direction is left trun)
#define	TYPE_TRAFFIC_LIGHT_BUS			302	// code 1 (bus only)

// traffic light signal
#define	TRAFFIC_LIGHT_COLOR_UNKNOWN		0
#define	TRAFFIC_LIGHT_COLOR_RED			1
#define	TRAFFIC_LIGHT_COLOR_YELLOW		2
#define	TRAFFIC_LIGHT_COLOR_GREEN		3
#define	TRAFFIC_LIGHT_COLOR_BLACK		4
#define	TRAFFIC_LIGHT_COLOR_RED_LEFT	5
#define	TRAFFIC_LIGHT_COLOR_GREEN_LEFT	6

// MovementPhaseState
#define	MOVEMENT_PHASE_STATE_UNAVAILABLE					0	// unavailable
#define	MOVEMENT_PHASE_STATE_DARK							1	// dark
#define	MOVEMENT_PHASE_STATE_STOP_THEN_PROCEED				2	// stop-Then-Proceed
#define	MOVEMENT_PHASE_STATE_STOP_AND_REMAIN				3	// stop-And-Remain
#define	MOVEMENT_PAHSE_STATE_PRE_MOVEMENT					4	// pre-Movement
#define	MOVEMENT_PHASE_STATE_PERMISSIVE_MOVEMENT_ALLOWED	5	// permissive-Movement-Allowed
#define	MOVEMENT_PHASE_STATE_PROTECTED_MOVEMENT_ALLOWED		6	// protected-Movement-Allowed
#define	MOVEMENT_PHASE_STATE_PERMISSIVE_CLEARANCE			7	// permissive-clearance
#define	MOVEMENT_PHASE_STATE_PROTECTED_CLEARANCE			8	// protected-clearance
#define	MOVEMENT_PAHSE_STATE_CAUTION_CONFLICTING_TRAFFIC	9	// caution-Conflicting-Traffic

// ITIS code
#define	ITIS_CODE_ROAD_CONSTRUCTION				1025	// road-construction
#define	ITIS_CODE_EMERGENCY_VEHICLES_ON_ROADWAY	1796	// emergency-vehicles-on-roadway
#define	ITIS_CODE_VISIBILITY_BLOCKED			5404	// visibility-blocked
#define	ITIS_CODE_ALLOW_EMERGENCY_VEHICLE_TO_PASS		7438	// allow-emergency-vehicles-to-pass
#define	ITIS_CODE_CLEAR_A_LANE_FOR_EMERGENCY_VEHICLES	7439	// clear-a-lane-for-emergency-vehicles
// this definitions are based on seoul c-its specification
#define ITIS_CODE_LONG_QUEUES                   262
#define ITIS_CODE_DISABLE_VEHICLE               534
#define ITIS_CODE_VEHICLE_ON_FIRE               540
#define ITIS_CODE_OBSTRUCTION_ON_ROADWAY        1281
#define ITIS_CODE_PEOPLE_ON_ROADWAY             1286
#define ITIS_CODE_CROWDED                       1545
#define ITIS_CODE_OVERCROWDED                   1546
#define ITIS_CODE_VEHICLE_TRAVELING_WRONG_WAY   1793
#define ITIS_CODE_PARKING                       4120
#define ITIS_CODE_DRIVE_CAREFULLY               7169
#define ITIS_CODE_APPROACH_WITH_CARE            7171
#define ITIS_CODE_U_TURN                        7751
#define ITIS_CODE_PEDESTRAINS                   9486
#define ITIS_CODE_BICYCLISTS                    9487


extern int add_traffic_signal(st_TR_SIGNAL* trsignal);
extern int remove_traffic_signal(const char* traffic_light_id);
extern st_TR_SIGNAL* get_traffic_signal(const char* traffic_light_id);
extern traffic_signal_list_t& get_sending_traffic_signal_list(void);
extern void clear_sending_traffic_signal_list(void);

extern int add_hdmap_info(unsigned int path_id);
extern int get_hdmap_info_number(void);
extern st_MAP_DATA* get_hdmap_info(unsigned int path_id);
extern void remove_hdmap_info(unsigned int path_id);
extern void resize_hdmap_info(std::map<int, int>& path_list);
extern void clear_hdmap_info(void);

extern int get_a2_linlk_usable(void);
extern int set_current_position(const char* a2_link);
extern int set_current_link_list(void);
// direction_val : 0 = process all link_id, 1 = process left link_id, 2 = process right_link_id;
extern int set_near_link_list(road_link_list_t& link_set, std::string a2_link_id, int direction_val);
extern bool is_near_link(const char* from_a2_link);

#if defined(USE_LOCAL_MAP_DATA)
extern traffic_and_direction_info_t* get_traffic_and_direction_info(const char* link_id);
extern int get_tr_light_info(std::string traffic_light_id);
#endif

extern int add_traffic_light_info(const char* ref_link_id, const char* traffic_light_id, int light_type);
extern traffic_light_info_t* get_traffic_light_info(const char* ref_link_id);
extern void clear_traffic_light_info(void);

extern int add_intersection_link_info(const char* link_id, int direction_type, int max_speed);
extern intersection_link_info_t* get_intersection_link_info(const char* link_id);
extern void clear_intersection_link_info(void);

extern int add_a2_link_info(const char* link_id, int link_type, int lane_no, const char* r_link_id, const char* l_link_id, int max_speed);
extern a2_link_info_t* get_a2_link_info(const char* link_id);
extern void clear_a2_link_info(void);

extern int get_traffic_light_id(const char* link_id, char* traffic_light_id, int& traffic_light_type);

extern void test_current_link_list(void);
extern void test_traffic_light_info(void);
extern void test_intersection_link_info(void);

extern int get_traffic_light_color(unsigned char ev_state);
extern int get_traffic_light_direction_color(unsigned char ev_state, int direction);
extern int get_merged_traffic_light_color(int l1, unsigned int& t1, int l2, unsigned int& t2);
extern const char* get_movement_phase_state(u8 state);
extern const char* get_itis_event(u16 event);
extern const char* get_traffic_ligt_color_str(unsigned char event);

#if defined(USE_LOCAL_MAP_DATA)
extern int proc_traffic_light_information(u8 *node_id, u8 ev_state, u32 remain_time);
#else
extern int proc_traffic_light_information(u8 *from_node_id, u8 *to_node_id, u8 ev_state, u32 remain_time);
#endif

extern void swm_ldm_msg_log(swm_ldm::st_SWMLDM& msg);

#ifdef __cplusplus
}
#endif

#endif /* SWM_CAV_DATA_H_ */
