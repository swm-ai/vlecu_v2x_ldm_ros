/*
 * swm_cav_data.cpp
 *
 *  Created on: May 20, 2021
 *      Author: yhcho
 */

#include <map>
#include <string>

#include "swm_define.h"
#include "swm_common.h"
#include "swm_cav_data.h"
#include "swm_ldm_dec.h"


static traffic_signal_list_t sending_traffic_signal_list;


int add_traffic_signal(st_TR_SIGNAL* trsignal)
{
	std::pair<traffic_signal_list_t::iterator, bool> result = sending_traffic_signal_list.insert({string((const char*)trsignal->traffic_light_id), trsignal});
	if (result.second)
		return 0;

	//SWM_INFO(CLI_INFO_MAP, "fail to add traffic signal to list. traffic id : %s\n", trsignal->traffic_light_id);

	return -1;
}

int remove_traffic_signal(const char* traffic_light_id)
{
	/*if (sending_traffic_signal_list.erase(traffic_light_id))
		return 0;*/
	auto search = sending_traffic_signal_list.find(traffic_light_id);

	if (search != sending_traffic_signal_list.end())
	{
		if (search->second != NULL) delete search->second;
		search->second = NULL;
		sending_traffic_signal_list.erase(search);
		return 0;
	}

	return -1;
}

st_TR_SIGNAL* get_traffic_signal(const char* traffic_light_id)
{
	auto search = sending_traffic_signal_list.find(traffic_light_id);

	if (search != sending_traffic_signal_list.end())
		return search->second;

	return NULL;
}

traffic_signal_list_t& get_sending_traffic_signal_list(void)
{
	return sending_traffic_signal_list;
}

void clear_sending_traffic_signal_list(void)
{
	sending_traffic_signal_list.clear();
}

#if defined(USE_LOCAL_MAP_DATA)
static std::map<std::string, traffic_and_direction_info_t> traffic_light_map = {
	// intersection	101
	// traffic light C119AW000004
	{"A219AW000046", std::make_pair("C119AW000004", TYPE_LINK_STR)},
	{"A219AW000034", std::make_pair("C119AW000004", TYPE_LINK_STR)},	// ?
	{"A219AW000090", std::make_pair("C119AW000004", TYPE_LINK_LEFT)},	//
	// traffic light C119AW000077
	{"A219AW000140", std::make_pair("C119AW000077", TYPE_LINK_LEFT)},	//
	{"A219AW000040", std::make_pair("C119AW000077", TYPE_LINK_STR)},	// ?
	{"A219AW000062", std::make_pair("C119AW000077", TYPE_LINK_STR)},	// ?
	{"A219AW000008", std::make_pair("C119AW000077", TYPE_LINK_STR)},	//
	// traffic light C119AW000018
	// A219AW000091 // left // bus
	{"A219AW000093", std::make_pair("C119AW000018", TYPE_LINK_LEFT)},	//
	{"A219AW000092", std::make_pair("C119AW000018", TYPE_LINK_STR)},
	{"A219AW000110", std::make_pair("C119AW000018", TYPE_LINK_STR)},	//?
	// traffic light C119AW000073
	{"A219AW000079", std::make_pair("C119AW000073", TYPE_LINK_STR)},	//?
	{"A219AW000042", std::make_pair("C119AW000073", TYPE_LINK_STR)},	//
	{"A219AW000139", std::make_pair("C119AW000073", TYPE_LINK_LEFT)},	//

	// traffic lights of intersection 105
	// traffic light C119AW000021
	{"A219AW000114", std::make_pair("C119AW000021", TYPE_LINK_STR)},
	// traffic light C119AW000022
	{"A219AW000138", std::make_pair("C119AW000022", TYPE_LINK_LEFT)},	//
	// traffic light C119AW000024
	// A219AW000178 // stright // bus
	{"A219AW000137", std::make_pair("C119AW000024", TYPE_LINK_LEFT)},	//
	{"A219AW000049", std::make_pair("C119AW000024", TYPE_LINK_STR)},	//
	{"A219AW000117", std::make_pair("C119AW000024", TYPE_LINK_STR)},	//?
	// traffic light C119AW000023
	{"A219AW000115", std::make_pair("C119AW000023", TYPE_LINK_STR)},	//
	{"A219AW000116", std::make_pair("C119AW000023", TYPE_LINK_LEFT)},	//

	// traffic light of intersection 106
	// traffic light C119AW000001
	{"A219AW000101", std::make_pair("C119AW000001", TYPE_LINK_STR)},	//
	{"A219AW000166", std::make_pair("C119AW000001", TYPE_LINK_LEFT)},	//?
	// traffic light C119AW000002
	{"A219AW000095", std::make_pair("C119AW000002", TYPE_LINK_LEFT)},	//
	{"A219AW000099", std::make_pair("C119AW000002", TYPE_LINK_STR)},	//
	// traffic light C119AW000003
	{"A219AW000100", std::make_pair("C119AW000003", TYPE_LINK_STR)},	//
};
#endif

#if defined(USE_LOCAL_MAP_DATA)
traffic_and_direction_info_t* get_traffic_and_direction_info(const char* link_id)
{
	auto search = traffic_light_map.find(link_id);

	if (search != traffic_light_map.end())
		return &search->second;

	return NULL;
}
#endif

#if defined(USE_LOCAL_MAP_DATA)
static std::map<std::string, int>	traffic_light_info = {
	// traffic lights of intersection 101
	{"C119AW000004", TYPE_TRAFFIC_LIGHT_STR_LEFT},
	{"C119AW000077", TYPE_TRAFFIC_LIGHT_STR_LEFT},
	{"C119AW000018", TYPE_TRAFFIC_LIGHT_STR_LEFT},
	{"C119AW000073", TYPE_TRAFFIC_LIGHT_STR_LEFT},
	// traffic lights of intersection 105
	{"C119AW000021", TYPE_TRAFFIC_LIGHT_STR_ONLY},
	{"C119AW000022", TYPE_TRAFFIC_LIGHT_LEFT_ONLY}, //?
	{"C119AW000024", TYPE_TRAFFIC_LIGHT_STR_LEFT},
	{"C119AW000023", TYPE_TRAFFIC_LIGHT_STR_LEFT},
	// traffic light of intersection 106
	{"C119AW000001", TYPE_TRAFFIC_LIGHT_STR_LEFT}, //?
	{"C119AW000002", TYPE_TRAFFIC_LIGHT_STR_LEFT}, 
	{"C119AW000003", TYPE_TRAFFIC_LIGHT_STR_ONLY},
};

int get_tr_light_info(std::string traffic_light_id)
{
	auto search = traffic_light_info.find(traffic_light_id);

	if (search != traffic_light_info.end())
		return search->second;

	return -1;
}
#endif

/** current position is set. */
static bool is_set = false;

/** current position(a2 link represented) */
static std::string current_a2_link;

// a2 link list of current position
/** link_id(key), std::pair<link_type, max_speed> */
static road_link_list_t current_link_list;

/** ref_link_id(key), std::pair<traffic_light_id, traffic_type> */
static traffic_light_list_t	traffic_light_list;

// this a2 links are only intersections inner
/** link_id(key), std::pair<direction_type, max_speed> */
static intersection_link_list_t	intersection_link_list;

/** link_id(key), tuple<link_type, lane_no, r_link_id, l_link_id, max_speed> */
static a2_link_list_t a2_link_list;

/**
 * whole map data divided by path id
 */
static hdmap_table_t	ldm_hdmap_data;


int add_hdmap_info(unsigned int path_id)
{
	if (ldm_hdmap_data.find(path_id) != ldm_hdmap_data.end()) return 1;

	std::pair<hdmap_table_t::iterator, bool> result = ldm_hdmap_data.insert({path_id, new st_MAP_DATA()});
	if (result.second) return 0;

	return -1;	
}

int get_hdmap_info_number(void)
{
	return ldm_hdmap_data.size();
}

st_MAP_DATA* get_hdmap_info(unsigned int path_id)
{
	st_MAP_DATA* data = NULL;

	auto search = ldm_hdmap_data.find(path_id);
	if (search != ldm_hdmap_data.end()) data = search->second;

	return data;
}

void remove_hdmap_info(unsigned int path_id)
{
	auto search = ldm_hdmap_data.find(path_id);
	if (search != ldm_hdmap_data.end())
	{
		search->second->intersection_link_list.clear();
		search->second->road_link_list.clear();
		search->second->traffic_light_list.clear();
		delete search->second;
		search->second = NULL;
	}

	ldm_hdmap_data.erase(path_id);
}

void resize_hdmap_info(std::map<int, int>& path_list)
{
	for (auto element : ldm_hdmap_data)
	{
		auto search = path_list.find(element.first);
		if (search == path_list.end())
		{
			element.second->intersection_link_list.clear();
			element.second->road_link_list.clear();
			element.second->traffic_light_list.clear();
			delete element.second;
			element.second = NULL;
			ldm_hdmap_data.erase(element.first);
		}
	}
}

void clear_hdmap_info(void)
{
	for (auto element : ldm_hdmap_data)
	{
		element.second->intersection_link_list.clear();
		element.second->road_link_list.clear();
		element.second->traffic_light_list.clear();
		delete element.second;
		element.second = NULL;
	}

	ldm_hdmap_data.clear();
}

int get_a2_linlk_usable(void)
{
	return is_set;
}

int set_current_position(const char* a2_link)
{
	int b_compare = -1;
	u8 link_id [16] = {0,};

	if (get_use_current_position() == 0)
	{
		return -1;
	}

	memcpy(link_id, a2_link, sizeof(u8) * 12);

	if (strlen((const char*)link_id) > 0 && (b_compare = current_a2_link.compare((const char*)link_id)) != 0)
	{
		current_a2_link = std::string((const char*)link_id);
		is_set = true;
		set_current_link_list();
	}
	else
	{
		// a2_link not set
		if (b_compare != 0)
		{
			if (is_set)
			{
				current_a2_link.clear();
				is_set = false;
				set_current_link_list();
			}
		}
		// a2_link is not changed.
		// else {} // do nothting
	}
	
	return is_set == true ? 0 : -1;
}

int set_current_link_list(void)
{
	int ret = -1;

	// when current a2 link is set
	if (current_a2_link.length() > 0)
	{
		// clean current_link_list to update datas
		current_link_list.clear();

		ret = set_near_link_list(current_link_list, current_a2_link, 0);
	}
	else
	{
		current_link_list.clear();
	}

	return ret;
}

// direction_val : 0 = process all link_id, 1 = process left link_id, 2 = process right_link_id;
int set_near_link_list(road_link_list_t& link_set, std::string a2_link_id, int direction_val)
{
	int ret = -1;

	// if a2_link_id is empty string
	if (a2_link_id.length() <= 0)
	{
		ret = 0;
	}
	else
	{
		// find a2_link
		auto search = a2_link_list.find(a2_link_id);
		std::string p_link_id ;

		if (search != a2_link_list.end())
		{
			auto& traffic_info = search->second;

#if defined(USE_BUS_TRAFFIC_LIGHT)
			if (link_set.size() > 0)
			{
				// process bus lane
				if ((std::get<0>(link_set.begin()->second) == 4) && (std::get<0>(traffic_info) != 4))
				{
					return 0;
				}
				// process non bus lane
				else
				{
					if (std::get<0>(traffic_info) == 4)
					{
						return 0;
					}
				}
			}
#endif	// USE_BUS_TRAFFIC_LIGHT

			// link_id, link_type, max_speed
			link_set.insert({a2_link_id, std::make_pair(std::get<0>(traffic_info), std::get<4>(traffic_info))});

			switch (direction_val)
			{
			case 0:	// all
			case 1:	// left
			{
				// get l_link_id and process it
				p_link_id = std::get<3>(traffic_info);
				ret = set_near_link_list(link_set, p_link_id, 1);
				if (direction_val != 0) break;
			}
			case 2:	// right
			{
				// get r_link_id and process it
				p_link_id = std::get<2>(traffic_info);
				ret = set_near_link_list(link_set, p_link_id, 2);
				break;
			}
			default:
				break;
			}
		}
	}

	return ret;
}

bool is_near_link(const char* from_a2_link)
{
	auto search = current_link_list.find(from_a2_link);
	
	if (search != current_link_list.end()) return true;

	return false;
}

int add_traffic_light_info(const char* ref_link_id, const char* traffic_light_id, int light_type)
{
	int ret = -1;
	int traffic_light_type = 0;

	switch (light_type)
	{
	case 1:	case 5:	case 14:
	{
		traffic_light_type = TYPE_TRAFFIC_LIGHT_STR_ONLY;
		ret = 0;
		break;
	}
	case 2:	case 3:	case 7:	case 15:
	{
		traffic_light_type = TYPE_TRAFFIC_LIGHT_STR_LEFT;
		ret = 0;
		break;
	}
	case 4:	case 6:
	{
		traffic_light_type = TYPE_TRAFFIC_LIGHT_LEFT_ONLY;
		ret = 0;
		break;
	}
#if defined(USE_BUS_TRAFFIC_LIGHT)
	case 8:
	{
		traffic_light_type = TYPE_TRAFFIC_LIGHT_BUS;
		ret = 0;
		break;
	}
#endif // USE_BUS_TRAFFIC_LIGHT
	default:
		break;
	}

	if (ret == 0)
	{
		traffic_light_list.insert({ref_link_id, std::make_pair(traffic_light_id, traffic_light_type)});
	}

	return ret;
}

traffic_light_info_t* get_traffic_light_info(const char* ref_link_id)
{
	auto search = traffic_light_list.find(ref_link_id);

	if (search != traffic_light_list.end())
		return &search->second;

	return NULL;
}

void clear_traffic_light_info(void)
{
	traffic_light_list.clear();
}

int add_intersection_link_info(const char* link_id, int direction_type, int max_speed)
{
	int ret = -1;

	intersection_link_list.insert({link_id, std::make_pair(direction_type, max_speed)});
	ret = 0;

	return ret;
}

intersection_link_info_t* get_intersection_link_info(const char* link_id)
{
	auto search = intersection_link_list.find(link_id);

	if (search != intersection_link_list.end())
		return &search->second;

	return NULL;
}

void clear_intersection_link_info(void)
{
	intersection_link_list.clear();
}

int add_a2_link_info(const char* link_id, int link_type, int lane_no, const char* r_link_id, const char* l_link_id, int max_speed)
{
	int ret = -1;

	a2_link_list.insert({link_id, std::make_tuple(link_type, lane_no, r_link_id, l_link_id, max_speed)});
	ret = 0;

	return ret;
}

a2_link_info_t* get_a2_link_info(const char* link_id)
{
	auto search = a2_link_list.find(link_id);

	if (search != a2_link_list.end())
		return &search->second;

	return NULL;
}

void clear_a2_link_info(void)
{
	a2_link_list.clear();
}

int get_traffic_light_id(const char* link_id, char* traffic_light_id, int& traffic_light_type)
{
	int ret = -1;
	road_link_list_t link_group_list;

	ret = set_near_link_list(link_group_list, link_id, 0);

	if (ret == 0)
	{
		ret = -1;
		for (auto element : link_group_list)
		{
			auto search = traffic_light_list.find(element.first);

			if (search != traffic_light_list.end())
			{
				strncpy(traffic_light_id, std::get<0>(search->second).c_str(), std::get<0>(search->second).length());
				traffic_light_type = std::get<1>(search->second);
				ret = 0;
				break;
			}
		}
	}

	return ret;
}

void test_current_link_list(void)
{
	for (auto& element : current_link_list)
	{
		SWM_INFO(CLI_INFO_POS, "########## link id : %s ########## link type : %d ########## max speed : %d\n",
		// link_id(key)
		element.first.c_str(),
		// link_type
		std::get<0>(element.second),
		// max_speed
		std::get<1>(element.second));
	}
}

void test_traffic_light_info(void)
{
	for (auto& element : traffic_light_list)
	{
		SWM_INFO(CLI_INFO_MAP, "########## ref link id : %s ########## traffic_light_id : %s ########## traffic_type : %s\n",
		// ref_link_id(key)
		element.first.c_str(),
		// traffic_light_id
		std::get<0>(element.second).c_str(),
		// traffic_type
		std::get<1>(element.second) == TYPE_TRAFFIC_LIGHT_STR_LEFT ? "TYPE_TRAFFIC_LIGHT_STR_LEFT" : "TYPE_TRAFFIC_LIGHT_STR_ONLY");
	}
}

void test_intersection_link_info(void)
{
	for (auto& element : intersection_link_list)
	{
		SWM_INFO(CLI_INFO_MAP, "########## intersection link id : %s ########## direction type : %s ########## max speed : %d\n",
		// link_id(key)
		element.first.c_str(),
		// direction_type
		std::get<0>(element.second) == TYPE_LINK_STR ? "TYPE_LINK_STR" : "TYPE_LINK_LEFT",
		// max_speed
		std::get<1>(element.second));
	}
}

int get_traffic_light_color(unsigned char ev_state)
{
	int color = 0;

	switch(ev_state)
	{
	//case MOVEMENT_PHASE_STATE_PERMISSIVE_MOVEMENT_ALLOWED: //permissive-Movement-Allowed	// ignore it
	case MOVEMENT_PHASE_STATE_PROTECTED_MOVEMENT_ALLOWED: //protected-Movement-Allowed
		color = TRAFFIC_LIGHT_COLOR_GREEN;
		break;
	case MOVEMENT_PHASE_STATE_PERMISSIVE_CLEARANCE: //permissive-clearance
	case MOVEMENT_PHASE_STATE_PROTECTED_CLEARANCE: //protected-clearance
		color = TRAFFIC_LIGHT_COLOR_YELLOW;
		break;
	case MOVEMENT_PHASE_STATE_PERMISSIVE_MOVEMENT_ALLOWED: //permissive-Movement-Allowed	// ignore it
	case MOVEMENT_PAHSE_STATE_CAUTION_CONFLICTING_TRAFFIC: //caution-Conflicting-Traffic
	case MOVEMENT_PHASE_STATE_UNAVAILABLE: //unavailable
	case MOVEMENT_PHASE_STATE_DARK: //dark
	case MOVEMENT_PHASE_STATE_STOP_THEN_PROCEED: //stop-Then-Proceed
	case MOVEMENT_PHASE_STATE_STOP_AND_REMAIN: //stop-And-Remain
	case MOVEMENT_PAHSE_STATE_PRE_MOVEMENT: // pre-Movement
	default:
		color = TRAFFIC_LIGHT_COLOR_RED;
		break;
	}

	return color;
}

int get_traffic_light_direction_color(unsigned char ev_state, int direction)
{
	int color = 0;

	switch (ev_state)
	{
	case TRAFFIC_LIGHT_COLOR_GREEN:
		{
			switch (direction)
			{
			case TYPE_LINK_LEFT:
				color = TRAFFIC_LIGHT_COLOR_RED_LEFT;
				break;
			default:
				color = ev_state;
				break;
			}
			break;
		}
	default:
		color = ev_state;
		break;
	}

	return color;
}

int get_merged_traffic_light_color(int l1, unsigned int& t1, int l2, unsigned int& t2)
{
	int color = 0;

	switch (l1)
	{
	case TRAFFIC_LIGHT_COLOR_GREEN:
		{
			switch (l2)
			{
			case TRAFFIC_LIGHT_COLOR_RED_LEFT:
				color = TRAFFIC_LIGHT_COLOR_GREEN_LEFT;
				t1 = t1 > t2 ? t2 : t1; // use shortest time.
				break;
			default:
				color = l1;
				break;
			}
			break;
		}
	case TRAFFIC_LIGHT_COLOR_RED_LEFT:
		{
			switch(l2)
			{
			case TRAFFIC_LIGHT_COLOR_GREEN:
				color = TRAFFIC_LIGHT_COLOR_GREEN_LEFT;
				t1 = t1 > t2 ? t2 : t1; // use shortest time.
				break;
			default:
				color = l1;
				break;
			}
			break;
		}
	case TRAFFIC_LIGHT_COLOR_GREEN_LEFT:
		{
			switch(l2)
			{
			//case TRAFFIC_LIGHT_COLOR_RED: // may not reached. but reached, could conflict.
			case TRAFFIC_LIGHT_COLOR_YELLOW: // could conflict.
			case TRAFFIC_LIGHT_COLOR_GREEN:
			case TRAFFIC_LIGHT_COLOR_RED_LEFT:
			default:
				color = l1;
				break;
			}
			break;
		}
	case TRAFFIC_LIGHT_COLOR_YELLOW:
		{
			switch(l2)
			{
			case TRAFFIC_LIGHT_COLOR_GREEN:
			case TRAFFIC_LIGHT_COLOR_RED_LEFT:
				color = l2;
				t1 = t2;
				break;
			case TRAFFIC_LIGHT_COLOR_YELLOW:
				color = l1;
				t1 = t1 > t2 ? t2 : t1; // use shortest time.
				break;
			default:
				color = l1;
				break;
			}
			break;
		}
	//case TRAFFIC_LIGHT_COLOR_UNKNOWN:	// may not reached.
	case TRAFFIC_LIGHT_COLOR_RED:
	//case TRAFFIC_LIGHT_COLOR_BLACK:	// may not reached.
	default:
		{
			color = l2;
			t1 = t2; // use longest time
			break;
		}
	}

	return color;
}

const char* get_movement_phase_state(u8 state)
{
	switch (state)
	{
	case MOVEMENT_PHASE_STATE_UNAVAILABLE:
		return "unavailable";
		break;
	case MOVEMENT_PHASE_STATE_DARK:
		return "dark";
		break;
	case MOVEMENT_PHASE_STATE_STOP_THEN_PROCEED:
		return "stop-Then-Proceed";
		break;
	case MOVEMENT_PHASE_STATE_STOP_AND_REMAIN:
		return "stop-And-Remain";
		break;
	case MOVEMENT_PAHSE_STATE_PRE_MOVEMENT:
		return "pre-Movement";
		break;
	case MOVEMENT_PHASE_STATE_PERMISSIVE_MOVEMENT_ALLOWED:
		return "permissive-Movement-Allowed";
		break;
	case MOVEMENT_PHASE_STATE_PROTECTED_MOVEMENT_ALLOWED:
		return "protected-Movement-Allowed";
		break;
	case MOVEMENT_PHASE_STATE_PERMISSIVE_CLEARANCE:
		return "permissive-clearance";
		break;
	case MOVEMENT_PHASE_STATE_PROTECTED_CLEARANCE:
		return "protected-clearance";
		break;
	case MOVEMENT_PAHSE_STATE_CAUTION_CONFLICTING_TRAFFIC:
		return "caution-Conflicting-Traffic";
		break;
	default:
		break;
	}
	return "unknown";
}

const char* get_itis_event(u16 event)
{
	switch (event)
	{
	case ITIS_CODE_ROAD_CONSTRUCTION:
		return "road-construction";
		break;
	case ITIS_CODE_VISIBILITY_BLOCKED:
		return "visibility-blocked";
		break;
	case ITIS_CODE_LONG_QUEUES:
		return "long queues";
		break;
	case ITIS_CODE_DISABLE_VEHICLE:
		return "disable vehicle";
		break;
	case ITIS_CODE_VEHICLE_ON_FIRE:
		return "vehicle on fire";
		break;
	case ITIS_CODE_OBSTRUCTION_ON_ROADWAY:
		return "obstruction on roadway";
		break;
	case ITIS_CODE_PEOPLE_ON_ROADWAY:
		return "people on roadway";
		break;
	case ITIS_CODE_CROWDED:
		return "crowded";
		break;
	case ITIS_CODE_OVERCROWDED:
		return "overcrowded";
		break;
	case ITIS_CODE_VEHICLE_TRAVELING_WRONG_WAY:
		return "vehicle traveling wrong way";
		break;
	case ITIS_CODE_EMERGENCY_VEHICLES_ON_ROADWAY:
		return "emergency vehicles on roadway";
		break;
	case ITIS_CODE_PARKING:
		return "parking";
		break;
	case ITIS_CODE_DRIVE_CAREFULLY:
		return "drive carefully";
		break;
	case ITIS_CODE_APPROACH_WITH_CARE:
		return "approach with care";
		break;
	case ITIS_CODE_U_TURN:
		return "u-turn";
		break;
	case ITIS_CODE_PEDESTRAINS:
		return "pedestrains";
		break;
	case ITIS_CODE_BICYCLISTS:
		return "bicyclists";
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

const char* get_traffic_ligt_color_str(unsigned char event)
{
	switch (event)
	{
	case TRAFFIC_LIGHT_COLOR_YELLOW: return "TRAFFIC_LIGHT_COLOR_YELLOW"; break;
	case TRAFFIC_LIGHT_COLOR_RED: return "TRAFFIC_LIGHT_COLOR_RED"; break;
	case TRAFFIC_LIGHT_COLOR_GREEN: return "TRAFFIC_LIGHT_COLOR_GREEN"; break;
	case TRAFFIC_LIGHT_COLOR_GREEN_LEFT: return "TRAFFIC_LIGHT_COLOR_GREEN_LEFT"; break;
	case TRAFFIC_LIGHT_COLOR_RED_LEFT: return "TRAFFIC_LIGHT_COLOR_RED_LEFT"; break;
	default: break;
	}

	return "unknown";
}


#if defined(USE_LOCAL_MAP_DATA)
int proc_traffic_light_information(u8 *node_id, u8 ev_state, u32 remain_time)
{
	int traffic_light_type = 0;
	u8 traffic_light_id[16] = {0,};
    traffic_and_direction_info_t *tr_info = NULL;
    st_TR_SIGNAL *store_info = NULL;

	if ((tr_info = get_traffic_and_direction_info((const char*)node_id)) == NULL)
	{
		SWM_LOG("%s proc_traffic_light_information: fail to find traffic light id and link (%s) direction.\n", timeval_print(0), node_id);
		return -1;
	}

    if ((traffic_light_type = get_tr_light_info(tr_info->first)) < 0)
	{
		SWM_LOG("%s proc_traffic_light_information: fail to find traffic light type information by traffic light id(%s).\n", timeval_print(0), tr_info->first.c_str());
		return -1;
	}

	// type 2 traffic light
    if (traffic_light_type == TYPE_TRAFFIC_LIGHT_STR_LEFT)
    {
        if ((store_info = get_traffic_signal(tr_info->first.c_str())) != NULL)
        {
            // merge
            if ((store_info->set_flag & 0xff) != (tr_info->second & 0xff))
            {
				ev_state = (unsigned char)get_traffic_light_color(ev_state, traffic_light_type);
				ev_state = (unsigned char)get_traffic_light_direction_color(ev_state, tr_info->second);
				store_info->event_state = (u8)get_merged_traffic_light_color(store_info->event_state, store_info->end_time, ev_state, remain_time);
				store_info->set_flag |= tr_info->second & 0xff;
            }
            // update
            else
            {
                memset(store_info, 0x00, sizeof(st_TR_SIGNAL));
                strncpy((char*)store_info->traffic_light_id, tr_info->first.c_str(), tr_info->first.length());
                store_info->event_state = (unsigned char)get_traffic_light_color(ev_state, traffic_light_type);
				store_info->event_state = (unsigned char)get_traffic_light_direction_color(store_info->event_state, tr_info->second);
                store_info->end_time = remain_time;
                store_info->traffic_light_type = traffic_light_type;
                store_info->set_flag = tr_info->second & 0xff;
            }
        }
        else
        {
            store_info = new st_TR_SIGNAL();
            memset(store_info, 0x00, sizeof(st_TR_SIGNAL));
            strncpy((char*)store_info->traffic_light_id, tr_info->first.c_str(), tr_info->first.length());
            store_info->event_state = (unsigned char)get_traffic_light_color(ev_state, traffic_light_type);
			store_info->event_state = (unsigned char)get_traffic_light_direction_color(store_info->event_state, tr_info->second);
            store_info->end_time = remain_time;
            store_info->traffic_light_type = traffic_light_type;
            store_info->set_flag = tr_info->second & 0xff;
            add_traffic_signal(store_info);
        }
    }
	// type 1 (or type 4) traffic light
    else
    {
		store_info = new st_TR_SIGNAL();
		memset(store_info, 0x00, sizeof(st_TR_SIGNAL));
		strncpy((char*)store_info->traffic_light_id, tr_info->first.c_str(), tr_info->first.length());
		store_info->event_state = (unsigned char)get_traffic_light_color(ev_state, traffic_light_type);
		store_info->end_time = remain_time;
		store_info->traffic_light_type = traffic_light_type;
		store_info->set_flag = tr_info->second & 0xff;
		add_traffic_signal(store_info);
    }

    return 0;
}
#else
int proc_traffic_light_information(u8 *from_node_id, u8 *to_node_id, u8 ev_state, u32 remain_time)
{
	int traffic_light_type = 0;
	u8 traffic_light_id[16] = {0,};
    intersection_link_info_t *link_info = NULL;
    st_TR_SIGNAL *store_info = NULL;

	if (get_traffic_light_id((const char*)from_node_id, (char*)traffic_light_id, traffic_light_type) != 0)
	{
		SWM_INFO(CLI_INFO_SIG, "proc_traffic_light_information: traffic light process exception. not found traffic light by (from_link_id - %s).\n", from_node_id);
		return 0;
	}

	if ((link_info = get_intersection_link_info((const char*)to_node_id)) == NULL)
	{
		SWM_LOG("%s proc_traffic_light_information: fail to find intersection link information by to_link_id(%s).\n", timeval_print(0), to_node_id);
		return -1;
	}

	if ((link_info->first == TYPE_LINK_RIGHT) || (link_info->first == TYPE_LINK_UTURN))
	{
		// if right or u-turn
		SWM_INFO(CLI_INFO_SIG, "proc_traffic_light_information: traffic light process exception.  no needs processing right or u-turn link(to_link_id - %s).\n", to_node_id);
		return 0;
	}

    if ((store_info = get_traffic_signal((const char*)traffic_light_id)) != NULL)
    {
        // merge
        if ((store_info->set_flag & 0xff) != (link_info->first & 0xff))
        {
			ev_state = (unsigned char)get_traffic_light_color(ev_state);
			ev_state = (unsigned char)get_traffic_light_direction_color(ev_state, link_info->first);
			store_info->event_state = (u8)get_merged_traffic_light_color(store_info->event_state, store_info->end_time, ev_state, remain_time);
			store_info->set_flag |= link_info->first & 0xff;
        }
    }
    else
    {
        store_info = new st_TR_SIGNAL();
        memset(store_info, 0x00, sizeof(st_TR_SIGNAL));
        strncpy((char*)store_info->traffic_light_id, (const char*)traffic_light_id, strlen((const char*)traffic_light_id));
        store_info->event_state = (unsigned char)get_traffic_light_color(ev_state);
		store_info->event_state = (unsigned char)get_traffic_light_direction_color(store_info->event_state, link_info->first);
		store_info->end_time = remain_time;
        store_info->traffic_light_type = traffic_light_type;
        store_info->set_flag = link_info->first & 0xff;
        add_traffic_signal(store_info);
    }
	
    return 0;
}
#endif

void swm_ldm_msg_log(swm_ldm::st_SWMLDM& msg)
{
	// if msg have SPaT
	if (msg.payloadMask & SWMLDM_PAYLOAD_SIG_FLAG)
	{
		char time_val_str[32] = {0,};
		strncpy(time_val_str, timeval_print(0), 26);
		for (int i = 0; i < msg.trSignal.size(); i++)
		{
			swm_ldm::st_TRSIGNAL ldm_data = msg.trSignal[i];
			SWM_INFO(CLI_INFO_CAV, "%s Traffic Signal message[%d] - traffic light id : %s, traffic light signal : %d, remain time : %d\r\n", 
						time_val_str,
						i + 1,
						ldm_data.trafficLightId.c_str(),
						ldm_data.evState,
						ldm_data.endTime);
		}
	}
	// if msg have BSM
	if (msg.payloadMask & SWMLDM_PAYLOAD_BSM_FLAG)
	{
		SWM_INFO(CLI_INFO_CAV, "%s BMS message sent\n", timeval_print(0));
	}
	// if msg have RSA
	if (msg.payloadMask & SWMLDM_PAYLOAD_RSA_FLAG)
	{
		swm_ldm::st_RSA rsa_data = msg.rsaMsg[0];
		SWM_INFO(CLI_INFO_CAV, "%s RSA message - itis:%d, latutidue:%d, longitude:%d\r\n", 
				timeval_print(0),
				rsa_data.itis, 
				rsa_data.latitude, 
				rsa_data.longitude);
	}
	// if msg have EVA
	if (msg.payloadMask & SWMLDM_PAYLOAD_EVA_FLAG)
	{
		SWM_INFO(CLI_INFO_CAV, "%s EVA message sent\n", timeval_print(0));
	}
	// if msg have RCP
	if (msg.payloadMask & SWMLDM_PAYLOAD_RCP_FLAG)
	{
		SWM_INFO(CLI_INFO_CAV, "%s RCP message sent\n", timeval_print(0));
	}
}
