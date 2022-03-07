/*
 * swm_ldm_data.h
 *
 *  Created on: May 22, 2021
 *      Author: yhcho
 */

#ifndef SWM_LDM_DATA_H_
#define SWM_LDM_DATA_H_

#include <string>
#include <utility>
#include <map>
#include <vector>

#include "swm_ldm/st_SWMLDM.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

#define SWMLDM_PAYLOAD_SIG_FLAG	0x01
#define SWMLDM_PAYLOAD_BSM_FLAG	0x02
#define SWMLDM_PAYLOAD_RSA_FLAG	0x04
#define SWMLDM_PAYLOAD_EVA_FLAG	0x08
#define SWMLDM_PAYLOAD_MOBJ_FLAG 0x20

extern pthread_mutex_t* get_mutex_ldm_monitor(void);
extern void ldm_monitor_lock(void);
extern void ldm_monitor_unlock(void);

extern int add_traffic_signal_data_list(swm_ldm::st_TRSIGNAL& traffic_light_signal);
extern std::map<string, swm_ldm::st_TRSIGNAL>& get_traffic_signal_data_list(void);
extern void clear_traffic_signal_data_list(void);

extern int add_rsa_data_list(swm_ldm::st_RSA& rsa);
extern void clear_rsa_data_list(void);
extern std::vector<swm_ldm::st_RSA>& get_rsa_data_list(void);

extern int add_eva_data_list(swm_ldm::st_EVA& eva);
extern void clear_eva_data_list(void);
extern std::vector<swm_ldm::st_EVA>& get_eva_data_list(void);

extern int add_bsm_data_list(swm_ldm::st_BSM& bsm);
extern void clear_bsm_data_list(void);
extern std::vector<swm_ldm::st_BSM>& get_bsm_data_list(void);

extern int add_mobj_data_list(swm_ldm::st_MOBJ& mobj);
extern void clear_mobj_data_list(void);
extern std::vector<swm_ldm::st_MOBJ>& get_mobj_data_list(void);

extern void display_ldm_monitor_title(void);
extern void display_traffic_light_signal(void);
extern void display_road_side_alert(void);
extern void display_emergency_vehicle_alert(void);
extern void display_bsm_data(void);
extern void display_mobj_data(void);

extern void getcurtime(struct timeval *ot);
extern long gettimediff(struct timeval *ot, struct timeval *dt);

#ifdef __cplusplus
}
#endif

#endif /* SWM_LDM_DATA_H_ */
