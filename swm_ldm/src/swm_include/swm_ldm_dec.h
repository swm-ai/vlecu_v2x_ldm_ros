/*
 * swm_ldm_dec.h
 *
 *  Created on: May 17, 2021
 *      Author: yhcho
 */

#ifndef SWM_LDM_DEC_H_
#define SWM_LDM_DEC_H_

#include "ros/ros.h"
#include "swm_define.h"
#include "swm_ros.h"
#include "swm_ldm_msg.h"
#include "swm_cav_data.h"
#include "swm_ldm/st_SWMLDM.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWMLDM_PAYLOAD_SIG_FLAG	0x01
#define SWMLDM_PAYLOAD_BSM_FLAG	0x02
#define SWMLDM_PAYLOAD_RSA_FLAG	0x04
#define SWMLDM_PAYLOAD_EVA_FLAG	0x08
#define SWMLDM_PAYLOAD_CAV_BSM_FLAG 0x10
#define SWMLDM_PAYLOAD_RCP_FLAG 0x20

#define	SWMLDM_CHECK_PL_FLG(x, y)	check_ldm_payload(x, y)
#define SWMLDM_SET_PL_FLG(x, y)	set_ldm_payload(x, y)

#define	SWMLDM_HAVE_SIG_PL(x)	SWMLDM_CHECK_PL_FLG(x, (u16)SWMLDM_PAYLOAD_SIG_FLAG)
#define SWMLDM_HAVE_BSM_PL(x)	SWMLDM_CHECK_PL_FLG(x, (u16)SWMLDM_PAYLOAD_BSM_FLAG)
#define SWMLDM_HAVE_RSA_PL(x)	SWMLDM_CHECK_PL_FLG(x, (u16)SWMLDM_PAYLOAD_RSA_FLAG)
#define SWMLDM_HAVE_EVA_PL(x)	SWMLDM_CHECK_PL_FLG(x, (u16)SWMLDM_PAYLOAD_EVA_FLAG)
#define SWMLDM_HAVE_CAV_BSM_PL(x)   SWMLDM_CHECK_PL_FLG(x, (u16)SWMLDM_PAYLOAD_CAV_BSM_FLAG)
#define SWMLDM_HAVE_RCP_PL(x)  SWMLDM_CHECK_PL_FLG(x, (u16)SWMLDM_PAYLOAD_RCP_FLAG)

#define SWMLDM_SET_SIG_PL(x)	SWMLDM_SET_PL_FLG(x, (u16)SWMLDM_PAYLOAD_SIG_FLAG)
#define SWMLDM_SET_BSM_PL(x)	SWMLDM_SET_PL_FLG(x, (u16)SWMLDM_PAYLOAD_BSM_FLAG)
#define SWMLDM_SET_RSA_PL(x)	SWMLDM_SET_PL_FLG(x, (u16)SWMLDM_PAYLOAD_RSA_FLAG)
#define SWMLDM_SET_EVA_PL(x)	SWMLDM_SET_PL_FLG(x, (u16)SWMLDM_PAYLOAD_EVA_FLAG)
#define SWMLDM_SET_CAV_BSM_PL(x)    SWMLDM_SET_PL_FLG(x, (u16)SWMLDM_PAYLOAD_CAV_BSM_FLAG)
#define SWMLDM_SET_RCP_PL(x)   SWMLDM_SET_PL_FLG(x, (u16)SWMLDM_PAYLOAD_RCP_FLAG)

extern int check_ldm_payload(u16 b, u16 f);
extern void set_ldm_payload(u16 *b, u16 f);

extern int get_ldm_field_from_msg(void* field, void* msg, int f_size);
extern int get_heading_azimuth(double **pos, int pos_num);

extern int decode_header(u8* pbuf, st_LDMMsgHeader* hdr);
extern int decode_position(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_path_control_message(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_path_control_data(st_LDMMsgHeader* hdr, u8* pbuf, std::map<int, int>& path_list);
extern int decode_basic_safety(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_BSM& bsm);
extern int decode_cfm_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_CAVBSM& cavbsm);
extern int decode_csm_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_CAVBSM& cavbsm);
extern int decode_clcm_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_CAVBSM& cavbsm);
extern int decode_rcp_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_RCP& rcp);
extern int decode_emergency_vehicle_alert(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_EVA& eva);
extern int decode_traffic_signal(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_signal_group_information(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_link_information(st_LDMMsgHeader* hdr, u8* pbuf, u8 state, u32 remain_time);
extern int decode_road_side_alert(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_RSA& rsa);
extern int decode_link(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_points(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_points_and_get_turn_type(st_LDMMsgHeader* hdr, u8* pbuf, double *position);
extern int decode_temp_node(st_LDMMsgHeader* hdr, u8* pbuf);
extern int decode_traffic_light(st_LDMMsgHeader* hdr, u8* pbuf);

extern int addSIGtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_TRSIGNAL& trsignal);
extern int addBSMtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_BSM& bsm);
extern int addRSAtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_RSA& rsa);
extern int addEVAtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_EVA& eva);
extern int addCAVBSMtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_CAVBSM& cavbsm);
extern int addRCPtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_RCP& rcp);
extern int addCFMtoCAVBSM(swm_ldm::st_CAVBSM& cavbsm, swm_ldm::st_CFM& cfm);
extern int addCSMtoCAVBSM(swm_ldm::st_CAVBSM& cavbsm, swm_ldm::st_CSM& csm);
extern int addCLCMtoCAVBSM(swm_ldm::st_CAVBSM& cavbsm, swm_ldm::st_CLCM& clcm);

extern int publish_traffic_signal(void);

extern int setTRSIGNAL(swm_ldm::st_TRSIGNAL& data, st_TR_SIGNAL* val);

extern int ldm_message_decoder(st_LDMMsgHeader* hdr, u8* msg_body);

// for test
extern void test_traffic_signal_process(void);
extern void test_road_side_alert(void);

#ifdef __cplusplus
}
#endif

#endif /* SWM_LDM_DEC_H_ */
