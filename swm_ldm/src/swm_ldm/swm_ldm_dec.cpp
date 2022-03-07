/*
 * swm_ldm_dec.cpp
 *
 *  Created on: May 17, 2021
 *      Author: yhcho
 */

#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>

#include "swm_define.h"
#include "swm_common.h"
#include "swm_log.h"
#include "swm_cav_data.h"
#include "swm_ldm_dec.h"


int ldm_message_decoder(st_LDMMsgHeader* hdr, u8* msg_body)
{
    switch (hdr->unMsgType)
    {
    // MAP message
    case PROFILE_TYPE_POSITION :
        {
            SWM_INFO(CLI_INFO_POS, "%s received position message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            decode_position(hdr, msg_body);
            break;
        }
    case PROFILE_TYPE_PATH_CONTROL :
        {
            SWM_INFO(CLI_INFO_POS, "%s received path control message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            decode_path_control_message(hdr, msg_body);
            break;
        }
    case PROFILE_TYPE_NODE :
        {
            SWM_INFO(CLI_INFO_POS, "%s received node message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            decode_temp_node(hdr, msg_body);
            break;
        }
    case PROFILE_TYPE_LINK :
        {
            SWM_INFO(CLI_INFO_MAP, "%s received link mesage. size : %d\n", timeval_print(0), hdr->unMsgSize);
            decode_link(hdr, msg_body);
            break;
        }
    case PROFILE_TYPE_TRAFFIC_LIGHT :
        {
            SWM_INFO(CLI_INFO_MAP, "%s received traffic light mesage. size : %d\n", timeval_print(0), hdr->unMsgSize);
            decode_traffic_light(hdr, msg_body);
            break;
        }
    // V2X receiving message
    case PROFILE_TYPE_TRAFFIC_SIGNAL :
        {
            SWM_INFO(CLI_INFO_SIG, "%s received traffic signal message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            if (decode_traffic_signal(hdr, msg_body) > 0)
            {
                if (publish_traffic_signal() < 0)
                {
                    SWM_LOG("%s publish_traffic_signal : fail to publish traffic signal.\n", timeval_print(0));
                }
            }
            break;
        }
    case PROFILE_TYPE_ROAD_SIDE_ALERT :
        {
            SWM_INFO(CLI_INFO_RSA, "%s received road side alert message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_RSA rsa_data = swm_ldm::st_RSA();
            if (decode_road_side_alert(hdr, msg_body, rsa_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addRSAtoSWMLDM(ldm_msg, rsa_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
#if defined(USE_DETAIL_SOCKET_LOG)
                    SWM_INFO(CLI_INFO_RSA, "%s sent RSA data to CAV\r\n", timeval_print(0));
#endif  // USE_DETAIL_SOCKET_LOG
                }
            }
            break;
        }
    case PROFILE_TYPE_ROADWAY_CROSSING_PEDESTRIAN_INFO :
        {
            SWM_INFO(CLI_INFO_RCP, "%s received roadway crossing pedestrian info message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_RCP rcp_data = swm_ldm::st_RCP();
            if (decode_rcp_data(hdr, msg_body, rcp_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addRCPtoSWMLDM(ldm_msg, rcp_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
                    SWM_INFO(CLI_INFO_RCP, "%s sent RCP data to CAV\r\n", timeval_print(0));
                }
            }
            break;
        }
    case PROFILE_TYPE_BASIC_SAFETY :
        {
            SWM_INFO(CLI_INFO_BSM, "%s received basic safety message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_BSM bsm_data = swm_ldm::st_BSM();
            if (decode_basic_safety(hdr, msg_body, bsm_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addBSMtoSWMLDM(ldm_msg, bsm_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
                    SWM_INFO(CLI_INFO_BSM, "%s sent BSM data to CAV\r\n", timeval_print(0));
                }
            }
            break;
        }
    case PROFILE_TYPE_CFM :
        {
            SWM_INFO(CLI_INFO_CFM, "%s received CFM message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_CAVBSM cavbsm_data = swm_ldm::st_CAVBSM();
            if (decode_cfm_data(hdr, msg_body, cavbsm_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addCAVBSMtoSWMLDM(ldm_msg, cavbsm_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
                    SWM_INFO(CLI_INFO_CFM, "%s sent CFM data to CAV\r\n", timeval_print(0));
                }
            }
            break;
        }
    case PROFILE_TYPE_CSM :
        {
            SWM_INFO(CLI_INFO_CSM, "%s received CSM message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_CAVBSM cavbsm_data = swm_ldm::st_CAVBSM();
            if (decode_csm_data(hdr, msg_body, cavbsm_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addCAVBSMtoSWMLDM(ldm_msg, cavbsm_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
                    SWM_INFO(CLI_INFO_CSM, "%s sent CSM data to CAV\r\n", timeval_print(0));
                }
            }
            break;
        }
    case PROFILE_TYPE_CLCM :
        {
            SWM_INFO(CLI_INFO_CLC, "%s received CLCM message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_CAVBSM cavbsm_data = swm_ldm::st_CAVBSM();
            if (decode_clcm_data(hdr, msg_body, cavbsm_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addCAVBSMtoSWMLDM(ldm_msg, cavbsm_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
                    SWM_INFO(CLI_INFO_CLC, "%s sent CLCM data to CAV\r\n", timeval_print(0));
                }
            }
            break;
        }
    case PROFILE_TYPE_EMERGENCY_VEHICLE_ALERT :
        {
            SWM_INFO(CLI_INFO_EVA, "%s received emergency vehicle alert message. size : %d\n", timeval_print(0), hdr->unMsgSize);
            swm_ldm::st_EVA eva_data = swm_ldm::st_EVA();
            if (decode_emergency_vehicle_alert(hdr, msg_body, eva_data) > 0)
            {
                swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
                if (addEVAtoSWMLDM(ldm_msg, eva_data) > 0)
                {
                    swm_ldm_msg_publish(ldm_msg);
                    SWM_INFO(CLI_INFO_EVA, "%s sent EVA data to CAV\r\n", timeval_print(0));
                }
            }
            break;
        }
    // V2X receiving message // not implemented yet
    case PROFILE_TYPE_EMERGENCY_VEHICLE_INFO :
    case PROFILE_TYPE_PEDESTRIAN_RISK_INFO :
    case PROFILE_TYPE_LANE_QUEUE_INFORMATION :
    //case PROFILE_TYPE_PROBE_VEHICLE_DATA :
    // MAP message // not implemented
    case PROFILE_TYPE_DRIVE_WAY_SECTION :
    case PROFILE_TYPE_SAFETY_SIGN :
    case PROFILE_TYPE_SURFACE_LINE_MARK :
    case PROFILE_TYPE_SURFACE_MARK :
    case PROFILE_TYPE_VEHICLE_PROTECTION_SAFETY :
    case PROFILE_TYPE_SPEED_BUMP :
    case PROFILE_TYPE_POST_POINT :
    case PROFILE_TYPE_SD_NODE :
    case PROFILE_TYPE_SD_LINK :
        {
            SWM_INFO(CLI_INFO_MAP, "%s received map message. type : %d, size : %d\n", timeval_print(0), hdr->unMsgType, hdr->unMsgSize);
            //swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();  // for test
            //swm_ldm_msg_publish(ldm_msg);   // for test
            break;
        }
    default:
        SWM_INFO(CLI_INFO_ETC, "%s received unknown message. type : %d, size : %d\n", timeval_print(0), hdr->unMsgType, hdr->unMsgSize);
        break;
    }

    return -1;
}


// signal // j2735 SPaT
int addSIGtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_TRSIGNAL& trsignal)
{
    msg.trSignal.push_back(trsignal);
    SWMLDM_SET_SIG_PL(&msg.payloadMask);
	return 1;
}

// BSM
int addBSMtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_BSM& bsm)
{
    msg.bsmMsg.push_back(bsm);
	SWMLDM_SET_BSM_PL(&msg.payloadMask);
	return 1;
}

// RSA
int addRSAtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_RSA& rsa)
{
    msg.rsaMsg.push_back(rsa);
	SWMLDM_SET_RSA_PL(&msg.payloadMask);
	return 1;
}

// EVA
int addEVAtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_EVA& eva)
{
    msg.evaMsg.push_back(eva);
	SWMLDM_SET_EVA_PL(&msg.payloadMask);
	return 1;
}

// CAVBSM
int addCAVBSMtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_CAVBSM& cavbsm)
{
    msg.cavbsm.push_back(cavbsm);
    SWMLDM_SET_CAV_BSM_PL(&msg.payloadMask);
    return 1;
}

// RCP
int addRCPtoSWMLDM(swm_ldm::st_SWMLDM& msg, swm_ldm::st_RCP& rcp)
{
    msg.rcp.push_back(rcp);
    SWMLDM_SET_RCP_PL(&msg.payloadMask);
    return 1;
}

// CFM
int addCFMtoCAVBSM(swm_ldm::st_CAVBSM& cavbsm, swm_ldm::st_CFM& cfm)
{
    cavbsm.covoyfoamtion.push_back(cfm);
    return 1;
}

// CSM
int addCSMtoCAVBSM(swm_ldm::st_CAVBSM& cavbsm, swm_ldm::st_CSM& csm)
{
    cavbsm.covoystatus.push_back(csm);
    return 1;
}

// CLCM
int addCLCMtoCAVBSM(swm_ldm::st_CAVBSM& cavbsm, swm_ldm::st_CLCM& clcm)
{
    cavbsm.convoylanechange.push_back(clcm);
    return 1;
}

int publish_traffic_signal(void)
{
    bool is_send = true;
    swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
    traffic_signal_list_t& traffic_list = get_sending_traffic_signal_list();

    for (auto &traffic_data : traffic_list)
    {
        swm_ldm::st_TRSIGNAL sig_data = swm_ldm::st_TRSIGNAL();
        setTRSIGNAL(sig_data, traffic_data.second);
        delete traffic_data.second;
        traffic_data.second = NULL;

        if (addSIGtoSWMLDM(ldm_msg, sig_data) < 0)
        {
            is_send = false;
            // if error but continue.
            // so, all traffic_data.second(st_TR_SIGNAL pointer) is cleared.
        }
    }

    clear_sending_traffic_signal_list();

    if (is_send)
    {
        swm_ldm_msg_publish(ldm_msg);
#if defined(USE_DETAIL_SOCKET_LOG)
        SWM_INFO(CLI_INFO_SIG, "%s sent traffic signal data to CAV\r\n", timeval_print(0));
#endif  // USE_DETAIL_SOCKET_LOG
        return 0;
    }

    return -1;
}

int setTRSIGNAL(swm_ldm::st_TRSIGNAL& data, st_TR_SIGNAL* val)
{
    if (val == NULL) return -1;

    data.trafficLightId = std::string((const char*)val->traffic_light_id);
    data.evState = val->event_state;
    data.endTime = val->end_time;// / 10;

    //SWM_INFO(CLI_INFO_SIG, "<<<<< traffic id : %s, event_state : %s, remain time : %d\n", (const char*)val->traffic_light_id, get_traffic_ligt_color_str(val->event_state), val->end_time);

    return 0;
}

int check_ldm_payload(u16 b, u16 f)
{
	return (b&f)!=0x00?1:0;
}

void set_ldm_payload(u16 *b, u16 f)
{
	*b |= f;
}


int decode_header(u8* pbuf, st_LDMMsgHeader* hdr)
{
    int pos = 0;
    long long tstamp = 0;
    memset(hdr, 0x00, sizeof(st_LDMMsgHeader));

    pos += get_ldm_field_from_msg(&tstamp, pbuf + pos, sizeof(tstamp));
    hdr->qTimeStamp = ntohll(tstamp);

    pos += get_ldm_field_from_msg(&hdr->unMsgSize, pbuf + pos, sizeof(hdr->unMsgSize));
    hdr->unMsgSize = ntohl(hdr->unMsgSize);

    pos += get_ldm_field_from_msg(&hdr->unMsgType, pbuf + pos, sizeof(hdr->unMsgType));
    hdr->unMsgType = ntohl(hdr->unMsgType);

    pos += get_ldm_field_from_msg(&hdr->unPathId, pbuf + pos, sizeof(hdr->unPathId));
    hdr->unPathId = ntohl(hdr->unPathId);

    pos += get_ldm_field_from_msg(&hdr->unLinkId, pbuf + pos, sizeof(u8) * 12);

#if defined(USE_DETAIL_SOCKET_LOG)
    SWM_LOG("===== header info start\n");
    SWM_LOG("\tTimeStamp : %s\n", utctime_print(hdr->qTimeStamp));
    SWM_LOG("\tMsgSize : %d\n", hdr->unMsgSize);
    SWM_LOG("\tMsgType : %d\n", hdr->unMsgType);
    SWM_LOG("\tPathId : %d\n", hdr->unPathId);
    SWM_LOG("\tLinkId : %s\n", hdr->unLinkId);
    SWM_LOG("===== header info end\n");
#endif  // USE_DETAIL_SOCKET_LOG

    return pos;
}

int decode_position(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    long long tstamp = 0;
    double longitude = 0, latitude = 0, altitude = 0;
    //static int count = 0;

    SWM_INFO(CLI_INFO_POS, "===== position info start\n");

    // qTimeStamp	long long	8	-	-	-	time
    pos += get_ldm_field_from_msg(&tstamp, pbuf + pos, sizeof(tstamp));
    tstamp = ntohll(tstamp);
    // stAbsolutePosition	DoubleUTM	24	-	-	-	-
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    //pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(latitude));
    memcpy(&latitude, pbuf + pos, sizeof(latitude));
    pos += sizeof(latitude);
    latitude = ntohdouble(latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    //pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(longitude));
    memcpy(&longitude, pbuf + pos, sizeof(longitude));
    pos += sizeof(longitude);
    longitude = ntohdouble(longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    //pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(altitude));
    memcpy(&altitude, pbuf + pos, sizeof(altitude));
    pos += sizeof(altitude);
    altitude = ntohdouble(altitude);

    SWM_INFO(CLI_INFO_POS, "\tutctime : %lld\n", tstamp);
    SWM_INFO(CLI_INFO_POS, "\tstAbsolutePosition (DoubleUTM)\n");
    SWM_INFO(CLI_INFO_POS, "\t\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_POS, "\t\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_POS, "\t\taltitue : %lf\n", altitude);

    set_current_position((const char*)hdr->unLinkId);
    //set_current_position(/*++count > 40 ?*/ (const char*)"A219AW000059" /*: (const char*)hdr->unLinkId*/);

    /*if (count > 40)
    {
        test_current_link_list();
    }*/

    SWM_INFO(CLI_INFO_POS, "===== position info end\n");

    return pos;
}

int decode_path_control_message(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    u32 id_first = 0, id_last = 0, num_values = 0;
    std::map<int, int> path_list;

    SWM_INFO(CLI_INFO_MAP, "===== path control message info start\n");

    // nIdFirst	unsigned int	4	0	UINT_MAX	0	-
    pos += get_ldm_field_from_msg(&id_first, pbuf + pos, sizeof(u32));
    id_first = ntohl(id_first);
    // nIdLast	unsigned int	4	0	UINT_MAX	0	-
    pos += get_ldm_field_from_msg(&id_last, pbuf + pos, sizeof(u32));
    id_last = ntohl(id_last);
    // nNoValues	unsigned int	4	0	UINT_MAX	0	-
    pos += get_ldm_field_from_msg(&num_values, pbuf + pos, sizeof(u32));
    num_values = ntohl(num_values);

    SWM_INFO(CLI_INFO_MAP, "\tnIdFirst : %d\n", id_first);
    SWM_INFO(CLI_INFO_MAP, "\tnIdLast : %d\n", id_last);
    SWM_INFO(CLI_INFO_MAP, "\tnNoValues : %d\n", num_values);

    // vcValues	vector<PathControl>	-	-	-	-	-
    for (int i = 0; i < num_values && pos < (hdr->unMsgSize - sizeof(st_LDMMsgHeader)); i++)
    {
        SWM_INFO(CLI_INFO_MAP, "\t----- path control %d info start\n", i + 1);
        // nPathId	unsigned int	4	0	UINT_MAX	0	-
        // nParentPathId	unsigned int	4	0	UINT_MAX	0	-
        pos += decode_path_control_data(hdr, pbuf + pos, path_list);
        SWM_INFO(CLI_INFO_MAP, "\t----- path control %d info end\n", i + 1);
    }

    if (get_hdmap_info_number() > 0)
    {
        resize_hdmap_info(path_list);
    }

    for (auto element : path_list)
    {
        add_hdmap_info(element.first);
    }

    path_list.clear();

    SWM_INFO(CLI_INFO_MAP, "===== path control message info end\n");

    return pos;
}

int decode_path_control_data(st_LDMMsgHeader* hdr, u8* pbuf, std::map<int, int>& path_list)
{
    int pos = 0;
    u32 path_id = 0, parent_path_id = 0;

    // vcValues	vector<PathControl>	-	-	-	-	-
    // nPathId	unsigned int	4	0	UINT_MAX	0	-
    pos += get_ldm_field_from_msg(&path_id, pbuf + pos, sizeof(u32));
    path_id = ntohl(path_id);
    // nParentPathId	unsigned int	4	0	UINT_MAX	0	
    pos += get_ldm_field_from_msg(&parent_path_id, pbuf + pos, sizeof(u32));
    parent_path_id = ntohl(parent_path_id);

    SWM_INFO(CLI_INFO_MAP, "\t\tnPathId : %d\n", path_id);
    SWM_INFO(CLI_INFO_MAP, "\t\tnParentPathId : %d\n", parent_path_id);

    path_list.insert({path_id, parent_path_id});

    return pos;
}

int decode_basic_safety(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_BSM& bsm)
{
    int pos = 0;
    s16 shortval = 0;
    s32 intval = 0;

    SWM_INFO(CLI_INFO_BSM, "===== basic safety message info start\n");

    // chVehicleId	unsinged char[4]	4		
    pos += get_ldm_field_from_msg(&bsm.id[0], pbuf + pos, sizeof(u8) * 4);		
    // chTransmission	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.transmission, pbuf + pos, sizeof(u8));
    // ucReserved	unsigned char[3]	3				
    pos += 3 * sizeof(u8);
    // nSpeed	int	4	
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));			
    bsm.speed = (u16)ntohl(intval);
    // nHeading	int	4				
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));
    bsm.heading = (u16)ntohl(intval);
    // nAngle	int	4				
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));
    bsm.angle = (s16)ntohl(intval);
    // nsAccel_lon	short	2			
    pos += get_ldm_field_from_msg(&shortval, pbuf + pos, sizeof(s16));
    bsm.acc_long = ntohs(shortval);	
    // nsAccel_lat	short	2		
    pos += get_ldm_field_from_msg(&shortval, pbuf + pos, sizeof(s16));
    bsm.acc_lat = ntohs(shortval);		
    // nVert	int	4			
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));
    bsm.acc_vert = (u8)ntohl(intval);	
    // nYaw	int	4				
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));
    bsm.acc_yaw = (s16)ntohl(intval);
    // chWheelbreaks	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.wheelbrakes, pbuf + pos, sizeof(u8));
    // chTraction	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.traction, pbuf + pos, sizeof(u8));
    // chAbs	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.abs, pbuf + pos, sizeof(u8));
    // chScs	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.scs, pbuf + pos, sizeof(u8));
    // chBreakBoost	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.brakeboost, pbuf + pos, sizeof(u8));
    // chAuxBrakes	unsigned char	1				
    pos += get_ldm_field_from_msg(&bsm.auxbrakes, pbuf + pos, sizeof(u8));
    // ucReserved	unsinged char[2]	2				
    pos += 2 * sizeof(u8);
    // nWidth	int	4	
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));
    bsm.width = ntohl(intval);
    // nLength	int	4				
    pos += get_ldm_field_from_msg(&intval, pbuf + pos, sizeof(s32));
    bsm.length = ntohl(intval);
    // stPoints	WGS84Point	24	-	-	-	-
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&bsm.lat, pbuf + pos, sizeof(double));
    bsm.lat = ntohdouble(bsm.lat);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&bsm.longi, pbuf + pos, sizeof(double));
    bsm.longi = ntohdouble(bsm.longi);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&bsm.elev, pbuf + pos, sizeof(double));
    bsm.elev = ntohdouble(bsm.elev);

    SWM_INFO(CLI_INFO_BSM, "\tvehicle id : 0x%02X%02X%02X%02X\n", bsm.id[0], bsm.id[1], bsm.id[2], bsm.id[3]);
    SWM_INFO(CLI_INFO_BSM, "\ttransmission : %d\n", bsm.transmission);
    SWM_INFO(CLI_INFO_BSM, "\tspeed : %d\n", bsm.speed);
    SWM_INFO(CLI_INFO_BSM, "\theading : %d\n", bsm.heading);
    SWM_INFO(CLI_INFO_BSM, "\tangle : %d\n", bsm.angle);
    SWM_INFO(CLI_INFO_BSM, "\taccel longitude : %d\n", bsm.acc_long);	
    SWM_INFO(CLI_INFO_BSM, "\taccel latitude : %d\n", bsm.acc_lat);
    SWM_INFO(CLI_INFO_BSM, "\taccel vert : %d\n", bsm.acc_vert);	
    SWM_INFO(CLI_INFO_BSM, "\taccel yaw : %d\n", bsm.acc_yaw);
    SWM_INFO(CLI_INFO_BSM, "\twheel brakes : %d\n", bsm.wheelbrakes);
    SWM_INFO(CLI_INFO_BSM, "\ttraction : %d\n", bsm.traction);
    SWM_INFO(CLI_INFO_BSM, "\tabs : %d\n", bsm.abs);
    SWM_INFO(CLI_INFO_BSM, "\tscs : %d\n", bsm.scs);
    SWM_INFO(CLI_INFO_BSM, "\tbrake boost : %d\n", bsm.brakeboost);
    SWM_INFO(CLI_INFO_BSM, "\taux brakes : %d\n", bsm.auxbrakes);
    SWM_INFO(CLI_INFO_BSM, "\twidth : %d\n", bsm.width);
    SWM_INFO(CLI_INFO_BSM, "\tlength : %d\n", bsm.length);
    SWM_INFO(CLI_INFO_BSM, "\tstPoints(WGS84Point)\n");
    SWM_INFO(CLI_INFO_BSM, "\tlatitude : %lf\n", bsm.lat);
    SWM_INFO(CLI_INFO_BSM, "\tlongitude : %lf\n", bsm.longi);
    SWM_INFO(CLI_INFO_BSM, "\taltitude : %lf\n", bsm.elev);

    SWM_INFO(CLI_INFO_BSM, "===== basic safety message info end\n");

    return pos;
}

int decode_cfm_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_CAVBSM& cavbsm)
{
    int pos = 0;
    u8 accept_response = 0;
    swm_ldm::st_CFM cfm_data = swm_ldm::st_CFM();
    swm_ldm::st_CVRES cvres_data;

    SWM_INFO(CLI_INFO_CFM, "===== cfm message info start\n");

    // chVehicleId	unsinged char[4]	4					차량의 ID
    pos += get_ldm_field_from_msg(&cavbsm.id[0], pbuf + pos, sizeof(u8) * 4);
    // nsConvoy_status	unsigned short	2					convoy 상태
    pos += get_ldm_field_from_msg(&cfm_data.convoystatus, pbuf + pos, sizeof(u16));
    cfm_data.convoystatus = ntohs((u16)cfm_data.convoystatus);
    // chConvoyid	unsigned char	4					Convoy id
    pos += get_ldm_field_from_msg(&cfm_data.convoyId[0], pbuf + pos, sizeof(u8) * 4);
    // chConvoy_join_request	unsigned char	1					convoy 합류 요청
    pos += get_ldm_field_from_msg(&cfm_data.convoyjoinrequest, pbuf + pos, sizeof(u8));
    // chConvoy_accept_response	unsigned char	1					convoy 합류 요청에 대한 응답
    pos += get_ldm_field_from_msg(&accept_response, pbuf + pos, sizeof(u8));
    if (accept_response == 1)
    {
        cvres_data = swm_ldm::st_CVRES();
        cvres_data.response = accept_response;
        // chConvoy_accept_msg_count	unsigned char	1					ConvoyJoinRequest 메시지 시퀀스 넘버
        pos += get_ldm_field_from_msg(&cvres_data.msgCnt, pbuf + pos, sizeof(u8));
        // ucReserved	unsigned char	3					reserved
        pos += sizeof(u8) * 3;
        // chConjoinreqvehicleID	unsigned char	4					ConvoyJoinRequest 송신 차량 임시ID
        pos += get_ldm_field_from_msg(&cvres_data.vid[0], pbuf + pos, sizeof(u8) * 4);

        cfm_data.convoyacceptresponse.push_back(cvres_data);
    } else {
        pos += sizeof(u8) * 8;
    }
    // chConvoy_leave_notice	unsigned char	1					convoy이탈 통지
    pos += get_ldm_field_from_msg(&cfm_data.convoyleavenotice, pbuf + pos, sizeof(u8));
    // ucReserved	unsigned char	3					reserved
    pos += sizeof(u8) * 3;
    addCFMtoCAVBSM(cavbsm, cfm_data);


    SWM_INFO(CLI_INFO_CFM, "\tchVehicleId : 0x%02X%02X%02X%02X\n", cavbsm.id[0], cavbsm.id[1], cavbsm.id[2], cavbsm.id[3]);
    SWM_INFO(CLI_INFO_CFM, "\tnsConvoy_status : %d\n", cfm_data.convoystatus);
    SWM_INFO(CLI_INFO_CFM, "\tchConvoyid : 0x%02X%02X%02X%02X\n", cfm_data.convoyId[0], cfm_data.convoyId[1], cfm_data.convoyId[2], cfm_data.convoyId[3]);
    // convoy join request
    if (cfm_data.convoyjoinrequest == 1)
    {
        SWM_INFO(CLI_INFO_CFM, "\tchConvoy_join_request : %d\n", cfm_data.convoyjoinrequest);
    }
    // convoy accept response
    if (accept_response == 1)
    {
        SWM_INFO(CLI_INFO_CFM, "\tchConvoy_accept_response : %d\n", accept_response);
        SWM_INFO(CLI_INFO_CFM, "\tchConvoy_accept_msg_count : %d\n", cvres_data.msgCnt);
        SWM_INFO(CLI_INFO_CFM, "\tchConjoinreqvehicleID : 0x%02X%02X%02X%02X\n", cvres_data.vid[0], cvres_data.vid[1], cvres_data.vid[2], cvres_data.vid[3]);
    }
    // convoy leave notice message
    if (cfm_data.convoyleavenotice == 1)
    {
        SWM_INFO(CLI_INFO_CFM, "\tchConvoy_leave_notice : %d\n", cfm_data.convoyleavenotice);
    }

    SWM_INFO(CLI_INFO_CFM, "===== cfm message info end\n");

    return pos;
}

int decode_csm_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_CAVBSM& cavbsm)
{
    int pos = 0;
    s32 convoy_id = 0;
    swm_ldm::st_CSM csm_data = swm_ldm::st_CSM();

    SWM_INFO(CLI_INFO_CSM, "===== csm message info start\n");

    // chVehicleId	unsinged char[4]	4					차량의 ID
    pos += get_ldm_field_from_msg(&cavbsm.id[0], pbuf + pos, sizeof(u8) * 4);
    // nsConvoy_status	unsigned short	2					convoy 상태
    pos += get_ldm_field_from_msg(&csm_data.convoystatus, pbuf + pos, sizeof(u16));
    // ucReserved	unsigned char[2]	2					reserved
    pos += sizeof(u8) * 2;
    // chConvoy_id	int	4					Convoy 임시 ID
    pos += get_ldm_field_from_msg(&convoy_id, pbuf + pos, sizeof(s32));
    convoy_id = ntohl(convoy_id);
    memcpy(&csm_data.convoyid[0], &convoy_id, sizeof(s32));
    addCSMtoCAVBSM(cavbsm, csm_data);


    SWM_INFO(CLI_INFO_CSM, "\tchVehicleId : 0x%02X%02X%02X%02X\n", cavbsm.id[0], cavbsm.id[1], cavbsm.id[2], cavbsm.id[3]);
    SWM_INFO(CLI_INFO_CSM, "\tnsConvoy_status : %d\n", csm_data.convoystatus);
    SWM_INFO(CLI_INFO_CSM, "\tchConvoy_id : 0x%02X%02X%02X%02X\n", csm_data.convoyid[0], csm_data.convoyid[1], csm_data.convoyid[2], csm_data.convoyid[3]);

    SWM_INFO(CLI_INFO_CSM, "===== csm message info end\n");

    return pos;
}

int decode_clcm_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_CAVBSM& cavbsm)
{
    int pos = 0;
    u8 change_response = 0;
    swm_ldm::st_CLCM clcm_data = swm_ldm::st_CLCM();
    swm_ldm::st_CVRES cvres_data;

    SWM_INFO(CLI_INFO_CLC, "===== clcm message info start\n");

    // chVehicleId	unsinged char[4]	4				
    pos += get_ldm_field_from_msg(&cavbsm.id[0], pbuf + pos, sizeof(u8) * 4);
    // nsConvoy_status	unsigned short	2				
    pos += get_ldm_field_from_msg(&clcm_data.convoystatus, pbuf + pos, sizeof(u16));
    clcm_data.convoystatus = ntohs(clcm_data.convoystatus);
    // chLane_change_request	unsigned char	1			
    pos += get_ldm_field_from_msg(&clcm_data.lanechangerequest, pbuf + pos, sizeof(u8));	
    // chPresent_lane	unsigned char	1				
    pos += get_ldm_field_from_msg(&clcm_data.presentlane, pbuf + pos, sizeof(u8));
    // chChanging_lane	unsigned char	1				
    pos += get_ldm_field_from_msg(&clcm_data.changinglane, pbuf + pos, sizeof(u8));
    // chHeadway	unsigned char	1				
    pos += get_ldm_field_from_msg(&clcm_data.headway, pbuf + pos, sizeof(u8));
    // chLane_change_response	unsigned char	1				
    pos += get_ldm_field_from_msg(&change_response, pbuf + pos, sizeof(u8));
    if (change_response == 1)
    {
        cvres_data = swm_ldm::st_CVRES();
        cvres_data.response = change_response;
        // chLane_change_msg_count	unsigned char	1
        pos += get_ldm_field_from_msg(&cvres_data.msgCnt, pbuf + pos, sizeof(u8));				
        // chLane_change_vehicle_id	char	4		
        pos += get_ldm_field_from_msg(&cvres_data.vid[0], pbuf + pos, sizeof(u8) * 4);		

        clcm_data.lanechangeresponse.push_back(cvres_data);
    } else {
        pos += sizeof(u8) * 5;
    }
    // chLane_change_notice	unsigned char	1				
    pos += get_ldm_field_from_msg(&clcm_data.lanechangenotice, pbuf + pos, sizeof(u8));
    // ucReserved	unsigned char[3]	3				
    pos += sizeof(u8) * 3;
    addCLCMtoCAVBSM(cavbsm, clcm_data);


    SWM_INFO(CLI_INFO_CLC, "\tchVehicleId : 0x%02X%02X%02X%02X\n", cavbsm.id[0], cavbsm.id[1], cavbsm.id[2], cavbsm.id[3]);
    SWM_INFO(CLI_INFO_CLC, "\tnsConvoy_status : %d\n", clcm_data.convoystatus);
    // lane change request message
    if (clcm_data.lanechangerequest == 1)
    {
        SWM_INFO(CLI_INFO_CLC, "\tchLane_change_request : %d\n", clcm_data.lanechangerequest);
        SWM_INFO(CLI_INFO_CLC, "\tchPresent_lane : %d\n", clcm_data.presentlane);
        SWM_INFO(CLI_INFO_CLC, "\tchChanging_lane : %d\n", clcm_data.changinglane);
        SWM_INFO(CLI_INFO_CLC, "\tchHeadway : %d\n", clcm_data.headway);
    }
    // lane change response message
    if (change_response == 1)
    {
        SWM_INFO(CLI_INFO_CLC, "\tchLane_change_response : %d\n", change_response);
        SWM_INFO(CLI_INFO_CLC, "\tchLane_change_msg_count : %d\n", cvres_data.msgCnt);
        SWM_INFO(CLI_INFO_CLC, "\tchLane_change_vehicle_id : 0x%02X%02X%02X%02X\n", cvres_data.vid[0], cvres_data.vid[1], cvres_data.vid[2], cvres_data.vid[3]);
    }
    // lane change notice message
    if (clcm_data.lanechangenotice > 0)
    {
        SWM_INFO(CLI_INFO_CLC, "\tchLane_change_notice : %d\n", clcm_data.lanechangenotice);
    }

    SWM_INFO(CLI_INFO_CLC, "===== clcm message info end\n");

    return pos;
}

int decode_rcp_data(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_RCP& rcp)
{
    int pos = 0;
    long long   qUTCTime = 0;
    u16 nSpeed = 0, nHeading = 0;
    u8  chExtent = 0;
    double dLat = 0.0, dLon = 0.0, dAlt = 0.0;

    SWM_INFO(CLI_INFO_RCP, "===== roadway corssing pedestrian info message info start\n");

    // qUTCTime	long long	8	-	-	-	time
    pos += get_ldm_field_from_msg(&qUTCTime, pbuf + pos, sizeof(long long));
    qUTCTime = ntohll(qUTCTime);
    // nSpeed	unsigned short	2	0	12	0	m/s
    pos += get_ldm_field_from_msg(&nSpeed, pbuf + pos, sizeof(u16));
    nSpeed = ntohs(nSpeed);
    // nHeading	unsigned short	2	0	359	0	degree
    pos += get_ldm_field_from_msg(&nHeading, pbuf + pos, sizeof(u16));
    nHeading = ntohs(nHeading);
    // chExtent	unsigned char	1	4	5	4	-
    pos += get_ldm_field_from_msg(&chExtent, pbuf + pos, sizeof(u8));
    // ucReserved	unsigned char[3]	3				
    pos += sizeof(u8) * 3;
    // stPoint	WGS84Point	24				
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&dLat, pbuf + pos, sizeof(double));
    dLat = ntohdouble(dLat);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&dLon, pbuf + pos, sizeof(double));
    dLon = ntohdouble(dLon);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&dAlt, pbuf + pos, sizeof(double));
    dAlt = ntohdouble(dAlt);

    rcp.latitude = dLat;
    rcp.longitude = dLon;
    rcp.altitude = dAlt;
    rcp.heading = nHeading;
    rcp.speed = nSpeed;
    rcp.extent = chExtent;

    SWM_INFO(CLI_INFO_RCP, "\tqUTCTime : %lld(%s)\n", qUTCTime, utctime_print(qUTCTime));
    SWM_INFO(CLI_INFO_RCP, "\tnSpeed : %d\n", nSpeed);
    SWM_INFO(CLI_INFO_RCP, "\tnHeading : %d\n", nHeading);
    SWM_INFO(CLI_INFO_RCP, "\tchExtent : %d\n", chExtent);
    SWM_INFO(CLI_INFO_RCP, "\tlatitude : %lf\n", dLat);
    SWM_INFO(CLI_INFO_RCP, "\tlongitude : %lf\n",dLon);
    SWM_INFO(CLI_INFO_RCP, "\taltitude : %lf\n", dAlt);

    SWM_INFO(CLI_INFO_RCP, "===== roadway crossing pedestrian info message info end\n");

    return pos;
}

int decode_emergency_vehicle_alert(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_EVA& eva)
{
    int pos = 0;

    SWM_INFO(CLI_INFO_EVA, "===== emergency vehicle alert info start\n");

    // qUTCTime	long long	8				time
    pos += get_ldm_field_from_msg(&eva.utctime, pbuf + pos, sizeof(s64));
    eva.utctime = ntohll(eva.utctime);
    // chId	char	4				
    pos += get_ldm_field_from_msg(&eva.id, pbuf + pos, sizeof(u8) * 4);
    // nsHeading	unsinged short	2				
    pos += get_ldm_field_from_msg(&eva.heading, pbuf + pos, sizeof(u16));
    eva.heading = ntohs(eva.heading);
    // nsSpeed	unsigned short	2	
    pos += get_ldm_field_from_msg(&eva.speed, pbuf + pos, sizeof(u16));
    eva.speed = ntohs(eva.speed);			
    // chTransmission	unsigned char	1				
    pos += get_ldm_field_from_msg(&eva.transmission, pbuf + pos, sizeof(u8));
    // reserved	unsigned char	1				
    pos += sizeof(u8);
    // nsBasic_type	unsigned short	2				
    pos += get_ldm_field_from_msg(&eva.basic_type, pbuf + pos, sizeof(u16));
    eva.basic_type = ntohs(eva.basic_type);
    // nsResponse_equip	unsigned short	2				
    pos += get_ldm_field_from_msg(&eva.response_equip, pbuf + pos, sizeof(u16));
    eva.response_equip = ntohs(eva.response_equip);
    // nsResponder_type	unsigned short	2				
    pos += get_ldm_field_from_msg(&eva.responder_type, pbuf + pos, sizeof(u16));
    eva.responder_type = ntohs(eva.responder_type);
    // stPoint	WGS84Point	24				
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&eva.latitude, pbuf + pos, sizeof(double));
    eva.latitude = ntohdouble(eva.latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&eva.longitude, pbuf + pos, sizeof(double));
    eva.longitude = ntohdouble(eva.longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&eva.altitude, pbuf + pos, sizeof(double));
    eva.altitude = ntohdouble(eva.altitude);


    SWM_INFO(CLI_INFO_EVA, "\tUTC Time : %lld\n", eva.utctime);
    SWM_INFO(CLI_INFO_EVA, "\tvehicle id : 0x%02X%02X%02X%02X\n", eva.id[0], eva.id[1], eva.id[2], eva.id[3]);
    SWM_INFO(CLI_INFO_EVA, "\tHeading : %d\n", eva.heading);
    SWM_INFO(CLI_INFO_EVA, "\tSpeed : %d\n", eva.speed);
    SWM_INFO(CLI_INFO_EVA, "\tTransmission : %d\n", eva.transmission);
    SWM_INFO(CLI_INFO_EVA, "\tBasic_type : %d\n", eva.basic_type);
    SWM_INFO(CLI_INFO_EVA, "\tResponse_equip : %d\n", eva.response_equip);
    SWM_INFO(CLI_INFO_EVA, "\tResponder_type : %d\n", eva.responder_type);
    SWM_INFO(CLI_INFO_EVA, "\tLatitude : %lf\n", eva.latitude);
    SWM_INFO(CLI_INFO_EVA, "\tLongitude : %lf\n", eva.longitude);
    SWM_INFO(CLI_INFO_EVA, "\tAltitude : %lf\n", eva.altitude);

    SWM_INFO(CLI_INFO_EVA, "===== emergency vehicle alert info end\n");

    return pos;
}

int decode_traffic_signal(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    u32 signal_group_num = 0, intersection_id = 0;

    SWM_INFO(CLI_INFO_SIG, "===== traffic signal info start\n");

    // unIntersectionId	unsigned int	4				
    pos += get_ldm_field_from_msg(&intersection_id, pbuf + pos, sizeof(u32));
    intersection_id = ntohl(intersection_id);
    // unNumOfSignalGroup	unsigned int	4	-	-	-	-
    pos += get_ldm_field_from_msg(&signal_group_num, pbuf + pos, sizeof(u32));
    signal_group_num = ntohl(signal_group_num);

    SWM_INFO(CLI_INFO_SIG, "\tIntersectionId : %d\n", intersection_id);
    SWM_INFO(CLI_INFO_SIG, "\tNumOfSignalGroup : %d\n", signal_group_num);

    for (int i = 0; i < signal_group_num; i++)
    {
        SWM_INFO(CLI_INFO_SIG, "\t----- signal group %d info start\n", i + 1);
        // stSignalGroup	vector<SignalGroup>	-	-	-	-	-
        pos += decode_signal_group_information(hdr, pbuf + pos);
        SWM_INFO(CLI_INFO_SIG, "\t----- signal group %d info end\n", i + 1);
    }

    SWM_INFO(CLI_INFO_SIG, "===== traffic signal info end\n");

    return pos;
}

int decode_signal_group_information(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    u32 signal_group_id = 0, remain_time = 0, num_of_link;
    u8 state = 0;

    // unSignalGroupId	unsigned int	4	-	-	-	-
    pos += get_ldm_field_from_msg(&signal_group_id, pbuf + pos, sizeof(u32));
    signal_group_id = ntohl(signal_group_id);
    // unRemainTime	unsigned int	4	-	-	-	-
    pos += get_ldm_field_from_msg(&remain_time, pbuf + pos, sizeof(u32));
    remain_time = ntohl(remain_time);
    // chState	unsigned char	1	-	-	-	-
    pos += get_ldm_field_from_msg(&state, pbuf + pos, sizeof(u8));
    // chReserved	unsigned char[3]	3	-	-	-	-
    pos += sizeof(u8) * 3;
    // unNumOfLink	unsigned int	4	0	MAX_INT	0	-
    pos += get_ldm_field_from_msg(&num_of_link, pbuf + pos, sizeof(u32));
    num_of_link = ntohl(num_of_link);

    SWM_INFO(CLI_INFO_SIG, "\t\tSignalGroupId : %d\n", signal_group_id);
    SWM_INFO(CLI_INFO_SIG, "\t\tRemainTime : %d\n", remain_time);
    SWM_INFO(CLI_INFO_SIG, "\t\tState : %s(0x%x)\n", get_movement_phase_state(state), state);
    SWM_INFO(CLI_INFO_SIG, "\t\tNumOfLink : %d\n", num_of_link);

    for (int i = 0; i < num_of_link; i++)
    {
        SWM_INFO(CLI_INFO_SIG, "\t\t+++++ %dst link info start\n", i + 1);
        // stLink	vector<LinkInformation>	36	-	-	-	-
        pos += decode_link_information(hdr, pbuf + pos, state, remain_time);
        SWM_INFO(CLI_INFO_SIG, "\t\t+++++ %dst link info end\n", i + 1);
    }

    return pos;
}

int decode_link_information(st_LDMMsgHeader* hdr, u8* pbuf, u8 state, u32 remain_time)
{
    int pos = 0;
    u8 from_link_id[16] = {0,}, to_link_id[16] = {0,}, node_id[16] = {0,};
    double longitude = 0, latitude = 0, altitude = 0;

    // chFromLinkId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&from_link_id, pbuf + pos, sizeof(u8) * 12);
    // chNodeId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&node_id, pbuf + pos, sizeof(u8) * 12);
    // chToLinkId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&to_link_id, pbuf + pos, sizeof(u8) * 12);
    // stPoint	WGS84Point	24	-	-	-	-
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(double));
    latitude = ntohdouble(latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(double));
    longitude = ntohdouble(longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(double));
    altitude = ntohdouble(altitude);

    SWM_INFO(CLI_INFO_SIG, "\t\t\tFromLinkId : %s\n", from_link_id);
    SWM_INFO(CLI_INFO_SIG, "\t\t\tNodeId : %s\n", node_id);
    SWM_INFO(CLI_INFO_SIG, "\t\t\tToLinKId : %s\n", to_link_id);
    SWM_INFO(CLI_INFO_SIG, "\t\t\tstPoint (WGS84Point)\n");
    SWM_INFO(CLI_INFO_SIG, "\t\t\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_SIG, "\t\t\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_SIG, "\t\t\taltitude : %lf\n", altitude);

#if defined(USE_LOCAL_MAP_DATA)
    if (proc_traffic_light_information(to_link_id, state, remain_time) < 0)
    {
        SWM_LOG("%s decode_link_information : fail to process traffic light information.\n", timeval_print(0));

        // remove all st_TR_SIGNAL.
        traffic_signal_list_t& traffic_list = get_sending_traffic_signal_list();

        for (auto &traffic_data : traffic_list)
        {
            delete traffic_data.second;
            traffic_data.second = NULL;
        }

        clear_sending_traffic_signal_list();
    }
#else
    bool is_process = true;

    if (get_a2_linlk_usable())
    {
        if (is_near_link((const char*)from_link_id) == false)
        {
            is_process = false;
        }
    }

    if (is_process)
    {
        if (proc_traffic_light_information(from_link_id, to_link_id, state, remain_time) < 0)
        {
            SWM_LOG("%s decode_link_information : fail to process traffic light information.\n", timeval_print(0));

            // remove all st_TR_SIGNAL.
            traffic_signal_list_t& traffic_list = get_sending_traffic_signal_list();

            for (auto &traffic_data : traffic_list)
            {
                delete traffic_data.second;
                traffic_data.second = NULL;
            }

            clear_sending_traffic_signal_list();
        }
    }
#endif

    return pos;
}

int decode_road_side_alert(st_LDMMsgHeader* hdr, u8* pbuf, swm_ldm::st_RSA& rsa)
{
    int pos = 0;
    s64 utctime = 0;
    double longitude = 0, latitude = 0, altitude = 0;

    //qUTCTime	long long	8	-	-	-	time
    pos += get_ldm_field_from_msg(&utctime, pbuf + pos, sizeof(s64));
    utctime = ntohll(utctime);
    //usEventType	unsigned short	2	-	-	-	-
    pos += get_ldm_field_from_msg(&rsa.itis, pbuf + pos, sizeof(u16));
    rsa.itis = ntohs(rsa.itis);
    //usExtent	unsigned short	2	-	-	0	meter
    pos += get_ldm_field_from_msg(&rsa.extent, pbuf + pos, sizeof(u16));
    rsa.extent = ntohs(rsa.extent);
    //chReserved	unsigned char[4]	4	-	-	-	-
    pos += sizeof(u8) * 4;
    //stPoints	WGS84Point	24	-	-	-	-
    //dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    //pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(s64));
    memcpy(&latitude, pbuf + pos, sizeof(double));
    pos += sizeof(double);
    latitude = ntohdouble(latitude);
    rsa.latitude = (int)(latitude * 10000000);
    //dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    //pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(s64));
    memcpy(&longitude, pbuf + pos, sizeof(double));
    pos += sizeof(double);
    longitude = ntohdouble(longitude);
    rsa.longitude = (int)(longitude * 10000000);
    //dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    //pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(s64));
    memcpy(&altitude, pbuf + pos, sizeof(double));
    pos += sizeof(double);
    altitude = ntohdouble(altitude);

    SWM_INFO(CLI_INFO_RSA, "##### road side alert start\n");
    SWM_INFO(CLI_INFO_RSA, "\tutctime : %lld\n", utctime);
    SWM_INFO(CLI_INFO_RSA, "\titis : %s(%d)\n", get_itis_event(rsa.itis), rsa.itis);
    SWM_INFO(CLI_INFO_RSA, "\textent : %d\n", rsa.extent);
    SWM_INFO(CLI_INFO_RSA, "\tstPoints (WGS84Point)\n");
    SWM_INFO(CLI_INFO_RSA, "\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_RSA, "\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_RSA, "\taltitude : %lf\n", altitude);
    SWM_INFO(CLI_INFO_RSA, "##### road side alert end\n");

    return pos;
}

int decode_link(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    u8 road_rank = 0, road_type = 0;
    u32 road_no = 0, link_type = 0,  max_speed = 0, lane_no = 0;
    u8 link_id[16] = {0,};
    u8 rlink_id[16] = {0,}, llink_id[16] = {0,}, from_node_id[16] = {0,}, to_node_id[16] = {0,}, section_id[16] = {0,}, core_link_id[16] = {0,};
    double nlength = 0;
    u32 direction_path_id = 0, num_of_points = 0;

    SWM_INFO(CLI_INFO_MAP, "===== link info start\n");
    // chLinkId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&link_id, pbuf + pos, sizeof(u8) * 12);
    // chRoadRank	unsigned char	1	1	9	0	-
    pos += get_ldm_field_from_msg(&road_rank, pbuf + pos, sizeof(u8));
    // chRoadType	unsigned char	1	1	5	0	-
    pos += get_ldm_field_from_msg(&road_type, pbuf + pos, sizeof(u8));
    // chReserved	unsigned char[2]	2	-	-	-	-
    pos += sizeof(u8) * 2;
    // nRoadNo	unsigned int	4	-	-	-	-
    pos += get_ldm_field_from_msg(&road_no, pbuf + pos, sizeof(u32));
    road_no = ntohl(road_no);
    // nLinkType	unsigned int	4	1	99	0	-
    pos += get_ldm_field_from_msg(&link_type, pbuf + pos, sizeof(u32));
    link_type = ntohl(link_type);
    // nMaxSpeed	unsigned int	4	-	-	-	km/h
    pos += get_ldm_field_from_msg(&max_speed, pbuf + pos, sizeof(u32));
    max_speed = ntohl(max_speed);
    // nLaneNo	unsigned int	4	-	-	-	-
    pos += get_ldm_field_from_msg(&lane_no, pbuf + pos, sizeof(u32));
    lane_no = ntohl(lane_no);
    // chRLinkId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&rlink_id, pbuf + pos, sizeof(u8) * 12);
    // chLLinkId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&llink_id, pbuf + pos, sizeof(u8) * 12);
    // chFromNodeId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&from_node_id, pbuf + pos, sizeof(u8) * 12);
    // chToNodeId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&to_node_id, pbuf + pos, sizeof(u8) * 12);
    //chCoreLinkId	unsigned char[12]	12	-	-	-	-	N2 Link ID
    pos += get_ldm_field_from_msg(&core_link_id, pbuf + pos, sizeof(u8) * 12);
    // chSectionId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(&section_id, pbuf + pos, sizeof(u8) * 12);
    // dLength	double	8	DBL_MIN	DBL_MAX	0	m
    pos += get_ldm_field_from_msg(&nlength, pbuf + pos, sizeof(double));
    nlength = ntohdouble(nlength);
    // nDirectionPathId	unsigned int	4	-	-	0	-
    pos += get_ldm_field_from_msg(&direction_path_id, pbuf + pos, sizeof(u32));
    direction_path_id = ntohl(direction_path_id);
    // nNumOfPoints	unsigned int	4	-	-	-	-
    pos += get_ldm_field_from_msg(&num_of_points, pbuf + pos, sizeof(u32));
    num_of_points = ntohl(num_of_points);
    // vcPoints	vector<WGS84Point>	24	-	-	-	-

    SWM_INFO(CLI_INFO_MAP, "\tlink_id : %s\n", link_id);
    SWM_INFO(CLI_INFO_MAP, "\troad_lank : 0x%x\n", (int)road_rank);
    SWM_INFO(CLI_INFO_MAP, "\troad_type : 0x%x\n", (int)road_type);
    SWM_INFO(CLI_INFO_MAP, "\troad_no : %d\n", road_no);
    SWM_INFO(CLI_INFO_MAP, "\tlink_type : %d\n", link_type);
    SWM_INFO(CLI_INFO_MAP, "\tmax_speed : %d\n", max_speed);
    SWM_INFO(CLI_INFO_MAP, "\tlane_no : %d\n", lane_no);
    SWM_INFO(CLI_INFO_MAP, "\trlink_id : %s\n", rlink_id);
    SWM_INFO(CLI_INFO_MAP, "\tllink_id : %s\n", llink_id);
    SWM_INFO(CLI_INFO_MAP, "\tfrom_node_id : %s\n", from_node_id);
    SWM_INFO(CLI_INFO_MAP, "\tto_node_id : %s\n", to_node_id);
    SWM_INFO(CLI_INFO_MAP, "\tsection_id : %s\n", section_id);
    SWM_INFO(CLI_INFO_MAP, "\tlength : %lf\n", nlength);
    SWM_INFO(CLI_INFO_MAP, "\tdirection_path_id : %d\n", direction_path_id);
    SWM_INFO(CLI_INFO_MAP, "\tnum_of_points : %u\n", num_of_points);

    double **point_values = NULL;
    point_values = new double*[num_of_points];

    for (int i = 0; i < num_of_points && pos < (hdr->unMsgSize - sizeof(st_LDMMsgHeader)); i++)
    {
        if (link_type == 1)
        {
            point_values[i] = new double[2];
            pos += decode_points_and_get_turn_type(hdr, pbuf + pos, point_values[i]);
        }
        else
        {
            pos += decode_points(hdr, pbuf + pos);
        }
    }

    if (link_type == 1)
    {
        int dir_val = get_heading_azimuth(point_values, num_of_points);
        SWM_INFO(CLI_INFO_MAP, ">>>>> direction : %s\n", TYPE_LINK_STR == dir_val ? "TYPE_LINK_STR" : (TYPE_LINK_LEFT == dir_val ? "TYPE_LINK_LEFT" : (TYPE_LINK_RIGHT == dir_val ? "TYPE_LINK_RIGHT" : "TYPE_LINK_UTURN")));
        for (int i = 0; i < num_of_points && pos < (hdr->unMsgSize - sizeof(st_LDMMsgHeader)); i++)
        {
            delete[] point_values[i];
        }
        delete[] point_values;
        point_values = NULL;
        add_intersection_link_info((const char*)link_id, dir_val, max_speed);
        //test_intersection_link_info();
    }
    else
    {
        add_a2_link_info((const char*)link_id, link_type, lane_no, (const char*)rlink_id, (const char*)llink_id, max_speed);
    }

    SWM_INFO(CLI_INFO_MAP, "===== link info end (size:%d)\n", pos);

    return pos;
}

int decode_points(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    //vcPoints	vector<WGS84Point>	24	-	-	-	-
    double longitude = 0, latitude = 0, altitude = 0;

    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(double));
    latitude = ntohdouble(latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(double));
    longitude = ntohdouble(longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(double));
    altitude = ntohdouble(altitude);

    /*SWM_INFO(CLI_INFO_MAP, "\t\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_MAP, "\t\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_MAP, "\t\taltitude : %lf\n", altitude);*/

    return pos;
}

int decode_points_and_get_turn_type(st_LDMMsgHeader* hdr, u8* pbuf, double *position)
{
    int pos = 0;
    //vcPoints	vector<WGS84Point>	24	-	-	-	-
    double longitude = 0, latitude = 0, altitude = 0;

    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(double));
    latitude = ntohdouble(latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(double));
    longitude = ntohdouble(longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(double));
    altitude = ntohdouble(altitude);

    /*SWM_INFO(CLI_INFO_MAP, "\t\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_MAP, "\t\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_MAP, "\t\taltitude : %lf\n", altitude);*/

    position[0] = latitude;
    position[1] = longitude;

    return pos;
}

int decode_temp_node(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    u8 node_id[16] = {0,};
    u8 node_type = 0;
    double longitude = 0, latitude = 0, altitude = 0;

    // chNodeId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(node_id, pbuf + pos, sizeof(u8) * 12);
    node_id[12] = 0;
    // chNodeType	unsigned char	1	0	99	0	-
    pos += get_ldm_field_from_msg(&node_type, pbuf + pos, sizeof(u8));
    // chReserved	unsigned char[3]	3	-	-	-	-
    pos += sizeof(u8) * 3;
    // stPoint	WGS84Point	24	-	-	-	-
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(double));
    latitude = ntohdouble(latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(double));
    longitude = ntohdouble(longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(double));
    altitude = ntohdouble(altitude);

    SWM_INFO(CLI_INFO_MAP, "===== node info start\n");
    SWM_INFO(CLI_INFO_MAP, "\tchNodeId : %s\n", node_id);
    SWM_INFO(CLI_INFO_MAP, "\tchNodeType : %d\n", node_type);
    SWM_INFO(CLI_INFO_MAP, "\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_MAP, "\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_MAP, "\taltitude : %lf\n", altitude);
    SWM_INFO(CLI_INFO_MAP, "===== node info end\n");

    return pos;
}

int decode_traffic_light(st_LDMMsgHeader* hdr, u8* pbuf)
{
    int pos = 0;
    u8 traffic_light_id[16] = {0,}, link_id[16] = {0,}, post_point_id[16] = {0,};
    u32 traffic_light_type = 0, ref_lane = 0;
    double longitude = 0, latitude = 0, altitude = 0;

    // chTrafficLightId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(traffic_light_id, pbuf + pos, sizeof(u8) * 12);
    // nType	unsigned int	4	0	99	0	-
    pos += get_ldm_field_from_msg(&traffic_light_type, pbuf + pos, sizeof(u32));
    traffic_light_type = ntohl(traffic_light_type);
    // chLinkId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(link_id, pbuf + pos, sizeof(u8) * 12);
    // nRefLane	unsigned int	4	0	UINT_MAX	0	-
    pos += get_ldm_field_from_msg(&ref_lane, pbuf + pos, sizeof(u32));
    ref_lane = ntohl(ref_lane);
    // chPostPointId	unsigned char[12]	12	-	-	-	-
    pos += get_ldm_field_from_msg(post_point_id, pbuf + pos, sizeof(u8) * 12);
    // stPoint	WGS84Point	24	-	-	-	-
    // dLat	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&latitude, pbuf + pos, sizeof(double));
    latitude = ntohdouble(latitude);
    // dLon	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&longitude, pbuf + pos, sizeof(double));
    longitude = ntohdouble(longitude);
    // dAlt	double	8	DBL_MIN	DBL_MAX	0	degree
    pos += get_ldm_field_from_msg(&altitude, pbuf + pos, sizeof(double));
    altitude = ntohdouble(altitude);

    SWM_INFO(CLI_INFO_MAP, "===== traffic light info start\n");
    SWM_INFO(CLI_INFO_MAP, "\tchTrafficLightId : %s\n", traffic_light_id);
    SWM_INFO(CLI_INFO_MAP, "\ttraffic_light_type : %d\n", traffic_light_type);
    SWM_INFO(CLI_INFO_MAP, "\tlink_id : %s\n", link_id);
    SWM_INFO(CLI_INFO_MAP, "\tref_lane : %d\n", ref_lane);
    SWM_INFO(CLI_INFO_MAP, "\tpost_point_id : %s\n", post_point_id);
    SWM_INFO(CLI_INFO_MAP, "\tlatitude : %lf\n", latitude);
    SWM_INFO(CLI_INFO_MAP, "\tlongitude : %lf\n", longitude);
    SWM_INFO(CLI_INFO_MAP, "\taltitude : %lf\n", altitude);

    add_traffic_light_info((const char*)link_id, (const char*)traffic_light_id, traffic_light_type);

    //test_traffic_light_info();

    SWM_INFO(CLI_INFO_MAP, "===== traffic light info end\n");

    return pos;
}

int get_heading_azimuth(double **pos, int pos_num)
{
    int pos1 = get_angle(pos[0][0], pos[0][1], pos[1][0], pos[1][1]);
    int pos2 = get_angle(pos[pos_num - 2][0], pos[pos_num - 2][1], pos[pos_num - 1][0], pos[pos_num - 1][1]);

    int angle = diff_clock_angle(pos2, pos1);

    if (angle >= 345 || angle <= 15)
    {
        // straight
        return TYPE_LINK_STR;
    }
    else if (angle > 15 && angle < 165)
    {
        return TYPE_LINK_RIGHT;
    }
    else if (angle < 345 && angle > 195)
    {
        // left turn
        return TYPE_LINK_LEFT;
    }
    //else if (angle <= 195 && angle >= 165)
    // u turn
    return TYPE_LINK_UTURN;
}

int get_ldm_field_from_msg(void* field, void* msg, int f_size)
{
    memcpy((char*)field, (char*)msg, f_size);
    return f_size;
}

void traffic_signal_process(void)
{
    static u32 count = 0;
    int temp_count = 0;

    count++;

    for (int i = 0; i < 4; i ++)
    {
        swm_ldm::st_TRSIGNAL temp_signal = swm_ldm::st_TRSIGNAL();
        switch (i)
        {
        case 0:
            {
                temp_signal.trafficLightId = std::string("C119AW000021");
                temp_signal.evState = (count % 40) > 20 ? TRAFFIC_LIGHT_COLOR_GREEN : TRAFFIC_LIGHT_COLOR_RED;
                temp_signal.endTime = count % 20;
                break;
            }
        case 1:
            {
                temp_signal.trafficLightId = std::string("C119AW000022");
                temp_signal.evState = (count % 50) > 25 ? TRAFFIC_LIGHT_COLOR_YELLOW : TRAFFIC_LIGHT_COLOR_RED_LEFT;
                temp_signal.endTime = count % 25;
                break;
            }
        case 2:
            {
                temp_signal.trafficLightId = std::string("C119AW000024");
                temp_signal.evState = (temp_count = count % 120) > 90 ? TRAFFIC_LIGHT_COLOR_GREEN_LEFT : 
                                            (temp_count > 60 ? TRAFFIC_LIGHT_COLOR_RED_LEFT :
                                                (temp_count > 30 ? TRAFFIC_LIGHT_COLOR_YELLOW :
                                                    TRAFFIC_LIGHT_COLOR_RED));
                temp_signal.endTime = count % 30;
                break;
            }
        case 3:
            {
                temp_signal.trafficLightId = std::string("C119AW000023");
                temp_signal.evState = (count % 100) > 50 ? TRAFFIC_LIGHT_COLOR_RED : TRAFFIC_LIGHT_COLOR_GREEN;
                temp_signal.endTime = count % 50;
                break;
            }
        default:
            break;
        }
        swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
        if (addSIGtoSWMLDM(ldm_msg, temp_signal) > 0)
        {
            swm_ldm_msg_publish(ldm_msg);
        }
    }
}

void test_road_side_alert(void)
{
    static u32 count = 0;
    int check_time = 0;
    count++;

    check_time = count % 120;
    swm_ldm::st_RSA rsa_data;
    if (check_time < 20)
    {
        rsa_data = swm_ldm::st_RSA();
        rsa_data.itis = ITIS_CODE_OBSTRUCTION_ON_ROADWAY;
        rsa_data.latitude = 372427654;
        rsa_data.longitude = 1267737169;
        swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
        if (addRSAtoSWMLDM(ldm_msg, rsa_data) > 0)
        {
            swm_ldm_msg_publish(ldm_msg);
        }
    }
    else if (check_time < 40)
    {
        rsa_data = swm_ldm::st_RSA();
        rsa_data.itis = ITIS_CODE_PARKING;
        rsa_data.latitude = 372427654;
        rsa_data.longitude = 1267737169;
        swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
        if (addRSAtoSWMLDM(ldm_msg, rsa_data) > 0)
        {
            swm_ldm_msg_publish(ldm_msg);
        }
    }
    else if (check_time < 60)
    {
        rsa_data = swm_ldm::st_RSA();
        rsa_data.itis = ITIS_CODE_PEOPLE_ON_ROADWAY;
        rsa_data.latitude = 372427654;
        rsa_data.longitude = 1267737169;
        swm_ldm::st_SWMLDM ldm_msg = swm_ldm::st_SWMLDM();
        if (addRSAtoSWMLDM(ldm_msg, rsa_data) > 0)
        {
            swm_ldm_msg_publish(ldm_msg);
        }
    }
}