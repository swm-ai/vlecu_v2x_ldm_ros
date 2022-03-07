/*
 * swm_ldm_enc.cpp
 *
 *  Created on: May 18, 2021
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
#include "swm_ldm_enc.h"
#include "swm_log.h"
#include "swm_ldm.h"


int send_cav_data_msg(const swm_ldm::st_SNUDATA::ConstPtr& msg)
{
    int ret = 0;
    u8 buf[1024 * 1024] = {0,};

    memset(buf, 0x00, sizeof(buf));

    if ((ret = make_cav_data_msg(msg, buf)) <= 0)
    {
        SWM_LOG("%s send_cav_data_msg : fail to make ldm bsm message.\n", timeval_print(0));
        return -1;
    }

    if (check_connected_status(CONNECTED_LDM))
    {
        if ((ret = send_ldm_data(buf, ret)) <= 0)
        {
            SWM_LOG("%s send_cav_data_msg : fail to send ldm bsm message.\n", timeval_print(0));
            return -2;
        }
    }
    else
    {
        SWM_LOG("%s send_cav_data_msg : ldm is disconnected.\n", timeval_print(0));
        return -3;
    }

    SWM_INFO(CLI_INFO_VID, "%s send_cav_data_msg : sucess to send ldm connected autonomous vehicle message.\n", timeval_print(0));

    return 0;
}

int make_cav_data_msg(const swm_ldm::st_SNUDATA::ConstPtr& msg, u8* buf)
{
    long long cur_time = (long long)time(NULL);
    int pos = 0;
    u8 u8val = 0;
    u16 u16val = 0;
    u32 u32val = 0;
    s16 s16val = 0;
    s32 s32val = 0;
    st_LDMMsgHeader hdr = {0,};
    
    
    if (buf == NULL)
    {
        SWM_LOG("%s make_cav_data_msg : invalid arg. msg_body = 0x%x.\n", timeval_print(0), buf);
        return -1;
    }

    memset(&hdr, 0x00, sizeof(st_LDMMsgHeader));
    hdr.qTimeStamp = htonll(cur_time);
    hdr.unMsgType = htonl(PROFILE_TYPE_CAV_DAT);

    pos = sizeof(st_LDMMsgHeader);

#if 1
    // chVehicle_id	unsigned char[4]	4		
    memcpy(buf + pos, &msg->id[0], sizeof(u8) * 4);
    pos += sizeof(u8) * 4;
    // nLat	int	4				1/10 micro degree
    s32val = htonl(msg->latitude);
    pos += SET_INT(buf + pos, &s32val);
    // nLon	int	4				1/10 micro degree
    s32val = htonl(msg->longitude);
    pos += SET_INT(buf + pos, &s32val);
    // nElevation	int	4				10cm steps
    s32val = htonl((s32)msg->elevation);
    pos += SET_INT(buf + pos, &s32val);
    // chTransmission	unsigned char	1				enum
    pos += SET_UCHAR(buf + pos, &msg->transmission);
    // nSpeed	int	4				0.02 m/s
    s32val = htonl((s32)msg->speed);
    pos += SET_INT(buf + pos, &s32val);
    // nHeading	int	4				0.0125 degree
    s32val = htonl((s32)msg->heading);
    pos += SET_INT(buf + pos, &s32val);
    // nAngle	int	4				1.5 degree
    s32val = htonl((s32)msg->angle);
    pos += SET_INT(buf + pos, &s32val);
    // nsAccel_lon	short	2				0.01 m/s^2
    s16val = htons(msg->acc_long);
    pos += SET_SHORT(buf + pos, &s16val);
    // nsAccel_lat	short	2				0.01 m/s^2
    s16val = htons(msg->acc_lat);
    pos += SET_SHORT(buf + pos, &s16val);
    // nVert	int	4				0.02 G
    s32val = htonl((s32)msg->acc_vert);
    pos += SET_INT(buf + pos, &s32val);
    // nYaw	int	4				0.01 degree
    s32val = htonl((s32)msg->acc_yaw);
    pos += SET_INT(buf + pos, &s32val);
    // chWheelbreaks	unsigned char	1				bit mask
    pos += SET_UCHAR(buf + pos, &msg->wheelbrakes);
    // chTraction	unsigned char	1				enum
    pos += SET_UCHAR(buf + pos, &msg->traction);
    // chAbs	unsigned char	1				enum
    pos += SET_UCHAR(buf + pos, &msg->abs);
    // chScs	unsigned char	1				enum
    pos += SET_UCHAR(buf + pos, &msg->scs);
    // chBreakBoost	unsigned char	1				enum
    pos += SET_UCHAR(buf + pos, &msg->brakeboost);
    // chAuxBrakes	unsigned char	1				enum
    pos += SET_UCHAR(buf + pos, &msg->auxbrakes);
    // nWidth	int	4				cm 
    s32val = htonl(199);
    pos += SET_INT(buf + pos, &s32val);
    // nLength	int	4				cm 
    s32val = htonl(500);
    pos += SET_INT(buf + pos, &s32val);

    // nCfm_size	int	4				
    s32val = htonl(msg->covoyfoamtion.size());
    pos += SET_INT(buf + pos, &s32val);
    for (auto cfm : msg->covoyfoamtion)
    {
        // nsConvoy_status	unsigned short	2				
        u16val = htons(cfm.convoystatus);
        pos += SET_USHORT(buf + pos, &u16val);
        // chConvoyid	unsigned char	4				
        pos += set_buf_data(buf + pos, cfm.convoyId.c_array(), sizeof(u8) * 4);
        // chConvoy_join_request	unsigned char	1				enum
        pos += SET_UCHAR(buf + pos, &cfm.convoyjoinrequest);

        if (cfm.convoyacceptresponse.size() > 0)
        {
            for (auto accept_response : cfm.convoyacceptresponse)
            {
                // chConvoy_accept_response	unsigned char	1				enum
                pos += SET_UCHAR(buf + pos, &accept_response.response);
                // chConvoy_accept_msg_count	unsigned char	1			
                pos += SET_UCHAR(buf + pos, &accept_response.msgCnt);
                // chVehicleId	unsigned char[4]	4				
                pos += set_buf_data(buf + pos, accept_response.vid.c_array(), sizeof(u8) * 4);
            }
        }
        else
        {
            pos += sizeof(u8) * 6;
        }

        // chConvoy_leave_notice	unsigned char	1				enum
        pos += SET_UCHAR(buf + pos, &cfm.convoyleavenotice);
    }

    // nCsm_size	int	4				
    s32val = htonl(msg->covoystatus.size());
    pos += SET_INT(buf + pos, &s32val);
    for (auto csm : msg->covoystatus)
    {
        // nsConvoy_status	unsigned short	2				enum
        u16val = htons(csm.convoystatus);
        pos += SET_USHORT(buf + pos, &u16val);
        // chConvoy_id	int	4	0	255		
        memcpy(&s32val, csm.convoyid.c_array(), sizeof(u8) * 4);
        s32val = htonl(s32val);
        pos += SET_INT(buf + pos, &s32val);
    }
    
    // nClcm_size	int	4				
    s32val = htonl(msg->convoylanechange.size());
    pos += SET_INT(buf + pos, &s32val);
    for (auto clcm : msg->convoylanechange)
    {
        // nsConvoy_status	unsigned short	2				
        u16val = htons(clcm.convoystatus);
        pos += SET_USHORT(buf + pos, &u16val);
        // chLane_change_request	unsigned char	1				enum
        pos += SET_UCHAR(buf + pos, &clcm.lanechangerequest);
        // chPresent_lane	unsigned char	1	0	12		
        pos += SET_UCHAR(buf + pos, &clcm.presentlane);
        // chChanging_lane	unsigned char	1	0	12
        pos += SET_UCHAR(buf + pos, &clcm.changinglane);		
        // chHeadway	unsigned char	1	0	255		
        pos += SET_UCHAR(buf + pos, &clcm.headway);

        if (clcm.lanechangeresponse.size() > 0)
        {
            for (auto lanechangeresponse : clcm.lanechangeresponse)
            {
                // chLane_change_response	unsigned char	1				enum
                pos += SET_UCHAR(buf + pos, &lanechangeresponse.response);
                // chLane_change_msg_count	unsigned char	1				
                pos += SET_UCHAR(buf + pos, &lanechangeresponse.msgCnt);
                // chLane_change_vehicle_id	char	4		
                pos += set_buf_data(buf + pos, lanechangeresponse.vid.c_array(), sizeof(u8) * 4);		
            }
        }
        else
        {
            pos += sizeof(u8) * 6;
        }

        // chLane_change_notice	unsigned char	1				enum
        pos += SET_UCHAR(buf + pos, &clcm.lanechangenotice);
    }
#else
    //nLat		int	4
    s32val = htonl(msg->latitude);
    pos += SET_INT(buf + pos, &s32val);
    //nLon		int	4
    s32val = htonl(msg->longitude);
    pos += SET_INT(buf + pos, &s32val);
    //nElevation		int	4
    s32val = htonl(msg->elevation);
    pos += SET_INT(buf + pos, &s32val);
    //chTransmission		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->transmission);
    //chReserved		unsinged char[3]	3
    pos += sizeof(u8) * 3;
    //nSpeed		int	4
    s32val = msg->speed;
    s32val = htonl(s32val);
    pos += SET_INT(buf + pos, &s32val);
    //nHeading		int	4
    s32val = msg->heading;
    s32val = htonl(s32val);
    pos += SET_INT(buf + pos, &s32val);
    //nAngle		int	4
    s32val = msg->angle;
    s32val = htonl(s32val);
    pos += SET_INT(buf + pos, &s32val);
    //nsAccel_lon		short	2
    s16val = htons(msg->acc_long);
    pos += SET_SHORT(buf + pos, &s16val);
    //nsAccel_lat		short	2
    s16val = htons(msg->acc_lat);
    pos += SET_SHORT(buf + pos, &s16val);
    //nVert		int	4
    s32val = msg->acc_vert;
    s32val = htonl(s32val);
    pos += SET_INT(buf + pos, &s32val);
    //nYaw		int	4
    s32val = msg->acc_yaw;
    s32val = htonl(s32val);
    pos += SET_INT(buf + pos, &s32val);
    //chWheelbreaks		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->wheelbrakes);
    //chTraction		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->traction);
    //chAbs		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->abs);
    //chScs		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->scs);
    //chBreakBoost		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->brakeboost);
    //chAuxBrakes		unsigned char	1
    pos += SET_UCHAR(buf + pos, &msg->auxbrakes);
    //chReserved		unsinged char[2]	2
    pos += sizeof(u8) * 2;
    //nWidth		int	4
    s32val = htonl(190);    //1900mm
    pos += SET_INT(buf + pos, &s32val);
    //nLength		int	4
    s32val = htonl(499);    //4990mm
    pos += SET_INT(buf + pos, &s32val);

    //nCfm_size		int	4   // currently not supported on LDM
    s32val = 0;
    pos += SET_INT(buf + pos, &s32val);
    //nCsm_size		int	4   // currently not supported on LDM
    pos += SET_INT(buf + pos, &s32val);
    //clcm_size		int	4   // currently not supported on LDM
    pos += SET_INT(buf + pos, &s32val);
#endif
    
    hdr.unMsgSize = htonl(pos);

    memcpy(buf, &hdr, sizeof(st_LDMMsgHeader));

    return pos;
}

int set_buf_data(u8* buf, void* val, size_t size)
{
    memcpy(buf, val, size);
    return size;
}
