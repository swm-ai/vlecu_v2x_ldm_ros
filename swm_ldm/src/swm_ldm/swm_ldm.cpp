/*
 * swm_ldm.cpp
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
#include "swm_ldm.h"
#include "swm_ldm_dec.h"

#define SAVE_DATA_SIZE (10*MBYTE)
static u8 ldm_run = 1;
static u8 pbuf[LDM_RECV_SIZE];
static st_LDMSaveData 	gtSaveInfo;
static st_LDMSaveData   gtTxSaveInfo;
static st_LDMStatus		gLdmStatus;
static pthread_mutex_t 	mutex_ldm;

void *ldm_proc(void *args)
{
    static int retval=0xf1;

    fd_set tmpfds;
    struct timeval ldm_tv;
#if defined(USE_SIGNAL_RSA_TEST)
    struct timeval test_tv, df_tv;  // for test
#endif  // USE_SIGNAL_RSA_TEST
    int ldm_cnt = 0, try_cnt = 0, err_val = 0;
    st_LDMMsgHeader msg_hdr = {0,};

    memset(&gLdmStatus, 0, sizeof(st_LDMStatus));
    pthread_mutex_init(&mutex_ldm, NULL);

    // received data init // such as ldm signal
    SWM_LOG("%s ###### Start LDM Proc ######\n", timeval_print(0));
    while (connect_ldm(&gLdmStatus) < 0)
    {
        usleep(1000000);
        SWM_LOG("%s connect retry %d\n", timeval_print(0), try_cnt++);
    }

    ldm_tv.tv_sec = 0;
    ldm_tv.tv_usec = (10 * 1000);

#if defined(USE_SIGNAL_RSA_TEST)
    getcurtime(&test_tv);   // for test
#endif  // USE_SIGNAL_RSA_TEST

    while (ldm_run)
    {
        tmpfds = gLdmStatus.ldmfds;
        ldm_cnt = select(gLdmStatus.sockfd + 1, &tmpfds, NULL, NULL, &ldm_tv);
        if (ldm_cnt < 0)
        {
            SWM_LOG("[s] ldm_proc - select error\n", timeval_print(0));

            if (reconnect_ldm(&gLdmStatus, 0) < 0) {
                ldm_run = 0;
                break;
            }
        }
        else if (ldm_cnt == 0) {
            usleep(10);
        }
        else
        {
            if ((err_val = recv_ldm_message(gLdmStatus.sockfd, &msg_hdr, pbuf, LDM_RECV_SIZE)) <= 0) {
                SWM_LOG("%s ldm_proc - recv_ldm_message error(%d:%s)\n", timeval_print(0), errno, strerror(errno));
                if (reconnect_ldm(&gLdmStatus, 0) < 0) {
                    ldm_run = 0;
                    break;
                }
            }
            else if (err_val > 0)
            {
                SWM_LOG("%s ldm_proc - received message ( %s message, %d bytes)\n", timeval_print(0), msg_hdr.unMsgType > 13 ? "V2X" : "MAP", msg_hdr.unMsgSize);
                ldm_message_decoder(&msg_hdr, pbuf);
            }
            else
            {
                SWM_LOG("%s ldm_proc - recv_ldm_message returns value 0.\n", timeval_print(0));
            }
        }

        // for test start
#if defined(USE_SIGNAL_RSA_TEST)
        if (gettimediff(&test_tv, &df_tv) > 1000)
        {
            getcurtime(&test_tv);
            test_traffic_signal_process();
            test_road_side_alert();
        }
#endif  // USE_SIGNAL_RSA_TEST
        // for test end
    }

    if (get_data_save()) {
        set_data_save(LDM_SAVE_NONE);
        data_save_stop();
    }

    if (get_txdata_save()) {
        set_txdata_save(LDM_SAVE_NONE);
        txdata_save_stop();
    }

    // remove data // if received or sending datas are exist.
    SWM_LOG("%s ###### Stop LDM Proc ######\n", timeval_print(0));
    
    disconnect_ldm(&gLdmStatus);
    pthread_exit((void*)&retval);
}

int connect_ldm(st_LDMStatus *pldm)
{
	st_SWMConfig gcfg;
	struct sockaddr_in ldmAddr;
	socklen_t ldm_ssize;

	if(check_connected_status(CONNECTED_LDM))	{
		clr_connected_status(CONNECTED_LDM);
	}

	get_config_file(&gcfg);

	pldm->sockfd = socket(PF_INET, SOCK_STREAM, 0);
	if(pldm->sockfd==-1)	{
    	SWM_LOG("%s connect_ldm - socket error(%d:%s)\n", timeval_print(0), errno, strerror(errno));
		return -1;
	}
	ldmAddr.sin_family = AF_INET;
	if(strncmp(gcfg.ldm_ip, "local_ip", 8)==0)	{
	    SWM_LOG("%s connect_ldm - local ip (port:15210)\n", timeval_print(0));
		ldmAddr.sin_port = htons(15210);
		ldmAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	else	{
		SWM_LOG("%s connect_ldm - %s:%d\n", timeval_print(0), gcfg.ldm_ip, gcfg.ldm_port);
		ldmAddr.sin_port = htons(gcfg.ldm_port);
		ldmAddr.sin_addr.s_addr = inet_addr(gcfg.ldm_ip);
	}

	memset(ldmAddr.sin_zero, '\0', sizeof ldmAddr.sin_zero);
	ldm_ssize = sizeof(ldmAddr);
	if( connect(pldm->sockfd, (struct sockaddr *) &ldmAddr, ldm_ssize) == -1)	{
  		SWM_LOG("%s connect_ldm - connect error: %s, %d (%d:%s)\n", timeval_print(0), gcfg.ldm_ip, gcfg.ldm_port, errno, strerror(errno));
  		close(pldm->sockfd);
		return -2;
 	}

	set_connected_status(CONNECTED_LDM);

	FD_ZERO(&pldm->ldmfds);
	FD_SET(pldm->sockfd, &pldm->ldmfds);

	return pldm->sockfd;
}

int reconnect_ldm(st_LDMStatus *pldm, int try_cnt)
{
    int count = try_cnt;
	disconnect_ldm(pldm);

	if(try_cnt==0) count=1;
	else count = try_cnt;

	while(count)	{
		if(connect_ldm(pldm)>0)	return 0;
		usleep(1000000);
		if(try_cnt>0) --count;
		SWM_LOG("%s reconnect_ldm - connection retry - %d\n", timeval_print(0), count);
	}
	return -1;
}

int disconnect_ldm(st_LDMStatus *pldm)
{
    if(check_connected_status(CONNECTED_LDM))	{
		close(pldm->sockfd);
		pldm->sockfd = 0;
		clr_connected_status(CONNECTED_LDM);
	}
	return 0;
}

int recv_ldm_message(int nsocket, st_LDMMsgHeader* hdr, u8* msg_body, int buf_size)
{
    int ret = 0, msg_body_size = 0, recv_size = 0;
    long long tstamp = 0;

#if defined(USE_DETAIL_SOCKET_LOG)
    SWM_LOG("%s recv_ldm_message -- start\n", timeval_print(0));
#endif  // USE_DETAIL_SOCKET_LOG

    // recv header
    while (recv_size < sizeof(st_LDMMsgHeader))
    {
        if ((ret = recv(nsocket, (void*)(msg_body + recv_size), sizeof(st_LDMMsgHeader) - recv_size, 0)) <= 0)
        {
            SWM_LOG("%s recv_ldm_message -- error(%d:%s)\n", timeval_print(0), errno, strerror(errno));
            return ret;
        }
        recv_size += ret;
#if defined(USE_DETAIL_SOCKET_LOG)
        SWM_LOG("%s recv_ldm_message -- recv header (%d)\n", timeval_print(0), recv_size);
#endif  // USE_DETAIL_SOCKET_LOG
    }

    data_save_func(msg_body, sizeof(st_LDMMsgHeader));
    decode_header(msg_body, hdr);

    recv_size = 0;
    msg_body_size = hdr->unMsgSize - sizeof(st_LDMMsgHeader);
#if defined(USE_DETAIL_SOCKET_LOG)
    SWM_LOG("%s recv_ldm_message -- type (%d) body size (%d)\n", timeval_print(0), hdr->unMsgType, msg_body_size);
#endif  // USE_DETAIL_SOCKET_LOG
    if (msg_body_size < 0)
    {
        SWM_LOG("%s recv_ldm_message -- message size (%d) error(%d:%s)\n", timeval_print(0), msg_body_size, errno, strerror(errno));
    }

    // recv body
    while (recv_size < msg_body_size)
    {
        if ((ret = recv(nsocket, (void*)(msg_body + recv_size),  msg_body_size - recv_size, 0)) <= 0)
        {
            SWM_LOG("%s recv_ldm_message -- error(%d:%s)\n", timeval_print(0), errno, strerror(errno));
            return ret;
        }
        recv_size += ret;
#if defined(USE_DETAIL_SOCKET_LOG)
        SWM_LOG("%s recv_ldm_message -- recv body (%d)\n", timeval_print(0), recv_size);
#endif  // USE_DETAIL_SOCKET_LOG
    }
    data_save_func(msg_body, msg_body_size);
#if defined(USE_DETAIL_SOCKET_LOG)
    SWM_LOG("%s recv_ldm_message (recv body:%d)-- end\n", timeval_print(0), recv_size);
#endif  // USE_DETAIL_SOCKET_LOG

    return hdr->unMsgSize;
}

int send_ldm_data(u8* pbuf, int buf_size)
{
    int ret = 0, sent_size = 0;

    while (sent_size < buf_size)
    {
        if ((ret = send(gLdmStatus.sockfd, pbuf + sent_size, buf_size - sent_size, 0)) <= 0)
        {
            SWM_LOG("%s send_ldm_data -- error(%d:%s)\n", timeval_print(0), errno, strerror(errno));
            return ret;
        }
        sent_size += ret;
    }

    txdata_save_func(pbuf, buf_size);

    return buf_size;
}

u8 data_save_start(void)
{
	char filename[24] = {0, };
	char filename_full_path[1048] = {0,};
//	if(gtSaveInfo.pF>0)
//		fclose(gtSaveInfo.pF);

	sprintf(filename, "%s", "LDMDATA_0000_000000.dat");
	getlogtimestr(filename+8);
	SWM_LOG("%s save start: %s\n", timeval_print(0), filename);
	memset(&gtSaveInfo, 0, sizeof(st_LDMSaveData));

	if (get_work_path() != NULL)
	{
		snprintf(filename_full_path, sizeof(filename_full_path), "%s/%s", get_work_path(), filename);
		gtSaveInfo.pF = fopen(filename_full_path, "w");
	}
	else
		gtSaveInfo.pF = fopen(filename, "w");
	if(gtSaveInfo.pF<=0)
		gtSaveInfo.isopen = 0;
	else
		gtSaveInfo.isopen = 1;
	return gtSaveInfo.isopen;
}

u8 data_save_stop(void)
{
    gtSaveInfo.pF = 0;
    SWM_LOG("%s save stop\n", timeval_print(0));
    gtSaveInfo.isopen = 0;
    gtSaveInfo.count = 0;
    return gtSaveInfo.isopen;
}

int data_save_func(u8 *pbuf, int size)
{
    u8 svflg = get_data_save();
    //u32 wsize = 0;

	if(svflg)
    {
		if(!gtSaveInfo.isopen)
        {
  			if (!data_save_start())
            {
	  			return -1;
            }
  		}

		switch (svflg)
        {
		case LDM_SAVE_DATA_ONLY:
		case LDM_SAVE_DATA_ALL:
		case LDM_SAVE_DATA_COUNT:
        {
			fwrite(pbuf, size, 1, gtSaveInfo.pF);
            fflush(gtSaveInfo.pF);
			//wsize = ftell(gtSaveInfo.pF);
		   	/*if( wsize > SAVE_DATA_SIZE )
            {
		   		data_save_stop();
 //  			set_data_save(LDM_SAVE_NONE);
 		  	}*/
			break;
        }
		default:
		case LDM_SAVE_NONE:
			break;
		}
  	}

    return 0;
}

u8 txdata_save_start(void)
{
	char filename[24] = {0, };
	char filename_full_path[1048] = {0,};
//	if(gtTxSaveInfo.pF>0)
//		fclose(gtTxSaveInfo.pF);

	sprintf(filename, "%s", "LDMTXDATA_0000_000000.dat");
	getlogtimestr(filename+10);
	SWM_LOG("%s save start: %s\n", timeval_print(0), filename);
	memset(&gtTxSaveInfo, 0, sizeof(st_LDMSaveData));

	if (get_work_path() != NULL)
	{
		snprintf(filename_full_path, sizeof(filename_full_path), "%s/%s", get_work_path(), filename);
		gtTxSaveInfo.pF = fopen(filename_full_path, "w");
	}
	else
		gtTxSaveInfo.pF = fopen(filename, "w");
	if(gtTxSaveInfo.pF<=0)
		gtTxSaveInfo.isopen = 0;
	else
		gtTxSaveInfo.isopen = 1;
	return gtTxSaveInfo.isopen;
}

u8 txdata_save_stop(void)
{
    gtTxSaveInfo.pF = 0;
    SWM_LOG("%s save stop\n", timeval_print(0));
    gtTxSaveInfo.isopen = 0;
    gtTxSaveInfo.count = 0;
    return gtTxSaveInfo.isopen;
}

int txdata_save_func(u8 *pbuf, int size)
{
    u8 svflg = get_txdata_save();
    //u32 wsize = 0;

	if(svflg)
    {
		if(!gtTxSaveInfo.isopen)
        {
  			if (!txdata_save_start())
            {
	  			return -1;
            }
  		}

		switch (svflg)
        {
		case LDM_SAVE_DATA_ONLY:
		case LDM_SAVE_DATA_ALL:
		case LDM_SAVE_DATA_COUNT:
        {
			fwrite(pbuf, size, 1, gtTxSaveInfo.pF);
            fflush(gtTxSaveInfo.pF);
			//wsize = ftell(gtTxSaveInfo.pF);
		   	/*if( wsize > SAVE_DATA_SIZE )
            {
		   		txdata_save_stop();
 //  			set_txdata_save(LDM_SAVE_NONE);
 		  	}*/
			break;
        }
		default:
		case LDM_SAVE_NONE:
			break;
		}
  	}

    return 0;
}