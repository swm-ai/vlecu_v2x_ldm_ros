/*
 * swm_ldm.h
 *
 *  Created on: May 17, 2021
 *      Author: yhcho
 */

#ifndef SWM_LDM_H_
#define SWM_LDM_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <mqueue.h>
#include "swm_define.h"
#include "swm_ldm_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LDM_SEND_SIZE (1024*1024)
#define LDM_RECV_SIZE (1024*1024)
#define LDM_HEADER_SIZE sizeof(st_TR_SIGNAL)

typedef struct _LDM_SAVEDATA {
	FILE *pF;
	u8 isopen;
	u8 count;
}st_LDMSaveData;

typedef struct _LDM_STATUS {
	int	sockfd;
	fd_set ldmfds;
	char mac_addr[6];
} st_LDMStatus;

extern int connect_ldm(st_LDMStatus *pldm);
extern int reconnect_ldm(st_LDMStatus *pldm, int try_cnt);
extern int disconnect_ldm(st_LDMStatus *pldm);
extern int recv_ldm_message(int nsocket, st_LDMMsgHeader* hdr, u8* msg_body, int buf_size);
extern int send_ldm_data(u8* pbuf, int buf_size);
extern u8 data_save_start(void);
extern u8 data_save_stop(void);
extern int data_save_func(u8 *pbuf, int size);
extern u8 txdata_save_start(void);
extern u8 txdata_save_stop(void);
extern int txdata_save_func(u8 *pbuf, int size);

extern void *ldm_proc(void *args);

#ifdef __cplusplus
}
#endif

#endif /* SWM_LDM_H_ */
