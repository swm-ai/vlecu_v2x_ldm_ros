/*
 * swm_common.h
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#ifndef SWM_COMMON_H_
#define SWM_COMMON_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <mqueue.h>
#include "swm_define.h"
#include "swm_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWM_COMMON_C_
#define COMMON_EXTERN
#else
#define COMMON_EXTERN extern
#endif

#define CONFIG_FILE		"swm.config"

#ifdef MQ_USE
#define	MQ_COMM_NAME	"/cipts_mq_comm"
#define	MQ_OBU_NAME		"/cipts_mq_obu"
#define	MQ_HDMAP_NAME	"/cipts_mq_hdmap"
#define	MQ_CAV_NAME		"/cipts_mq_cav"

#define MSG_COMMON_BASE	0x0100
#define MSG_OBU_BASE	0x0200
#define MSG_HDMAP_BASE	0x0300
#define MSG_CAV_BASE	0x0500

typedef enum
{
	MSG_COMMON_INIT = MSG_COMMON_BASE +1,
	MSG_COMMON_CLOSE,
	MSG_COMMON_TEST0,
	MSG_COMMON_TEST1,
	MSG_COMMON_TEST2,

	MSG_OBU_INIT = MSG_OBU_BASE+1,
	MSG_OBU_CLOSE,
	MSG_OBU_TEST0,
	MSG_OBU_TEST1,
	MSG_OBU_TEST2,

	MSG_HDMAP_INIT = MSG_HDMAP_BASE+1,
	MSG_HDMAP_CLOSE,
	MSG_HDMAP_GPS,
	MSG_HDMAP_NDS,
	MSG_HDMAP_TEST,

	MSG_CAV_INIT = MSG_CAV_BASE+1,
	MSG_CAV_CLOSE,
	MSG_CAV_TEST0,
	MSG_CAV_TEST1,
	MSG_CAV_TEST2,

	SWM_MSG_END
}_SWM_MSG_ID;
#endif

#ifdef SHM_USE
#define SHM_LOG_KEY		9800
#define SHM_INFO_KEY	9810
#define SHM_GPS_KEY		9820
#define SHM_HDMAP_KEY	9830
#define SHM_COMMAND_KEY	9840
#endif

#define dPI			(3.1415926535)
#define dRADIAN(x)	(x*(dPI/180.0f))
#define dDEGREE(x)	(x*(180.0f/dPI))
// V2X Latitude, Longitude 값을 double 형으로 변경
#define dINT2DOU(x) ((double)x/10000000)
#define dDOU2INT(x) (int)(x*10000000)
// V2X heading 정보를 degree 형태로 변경
#define dHED2DEG(x) (x/80)

typedef enum {
  LDM_MESSAGE_SIG = 0,
  LDM_MESSAGE_BSM,
  LDM_MESSAGE_PVD,
  LDM_MESSAGE_RSA,
  LDM_MESSAGE_TIM,
  LDM_MESSAGE_ALL
}_LDM_MESSAGE_TYPE;

#define PRINT_TYPE_CAN				(1<<8)
#define PRINT_TYPE_LSM				(1<<9)
#define PRINT_TYPE_HDMAP			(1<<10)

//#define PrintType_CAN					0
//#define PrintType_LDM					1

//#define CheckPrintType(a,b)             ((a&0xff00)&b)
//#define CheckMessageType(a,b)           ((a&0x00ff)&(1<<b))
#define SIGNALGROUP_MAX			8

typedef enum
{
	CONNECTED_NONE 		= 0x00,
	CONNECTED_LDM,
	CONNECTED_HDMAP,
	CONNECTED_CAV,

	CONNECTED_END
}_CONNECTED_STATUS;

typedef enum
{
	LDM_SAVE_NONE 		= 0x00,
	LDM_SAVE_DATA_ONLY,
	LDM_SAVE_DATA_ALL,
	LDM_SAVE_DATA_COUNT,

	LDM_SAVE_END
}_LDM_SAVE_TYPE;

//#define MQ_BUFSIZE	(1024*100)
//#define DEATH(msg) { perror(msg); exit(errno); }

typedef struct _SWM_MSG {
	u16 msg_id;
	u16 msg_length;
	char *pdata;
}st_SWMMSg;

typedef struct _SWM_CONFIG {
// swm config
	char swm_ip[16];
	char swm_sub[16];
	u16  swm_port;

// ldm info
	char ldm_ip[16];
	char ldm_sub[16];
	u16  ldm_port;

// cav info
	char cav_ip[16];
	char cav_sub[16];
	u16  udp_send;
	u16  udp_recv;

// hdmap info
	char hdmap_ip[16];
	char hdmap_sub[16];
	u16  hdmap_port;

	char bsm_id[5];
//	char vehicle_id[20];
	wchar_t vehicle_id[20];
// etc
	u16 data_save;
	u16 txdata_save;
	u16 use_current_position;
// 0, 1:spat, 2:map
	u16 debug_message;
	u16 log_dump;
	u16 connected_status;
}st_SWMConfig;

#ifdef MQ_USE
COMMON_EXTERN void create_mq(char *mqname);
COMMON_EXTERN void remove_mq(char *mqname);
COMMON_EXTERN void send_mq(char *mqname, st_SWMMSg *msg);
COMMON_EXTERN void receive_mq(char *mqname, st_SWMMSg *msg);
COMMON_EXTERN void send_msg(char *mqname, u16 mid);
#endif
COMMON_EXTERN int getlogtimestr(char *buf);
COMMON_EXTERN char* timeval_print(struct timeval *ot);
COMMON_EXTERN char* timestamp_print(u16 val);
COMMON_EXTERN char* utctime_print(long long val);
COMMON_EXTERN void getcurtime(struct timeval *ot);
COMMON_EXTERN long gettimediff(struct timeval *ot, struct timeval *dt);

COMMON_EXTERN double get_distance(double x1, double y1, double x2, double y2);
COMMON_EXTERN double get_degree(double x, double y);
COMMON_EXTERN int get_angle(double x1, double y1, double x2, double y2);
COMMON_EXTERN int diff_angle(int a1, int a2);
COMMON_EXTERN int diff_clock_angle(int a1, int a2);
COMMON_EXTERN int get_signalgroup_entry(int angle);
COMMON_EXTERN int get_entry_id(int sgid);
COMMON_EXTERN int get_heading(double x1, double y1, double x2, double y2);
COMMON_EXTERN int get_intersection_angle(double x1, double y1);

COMMON_EXTERN void set_work_path(const char* path);
COMMON_EXTERN const char* get_work_path(void);
COMMON_EXTERN void set_data_save(u16 set);
COMMON_EXTERN u16 get_data_save(void);
COMMON_EXTERN void set_txdata_save(u16 set);
COMMON_EXTERN u16 get_txdata_save(void);
COMMON_EXTERN void set_use_current_position(u16 set);
COMMON_EXTERN u16 get_use_current_position(void);
COMMON_EXTERN void set_log_message(u16 set);
COMMON_EXTERN u16 get_log_message(void);
COMMON_EXTERN u16 check_print_type(u16 flg);
COMMON_EXTERN u16 check_message_type(u16 flg);
COMMON_EXTERN u16 set_connected_status(u16 flg);
COMMON_EXTERN u16 clr_connected_status(u16 flg);
COMMON_EXTERN u16 check_connected_status(u16 flg);
COMMON_EXTERN u16 get_asn_log(void);
COMMON_EXTERN void set_asn_log(int flag);

COMMON_EXTERN void set_config_file(st_SWMConfig *cfg);
COMMON_EXTERN void get_config_file(st_SWMConfig *cfg);
COMMON_EXTERN u16 init_config_file(void);
COMMON_EXTERN void print_config(st_SWMConfig *pconfig);
COMMON_EXTERN void default_config(st_SWMConfig *pconfig);
COMMON_EXTERN u16 read_config_file(st_SWMConfig *pconfig);
COMMON_EXTERN u16 write_config_file(st_SWMConfig *pconfig);
COMMON_EXTERN double reverse_double(double val);

#ifdef __cplusplus
}
#endif

#endif /* SWM_COMMON_H_ */
