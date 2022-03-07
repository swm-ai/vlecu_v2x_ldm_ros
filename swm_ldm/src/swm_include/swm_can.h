/*
 * swm_can.h
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#ifndef SWM_CAN_H_
#define SWM_CAN_H_

// socket can
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "swm_define.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWM_CAN_C_
#define CAN_EXTERN
#else
#define CAN_EXTERN extern
#endif

#define CANBUS_0		(0U << 29)		// 0
#define CANBUS_1		(1U << 29)		// 0x20000000

typedef u32 CAN_BUS_MSG_ID_PTR;

// cav can msg id
#define MSG_ID_CAV_BASE						   ((CAN_BUS_MSG_ID_PTR) (CANBUS_0 | 0x300))

#define MSG_ID_CAV_COUNT_TIME					(MSG_ID_CAV_BASE | 0x001)
#define MSG_ID_VID_INFO                         (MSG_ID_CAV_BASE | 0x002)
#define MSG_ID_GPS_INFO                         (MSG_ID_CAV_BASE | 0x003)
#define MSG_ID_TRANSMISSION					    (MSG_ID_CAV_BASE | 0x004)
#define MSG_ID_ACCELERATION					    (MSG_ID_CAV_BASE | 0x005)
#define MSG_ID_BRAKES							(MSG_ID_CAV_BASE | 0x006)

#define MSG_ID_MAX                              (MSG_ID_BRAKES + 1 - MSG_ID_CAV_COUNT_TIME)

#define MSG_SIZE_CAV_COUNT_TIME					8
#define MSG_SIZE_VID_INFO						5
#define MSG_SIZE_GPS_INFO						8
#define MSG_SIZE_TRANSMISSION					7
#define MSG_SIZE_ACCELERATION					8
#define MSG_SIZE_BRAKES							6

CAN_EXTERN int can_open(const char *port, int *can_id);
CAN_EXTERN int can_close(int canfd);
CAN_EXTERN int can_send(int canfd, struct can_frame *wframe);
CAN_EXTERN int can_read(int canfd, struct can_frame *rframe);
CAN_EXTERN int cavdata_write(int canfd, int sid, u8 *buf);
CAN_EXTERN int cavdata_read(int canfd, u8 *buf);

#ifdef __cplusplus
}
#endif

#endif /* SWM_CAN_H_ */
