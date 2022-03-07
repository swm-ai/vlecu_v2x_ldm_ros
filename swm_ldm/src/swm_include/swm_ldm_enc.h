/*
 * swm_ldm_enc.h
 *
 *  Created on: May 18, 2021
 *      Author: yhcho
 */

#ifndef SWM_LDM_ENC_H_
#define SWM_LDM_ENC_H_

#include "ros/ros.h"
#include "swm_define.h"
#include "swm_ros.h"
#include "swm_ldm_msg.h"
#include "swm_cav_data.h"
#include "swm_ldm/st_SNUDATA.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SET_CHAR(buf, val) set_buf_data(buf, (u8*)val, sizeof(s8))
#define SET_SHORT(buf, val) set_buf_data(buf, (u8*)val, sizeof(s16))
#define SET_INT(buf, val) set_buf_data(buf, (u8*)val, sizeof(s32))
#define SET_LONG(buf, val)  set_buf_data(buf, (u8*)val, sizeof(s32))
#define SET_LONGLONG(buf, val)  set_buf_data(buf, (u8*)val, sizeof(s64))

#define SET_UCHAR(buf, val) set_buf_data(buf, (u8*)val, sizeof(u8))
#define SET_USHORT(buf, val) set_buf_data(buf, (u8*)val, sizeof(u16))
#define SET_UINT(buf, val) set_buf_data(buf, (u8*)val, sizeof(u32))
#define SET_ULONG(buf, val)  set_buf_data(buf, (u8*)val, sizeof(u32))
#define SET_ULONGLONG(buf, val)  set_buf_data(buf, (u8*)val, sizeof(u64))

extern int send_cav_data_msg(const swm_ldm::st_SNUDATA::ConstPtr& msg);

extern int make_cav_data_msg(const swm_ldm::st_SNUDATA::ConstPtr& msg, u8* buf);

extern int set_buf_data(u8* buf, void* val, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* SWM_LDM_ENC_H_ */
