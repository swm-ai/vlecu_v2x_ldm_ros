/*
 * nparse.h
 *
 *  Created on: Mar 4, 2021
 *      Author: yhcho
 */

#ifndef SWM_NPARSE_H_
#define SWM_NPARSE_H_

#ifdef SWM_NPARSE_C_
#define SWM_NPARSE_EXTERN
#else
#define SWM_NPARSE_EXTERN extern
#endif

#include "swm_define.h"
#include "ros_swm_gps/st_SNUDATA.h"


// taker id
#define TALKER_GN_ID	"GN"
#define TALKER_GPS_ID	"GP"

#define	GGA_ID	"GGA"
#define	RMC_ID	"RMC"
#define	HDT_ID	"HDT"

#define GGA_SENT_ID	TALKER_GN_ID GGA_ID
#define RMC_SENT_ID	TALKER_GN_ID RMC_ID
#define HDT_SENT_ID	TALKER_GN_ID HDT_ID

#define	INV_SENT_NUM	0
#define	GGA_SENT_NUM	1
#define	RMC_SENT_NUM	2
#define HDT_SENT_NUM	3

#define	SENT_STM	'$'
#define	SENT_DELIM	','
#define	SENT_ENDM	'*'
#define	SENT_CRLF	"\r\n"

#define	SENT_ID_LEN	5

#define	GGA_FIELD_NUM	14
#define	RMC_FIELD_NUM	13
#define	HDT_FIELD_NUM	2


char** parse_sentence(const char* in_buf, int *type);
int make_cavdat_msg(char* buf, int bufsize, char** data);
int make_snu_cav_msg(ros_swm_gps::st_SNUDATA& msg, char** data);
double make_degree(char* buffer);

#endif /* SWM_NPARSE_H_ */
