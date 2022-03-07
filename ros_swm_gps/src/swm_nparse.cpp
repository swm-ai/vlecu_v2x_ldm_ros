/*
 * swm_nparse.c
 *
 *  Created on: Mar 4, 2021
 *      Author: yhcho
 */

//#define SWM_NPARSE_C_

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <pthread.h>
#include <signal.h>
#include "swm_define.h"
#include "swm_nparse.h"
#include "ros_swm_gps/st_SNUDATA.h"


char** parse_sentence(const char* in_buf, int *type)
{
	int count = 0, fnum  = 0, len = 0;
	char**	out_buf = NULL;
	char*	pstr = NULL;
	char*	ppt = NULL;

	if (strncmp(in_buf + 1, GGA_SENT_ID, SENT_ID_LEN) == 0)
	{
		*type = GGA_SENT_NUM;
		fnum = 14;
	}
	else if (strncmp(in_buf + 1, RMC_SENT_ID, SENT_ID_LEN) == 0)
	{
		*type = RMC_SENT_NUM;
		fnum = 13;
	}
	else if (strncmp(in_buf + 1, HDT_SENT_ID, SENT_ID_LEN) == 0)
	{
		*type = HDT_SENT_NUM;
		fnum = 2;
	}
	else
	{
		*type = INV_SENT_NUM;
		return NULL;
	}

	out_buf = (char**)malloc(sizeof(char*) * fnum);
	pstr = (char*)in_buf + 2 + SENT_ID_LEN;
	while ((ppt = strchr(pstr, SENT_DELIM)) || (ppt = strchr(pstr, SENT_ENDM)))
	{
		if ((len = ppt - pstr) > 0)
		{
			out_buf[count] = (char*)malloc(len + 1);
			strncpy(out_buf[count], pstr, len);
			out_buf[count][len] = 0;
		} else out_buf[count] = 0;
		pstr = ppt + 1;
		count++;
	}

	return out_buf;
}

// {CAVDAT}[COUN]0[SECD]0[MSEC]0[VEID]SLT0[LATI]375676190[LONG]1268937290[ELEV]208[TRAN]0[SPED]0[HEAD]0[ANGL]0[ACCE](LAT)0(LON)0(VER)0(YAW)0[BRAK](WHE)0(TRA)0(ABS)0(SCS)0(BOO)0(AUX)0
int make_cavdat_msg(char* buf, int bufsize, char** data)
{
	double longi = 0, lati = 0;

	// for MBC GPS
	lati = make_degree(data[2]) * 10000000;
	longi = make_degree(data[4]) * 10000000;

	snprintf(buf, bufsize, "{CAVDAT}[COUN]0[SECD]0[MSEC]0[VEID]SLT0[LATI]%.f[LONG]%.f[ELEV]200[TRAN]0[SPED]0[HEAD]0[ANGL]0[ACCE](LAT)0(LON)0(VER)0(YAW)0[BRAK](WHE)0(TRA)0(ABS)0(SCS)0(BOO)0(AUX)0", lati, longi);

	return 0;
}

int make_snu_cav_msg(ros_swm_gps::st_SNUDATA& msg, char** data)
{
	static int count = 0;
	// use MBC GPS // NMEA-0183
	//msg.count = count>=65535?count=0:count++;
	memcpy((char*)&msg.id, "SLT0", 4);
	msg.latitude = make_degree(data[2]) * 10000000;
	msg.longitude = make_degree(data[4]) * 10000000;
	msg.heading = (data[7] != NULL && strlen(data[7]) != 0) ? atof(data[7]) * 10000 /125 : 0;
	msg.speed = (data[6] != NULL && strlen(data[6]) != 0) ? ((atof(data[6]) * 1852) * 50) / 3600 : 0;
	msg.elevation = 200;

	return 0;
}

double make_degree(char* buffer)
{
	double ddmm = 0, dd = 0;

	ddmm = atof(buffer);
	dd = (ddmm - ((int)(ddmm/100))*100 ) / 60;
	dd += (int)(ddmm/100);

	return dd;
}
