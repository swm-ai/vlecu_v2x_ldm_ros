/*
 * swm_common.c
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <mqueue.h>
#include <pthread.h>
#include <math.h>
#include <sys/select.h>
#include <signal.h>

#include "swm_common.h"

// gcc -o mutex_lock mutex_lock.c -lpthread
// pthread_mutex_t  msg_mutex = PTHREAD_MUTEX_INITIALIZER;
int opt_debug	= 0;
static pthread_mutex_t mutex_mq;
static st_SWMConfig gSWMConfig;
// work path to work on ros platform. // yhcho // 2021.04.20
static char work_path[1024] = {0,};

#ifdef MQ_USE
void create_mq(char *mqname)
{
	mqd_t msg_fd;
	struct mq_attr attr;

	attr.mq_maxmsg = 10;
	attr.mq_msgsize = sizeof(st_SWMMSg);
//    SWM_LOG("-%s: create_mq", mqname);

#if 0
    if ((msg_fd = mq_open (mqname, O_RDWR | O_CREAT, 0666, &attr)) == -1)
    {
    	SWM_LOG("errno : %x\n", errno);
        SWM_LOG ("mq_open\n");
    }
#else
//    while ((msg_fd = mq_open (mqname, O_RDWR | O_CREAT | O_EXCL, 0666, &attr)) == -1)
	while ((msg_fd = mq_open (mqname, O_RDWR | O_NONBLOCK | O_CREAT | O_EXCL, 0666, &attr)) == -1)
    {
     	remove_mq(mqname);
//    	usleep(10);
    	sleep(1);
    }
#endif

    mq_getattr (msg_fd, &attr);

    // pthread_mutex_lock(&msg_mutex);
	if (mq_close (msg_fd)) {
	   // pthread_mutex_unlock(&msg_mutex);
	   SWM_LOG ("mq_close\n");
	}
  	// pthread_mutex_unlock(&msg_mutex);
    SWM_LOG("%s Message Queue successfully created\n", mqname);
//    exit (EXIT_SUCCESS);
}

void remove_mq(char *mqname)
{
    mqd_t msg_fd;

//    SWM_LOG("-%s: remove_mq", mqname);
/*
    if ((msg_fd = mq_open (mqname, O_RDWR, 0, NULL)) == -1)
        SWM_LOG ("mq_open");

    if (mq_close (msg_fd))
        SWM_LOG ("mq_close");
*/
    // pthread_mutex_lock(&msg_mutex);
    if (mq_unlink (mqname)) {
 	   // pthread_mutex_unlock(&msg_mutex);
 	   SWM_LOG ("mq_unlink\n");
 	}
    // pthread_mutex_unlock(&msg_mutex);
    SWM_LOG("%s Message Queue successfully destroyed\n", mqname);
}

void send_mq(char *mqname, st_SWMMSg *msg)
{
    mqd_t msg_fd;
	char *pbuf;
	int size;

    if( (msg->msg_id<MSG_COMMON_BASE) || (msg->msg_id>SWM_MSG_END) )	{
    	return;
	}

	size = sizeof(st_SWMMSg) + msg->msg_length;
	pbuf = malloc (size);

	memcpy(pbuf, msg, sizeof(st_SWMMSg));
	if(msg->msg_length>0)
		memcpy(pbuf+4, msg->pdata, msg->msg_length);

	pthread_mutex_lock(&mutex_mq);
	if ((msg_fd = mq_open (mqname, O_WRONLY, 0, NULL)) == -1)
	{
	   SWM_LOG ("mq_open\n");
 	}
	if (mq_send (msg_fd, pbuf, size, 0) == -1)
	{
	   SWM_LOG ("mq_send\n");
 	}

    if (mq_close (msg_fd))
	{
	   SWM_LOG ("mq_close\n");
 	}
	pthread_mutex_unlock(&mutex_mq);
	free(pbuf);
}

void receive_mq(char *mqname, st_SWMMSg *msg)
{
	struct timespec tm;
    mqd_t msg_fd;
	char pbuf[MSG_BUF_SIZE];
    int size;

	pthread_mutex_lock(&mutex_mq);
    if ((msg_fd = mq_open (mqname, O_RDONLY, 0, NULL)) == -1)
	{
	   SWM_LOG ("mq_open\n");
 	}

    clock_gettime(CLOCK_REALTIME, &tm);
    tm.tv_nsec += (100 * 1000);

	size = mq_timedreceive( msg_fd, pbuf, MSG_BUF_SIZE, NULL, &tm );
	if(size>0)
	{
 		memcpy(msg, pbuf, sizeof(st_SWMMSg));
		if(msg->msg_length>0)	{
			msg->pdata = &pbuf[4];
		}
 	}

    if (mq_close (msg_fd))
	{
	   SWM_LOG ("mq_close\n");
 	}
	pthread_mutex_unlock(&mutex_mq);
}

void send_msg(char *mqname, u16 mid)
{
	st_SWMMSg msg;

	memset( &msg, 0, sizeof(st_SWMMSg));
	msg.msg_id = mid;
	msg.msg_length = 0;
	msg.pdata = 0;
	send_mq(mqname, &msg);
}
#endif
int getlogtimestr(char *buf)
{
	struct tm *date;
	const time_t t = time(NULL);

	date = localtime(&t);
	sprintf(buf, "%02d%02d_%02d%02d%02d.log", date->tm_mon+1 , date->tm_mday, date->tm_hour , date->tm_min , date->tm_sec);

	return 0;
//	SWM_LOG("[%02d:%02d:%02d]\n" , date->tm_hour , date->tm_min , date->tm_sec);
}

char* timeval_print(struct timeval *ot)
{
	struct timeval dt;
	struct tm *date;
	long dtime=0;
	static char outbuf[KBYTE];

	memset(outbuf, 0, KBYTE);
	if(ot)	{
		dtime = gettimediff(ot, &dt);
		sprintf(outbuf, "interval time [%ld ms]", dtime);
	}
	else	{
		gettimeofday(&dt, NULL);
		date = localtime(&dt.tv_sec);
#if 1
		sprintf(outbuf, "[%04d-%02d-%02d, %02d:%02d:%02d.%03ld]", date->tm_year+1900, date->tm_mon+1, date->tm_mday, date->tm_hour, date->tm_min, date->tm_sec, dt.tv_usec/1000);
#else
		sprintf(outbuf, "[%04d-%02d-%02d, %02d:%02d:%02d][%03ld ms]", date->tm_year+1900, date->tm_mon+1, date->tm_mday, date->tm_hour, date->tm_min, date->tm_sec, dt.tv_usec/1000);
#endif
	}

	return outbuf;
}

char* timestamp_print(u16 val)
{
	long in_time;
	struct tm *date;
	static char outbuf[KBYTE];

	in_time = (long)val * 60;
	date = localtime(&in_time);
	memset(outbuf, 0, KBYTE);
	sprintf(outbuf, "%d(%04d-%02d-%02d/%02d:%02d)", val, date->tm_year+1900, date->tm_mon+1, date->tm_mday, date->tm_hour, date->tm_min);

	return outbuf;
}

char* utctime_print(long long val)
{
	time_t in_time;
	struct tm *date;
	static char outbuf[KBYTE];

	in_time = (time_t)val;
	date = localtime(&in_time);
	memset(outbuf, 0, KBYTE);
	sprintf(outbuf, "%lld(%04d-%02d-%02d/%02d:%02d)", val, date->tm_year+1900, date->tm_mon+1, date->tm_mday, date->tm_hour, date->tm_min);

	return outbuf;
}

void getcurtime(struct timeval *ot)
{
	gettimeofday(ot, NULL);
}

long gettimediff(struct timeval *ot, struct timeval *dt)
{
	long diff_ms=0;
	struct timeval ct;

	getcurtime(&ct);
	dt->tv_sec =  ct.tv_sec - ot->tv_sec;
	if(ct.tv_usec>=ot->tv_usec)
		dt->tv_usec  =  ct.tv_usec  - ot->tv_usec;
	else {
		dt->tv_sec--;
		dt->tv_usec = (1000000 - ot->tv_usec)+ct.tv_usec;
	}
	diff_ms = dt->tv_sec * 1000 + (dt->tv_usec/1000);

	return diff_ms;
}

#if 0
/*
 - old function ( from c-ipts v1 )
 - x : LATITUDE
 - y : LONGITUDE
 - 좌표A(x1,y1)에서 좌료B(x2,y2) 까지의 직선거리 계산
*/
double get_distance(double x1, double y1, double x2, double y2)
{
	int radius = 6371;
	double dLat, dLon;
	double y1r, y2r;
	double a, c, dDistance;

	dLat = dRADIAN( (x2-x1) );
	dLon = dRADIAN( (y2-y1) );

//	log_debug("(x1, y1), (x2, y2) = (%lf, %lf), (%lf, %lf)\n", x1, y1, x2, y2);
//	log_debug("x = %f, y = %f\n", dLon, dLat);
	y1r = dRADIAN( x1 );
	y2r = dRADIAN( x2 );

	a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(y1r) * cos(y2r);
	c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
	dDistance = radius * c;
	dDistance *= 1000;

	return dDistance;
}
#else
/*
 - new function
 - x : LATITUDE
 - y : LONGITUDE
 - 좌표A(x1,y1)에서 좌료B(x2,y2) 까지의 직선거리 계산
*/
double get_distance(double lat1, double lon1, double lat2, double lon2)
{
	double theta, dist;

    if ((lat1==lat2)&&(lon1==lon2)) {

        return 0;
    }

    theta = lon1 - lon2;

    dist = sin(dRADIAN(lat1))*sin(dRADIAN(lat2)) \
        + cos(dRADIAN(lat1))*cos(dRADIAN(lat2))*cos(dRADIAN(theta));

    dist = acos(dist);
    dist = dDEGREE(dist);
    dist = dist*60*1.1515;
// meter
    dist = dist*1.609344*1000;

	return dist;
}
#endif

#if 0
int get_angle(double x1, double y1, double x2, double y2)
{
	int val;

//	double cav_lat1, cav_lon1, cav_lat2, cav_lon2;
	double lat1_rad, lat2_rad, diff_lon;
	double y, x, heading;

	lat1_rad = dRADIAN(x1);
	lat2_rad = dRADIAN(x2);
	diff_lon = dRADIAN(y2 - y1);
	y = sin(diff_lon)*cos(x2);
	x = cos(lat1_rad)*sin(x2) - sin(lat2_rad)*cos(lat1_rad)*cos(diff_lon);
	heading = atan2(y, x);
	val = (int)dDEGREE(heading);

SWM_FUNC("lat1_rad: %f, lat2_rad: %f, diff_lon: %f\n", lat1_rad, lat2_rad, diff_lon);
SWM_FUNC("heading: %f, dAngle: %d\n", heading, val);

	return ((val+360)%360);
}
#elif 0
int get_angle(double x1, double y1, double x2, double y2)
{
	double dLat, dLon;
	double dAngle;

	dLat = x2 - x1;
	dLon = y2 - y1;

	if((dLat==0)&&(dLon==0))	{
		return -1;
	}

	dAngle = dDEGREE(atan2(dLon,dLat));

	if(dAngle<0)	{
		dAngle += 360;
	}

//SWM_FUNC("x1: %f, y1: %f, x2: %f, y2: %f\n", x1, y1, x2, y2);
//SWM_FUNC("dLat: %f, dLon: %f, dAngle: %f\n", dLat, dLon, dAngle);

	return (int)dAngle;
}
#else
/*
 - old function ( from c-ipts v1 )
 - x : LATITUDE
 - y : LONGITUDE
 - 영점을 기준으로 좌표(x,y)의 방위각 계산
 - 정북방향이 0도이며 시계방향으로 회전한다.
 - x는 현재의 위도에서 목표지점까지의 위도 차이 (동쪽으로 증가함)
 - y는 현재의 경도에서 목표지점까지의 경도 차이 (북쪽으로 증가함)
 - 소수점은 버려진다.
*/
double get_degree(double x, double y)
{
	double ang;

	ang = ((atan2(x,y)*180/dPI));

	if(ang < 0) {
		ang = 180 - (180+ang);
	} else {
		ang = 360-ang;
	}

	if(ang == 360) ang = 0;
	ang = ang + 90;

	if(ang >= 360) {
		ang -= 360;
	}

//	SWM_FUNC("x: %f, y: %f, ang: %f\n", x, y, ang);
	return ang;
}
/*
 - old function ( from c-ipts v1 )
 - x : LATITUDE
 - y : LONGITUDE
 - 현재 좌표(x1, y1)에서 목표좌표(x2, y2)까지의 방위각을 구한다.
*/
int get_angle(double x1, double y1, double x2, double y2)
{
	double dLat, dLon, dAngle;

	dLat = x2-x1;
	dLon = y2-y1;

	return (int)get_degree(dLat, dLon);
}
#endif
/*
 - 두 각도의 차를 구한다.
*/
int diff_angle(int a1, int a2)
{
	int dAngle = -1;

	dAngle = a1 - a2;

	if(dAngle<0)	{
		dAngle += 360;
	}
	if(dAngle > 180) {
		dAngle = 360 - dAngle;
	}

	return dAngle;
}

int diff_clock_angle(int a1, int a2)
{
	int dAngle = -1;

	dAngle = a1 - a2;

	if(dAngle<0)	{
		dAngle += 360;
	}

	return dAngle;
}

/*
 - old function ( from c-ipts v1 )
 - angle : 진입각, get_angle 함수로 부터 얻은 차량의 진입각
 - 각도에 따른 교차로의 SignalGroup ID를 추출한다.
 - SignalGroup ID는 3개가 1set이며 반환되는 ID는 각 Set의 시작 번호를 반환한다.
 - cest 기준
*/
int get_signalgroup_entry (int angle)
{
	int sig_gid = -1;

// 0 = 338 ~  23
	if(((angle >= 338) && (angle <= 360)) || ((angle >= 0) && (angle < 23)))	{
		sig_gid = 0;
	}
// 1 = 23 ~  68
	else if((angle >= 23) && (angle < 68)){
		sig_gid = 1;
	}
// 2 = 68 ~  113
	else if((angle >= 68) && (angle < 113)){
		sig_gid = 2;
	}
// 3 = 113 ~  158
	else if((angle >= 113) && (angle < 158)){
		sig_gid = 3;
	}
// 4 = 158 ~  203
	else if((angle >= 158) && (angle < 203)){
		sig_gid = 4;
	}
// 5 = 203 ~  248
	else if((angle >= 203) && (angle < 248)){
		sig_gid = 5;
	}
// 6 = 248 ~  293
	else if((angle >= 248) && (angle < 293)){
		sig_gid = 6;
	}
// 7 = 293 ~  338
	else if((angle >= 293) && (angle < 338)){
		sig_gid = 7;
	}
	else {
		sig_gid = -1;
	}

	return sig_gid;
}

// basic
const int sg_id_list[SIGNALGROUP_MAX] = {1, 13, 4, 16, 7, 19, 10, 22};
// reverse
//const int sg_id_list[SIGNALGROUP_MAX] = {7, 19, 10, 22, 1, 13, 4, 16};

int get_entry_id(int sgid)
{
	if(sgid>7)	return -1;

	return sg_id_list[sgid];
}

int get_heading(double x1, double y1, double x2, double y2)
{
	int val;

//	double cav_lat1, cav_lon1, cav_lat2, cav_lon2;
	double lat1_rad, lat2_rad, diff_lon;
	double y, x, heading;

	lat1_rad = dRADIAN(x1);
	lat2_rad = dRADIAN(x2);
	diff_lon = dRADIAN(y2 - y1);
	y = sin(diff_lon)*cos(x2);
	x = cos(lat1_rad)*sin(x2) - sin(lat2_rad)*cos(lat1_rad)*cos(diff_lon);
	heading = atan2(y, x);
	val = (int)dDEGREE(heading);

	return ((val+360)%360);
}

// 자동차의 위치 정보를 기반으로 자동차의 방향을 구한다.
int get_intersection_angle(double x1, double y1)
{
	double ang;

//	ang = asin((x1*y2 - y1*x2)/(sqrt(x1*x1 + y1+y1)*sqrt(x2*x2 + y2*y2))) * 180/dPI;

	return ang;
}

void set_work_path(const char* path)
{
	char* str_path = NULL;
	if ((str_path = (char*)strchr(path, '=')))
	{
		str_path++;
		memset(work_path, 0x00, sizeof(work_path));
		strncpy(work_path, str_path, strlen(str_path));
	}
}

const char* get_work_path(void)
{
	return (work_path[0] != 0 ? work_path : NULL);
}

// Test func
void set_data_save(u16 set)
{
	gSWMConfig.data_save = set;
}
// Test func
u16 get_data_save(void)
{
	return gSWMConfig.data_save;
}

void set_txdata_save(u16 set)
{
	gSWMConfig.txdata_save = set;
}

u16 get_txdata_save(void)
{
	return gSWMConfig.txdata_save;
}

void set_use_current_position(u16 set)
{
	gSWMConfig.use_current_position = set;
}

u16 get_use_current_position(void)
{
	return gSWMConfig.use_current_position;
}

void set_log_message(u16 set)
{
	gSWMConfig.debug_message = set;

	SWM_LOG("set_log_message: %x\n", gSWMConfig.debug_message);
}

u16 get_log_message(void)
{
	return gSWMConfig.debug_message;
}

u16 check_print_type(u16 flg)
{
	return (gSWMConfig.debug_message & flg);
}

u16 check_message_type(u16 flg)
{
	return BITGET(gSWMConfig.debug_message, flg);
}

u16 set_connected_status(u16 flg)
{
	return BITSET(gSWMConfig.connected_status, flg);
}

u16 clr_connected_status(u16 flg)
{
	return BITCLR(gSWMConfig.connected_status, flg);
}

u16 check_connected_status(u16 flg)
{
	return BITGET(gSWMConfig.connected_status, flg);
}

u16 get_asn_log(void)
{
	return opt_debug;
}

void set_asn_log(int flag)
{
	if(flag)
		SWM_LOG("%s save the log data start\n", timeval_print(0));
	else
		SWM_LOG("%s save the log data stop\n", timeval_print(0));
	opt_debug = flag;
}

void set_config_file(st_SWMConfig *cfg)
{
	pthread_mutex_lock(&mutex_mq);
	if(cfg != NULL)
		memcpy(&gSWMConfig, cfg, sizeof(st_SWMConfig));
	pthread_mutex_unlock(&mutex_mq);
}

void get_config_file(st_SWMConfig *cfg)
{
	pthread_mutex_lock(&mutex_mq);
	if(cfg != NULL)
		memcpy(cfg, &gSWMConfig, sizeof(st_SWMConfig));
	pthread_mutex_unlock(&mutex_mq);
}

u16 init_config_file(void)
{
	SWM_LOG("%s init_config_file", timeval_print(0));
	memset(&gSWMConfig, 0, sizeof(st_SWMConfig));

	if(read_config_file(&gSWMConfig)<0)
	{
		default_config(&gSWMConfig);
		write_config_file(&gSWMConfig);
	}

//	gSWMConfig.debug_message = PRINT_TYPE_CAN;
	print_config(&gSWMConfig);

	return 0;
}


void print_config(st_SWMConfig *pconfig)
{
    if(pconfig==0) return;
	SWM_LOG("> config file info --------------\n");
	SWM_LOG("swm_ip : %s\n", pconfig->swm_ip);
	SWM_LOG("swm_sub_ip : %s\n", pconfig->swm_sub);
	SWM_LOG("swm_port : %d\n", pconfig->swm_port);
	SWM_LOG("ldm_ip : %s\n", pconfig->ldm_ip);
	SWM_LOG("ldm_sub_ip : %s\n", pconfig->ldm_sub);
	SWM_LOG("ldm_port : %d\n", pconfig->ldm_port);
	SWM_LOG("cav_ip : %s\n", pconfig->cav_ip);
	SWM_LOG("cav_sub_ip : %s\n", pconfig->cav_sub);
	SWM_LOG("cav_port : %d\n", pconfig->udp_send);
	SWM_LOG("receive_port : %d\n", pconfig->udp_recv);
	SWM_LOG("debug_message : %d\n", pconfig->debug_message);
	SWM_LOG("log_dump : %d\n", pconfig->log_dump);
	SWM_LOG("---------------------------- <\n");

}

void default_config(st_SWMConfig *pconfig)
{
    if(pconfig==0) return;

	SWM_LOG("default_config\n");
	sprintf(pconfig->swm_ip, "192.168.42.104");
	sprintf(pconfig->swm_sub, "192.168.0.104");
	pconfig->swm_port = 9100;

	sprintf(pconfig->ldm_ip, "192.168.42.157");
	sprintf(pconfig->ldm_sub, "192.168.0.157");
	pconfig->ldm_port = 9000;

	sprintf(pconfig->cav_ip, "192.168.42.110");
	sprintf(pconfig->cav_sub, "192.168.0.110");
	pconfig->udp_send = 9200;
	pconfig->udp_recv = 9300;


	pconfig->data_save = 0;
	pconfig->txdata_save = 0;
	pconfig->debug_message =1;
	pconfig->log_dump =1;
	pconfig->use_current_position = 1;
}

u16 read_config_file(st_SWMConfig *pconfig)
{
	FILE *fcfg;
	char buf[128], *fs1, *fs2;

    if(pconfig==0) return -1;

	SWM_LOG("read_config_file\n");
// yhcho // add to process work path for ros // 2021.04.20
	if (get_work_path() != NULL)
	{
		char full_path[1048] = {0,};
		snprintf(full_path, sizeof(full_path), "%s/%s", get_work_path(), CONFIG_FILE);
		fcfg = fopen(full_path, "r");
	}
	else
		fcfg = fopen(CONFIG_FILE, "r");
	if( fcfg <= 0)
        return -1;

	memset(buf, 0, 128);

	while(fgets(buf, 128, fcfg)!=NULL)
	{
//		printf("%s", buf);

		if(buf[0]=='[' || buf[0]=='#')
			continue;
		if(strncmp(buf, "SWM_IP", 6)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->swm_ip, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "SWM_SUB_IP", 10)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->swm_sub, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "SWM_PORT", 8)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->swm_port = atoi(fs1+1);
			}
		}
		else if(strncmp(buf, "LDM_IP", 6)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->ldm_ip, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "LDM_SUB_IP", 10)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->ldm_sub, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "LDM_PORT", 8)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->ldm_port = atoi(fs1+1);
			}
		}
		else if(strncmp(buf, "CAV_IP", 6)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->cav_ip, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "CAV_SUB_IP", 10)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->cav_sub, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "UDP_SEND_PORT", 13)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->udp_send = atoi(fs1+1);
			}
		}
		else if(strncmp(buf, "UDP_RECV_PORT", 13)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->udp_recv = atoi(fs1+1);
			}
		}
		else if(strncmp(buf, "HDMAP_IP", 8)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->hdmap_ip, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "HDMAP_SUB_IP", 12)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)
				memcpy(pconfig->hdmap_sub, fs1+1, fs2-(fs1+1));
		}
		else if(strncmp(buf, "HDMAP_PORT", 10)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->hdmap_port = atoi(fs1+1);
			}
		}
		else if(strncmp(buf, "DEBUG_MESSAGE", 13)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->debug_message = atoi(fs1+1);
			}
		}
		else if(strncmp(buf, "LOG_DUMP", 8)==0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0)	{
				pconfig->log_dump = atoi(fs1+1);
			}
		}
		else if (strncmp(buf, "SAVE_RSUDATA", 12) == 0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0) {
				pconfig->data_save = atoi(fs1+1);
			}
		}
		else if (strncmp(buf, "SAVE_TXDATA", 11) == 0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if(fs1>0 && fs2>0) {
				pconfig->txdata_save = atoi(fs1+1);
			}
		}
		else if (strncmp(buf, "USE_CURRENT_POSITION", 20) == 0)
		{
			fs1 = strchr(buf, '=');
			fs2 = strchr(buf, '\n');
			if (fs1>0 && fs2>0) {
				pconfig->use_current_position = atoi(fs1+1);
			}
		}
		memset(buf, 0, 128);
	}

	fclose(fcfg);
	return 0;
}

u16 write_config_file(st_SWMConfig *pconfig)
{
	FILE *fcfg;

    if(pconfig==0) return -1;

	SWM_LOG("write_config_file\n");
	// yhcho // add to process work path for ros // 2021.04.20
	if (get_work_path() != NULL)
	{
		char full_path[1048] = {0,};
		snprintf(full_path, sizeof(full_path), "%s/%s", get_work_path(), CONFIG_FILE);
		fcfg = fopen(full_path, "w");
	}
	else
		fcfg = fopen(CONFIG_FILE, "w");
	if( fcfg <= 0)
        return -1;

	fprintf(fcfg, "[SWM]\n");
	fprintf(fcfg, "SWM_IP=%s\n", pconfig->swm_ip);
	fprintf(fcfg, "SWM_SUB_IP=%s\n", pconfig->swm_sub);
	fprintf(fcfg, "SWM_PORT=%d\n", pconfig->swm_port);
	fprintf(fcfg, "\n");
	fprintf(fcfg, "[LDM]\n");
	fprintf(fcfg, "LDM_IP=%s\n", pconfig->ldm_ip);
	fprintf(fcfg, "LDM_SUB_IP=%s\n", pconfig->ldm_sub);
	fprintf(fcfg, "LDM_PORT=%d\n", pconfig->ldm_port);
	fprintf(fcfg, "\n");
	fprintf(fcfg, "[CAV]\n");
	fprintf(fcfg, "CAV_IP=%s\n", pconfig->cav_ip);
	fprintf(fcfg, "CAV_SUB_IP=%s\n", pconfig->cav_sub);
	fprintf(fcfg, "UDP_SEND_PORT=%d\n", pconfig->udp_send);
	fprintf(fcfg, "UDP_RECV_PORT=%d\n", pconfig->udp_recv);
	fprintf(fcfg, "\n");
	fprintf(fcfg, "[HDMAP]\n");
	fprintf(fcfg, "HDMAP_IP=%s\n", pconfig->hdmap_ip);
	fprintf(fcfg, "HDMAP_SUB_IP=%s\n", pconfig->hdmap_sub);
	fprintf(fcfg, "HDMAP_PORT=%d\n", pconfig->hdmap_port);
	fprintf(fcfg, "\n");
	fprintf(fcfg, "[ETC]\n");
	fprintf(fcfg, "DEBUG_MESSAGE=%d\n", pconfig->debug_message);
	fprintf(fcfg, "LOG_DUMP=%d\n", pconfig->log_dump);
	fprintf(fcfg, "SAVE_RSUDATA=%d\n", pconfig->data_save);
	fprintf(fcfg, "SAVE_TXDATA=%d\n", pconfig->txdata_save);
	fprintf(fcfg, "USE_CURRENT_POSITION=%d\n", pconfig->use_current_position);

	fclose(fcfg);

	return 0;
}

double reverse_double(double val)
{
    double result = 0;
    u8 data[8] = {0,}, cvt[8] = {0,};
    memcpy(&data, &val, sizeof(double));
    for (int i = 0; i < sizeof(double); i++)
        cvt[i] = data[sizeof(double) - i - 1];
    memcpy(&result, &cvt, sizeof(double));
    return result;
}