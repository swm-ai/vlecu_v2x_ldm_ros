#include "ros/ros.h"
#include "ros_swm_gps/MsgGpsSwm.h"
#include "ros_swm_gps/st_SNUDATA.h"
#include "ros_swm_gps/st_SWMV2X.h"
#include "ros_swm_gps/st_SPAT.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>

#include "swm_define.h"
#include "swm_nparse.h"


#define BUF_SIZE			1024


#define SWMV2X_PAYLOAD_SPAT_FLAG    0x01
#define SWMV2X_PAYLOAD_BSM_FLAG 0x02
#define SWMV2X_PAYLOAD_RSA_FLAG 0x04
#define SWMV2X_PAYLOAD_EVA_FLAG 0x08


ros::Publisher ros_snu_msg_pub;


void msgCallback(const ros_swm_gps::MsgGpsSwm::ConstPtr& msg)
{
	static int msg_count = 0;
	int type = 0, count = 0;
	char outbuf[BUF_SIZE] = {0,};
	if (msg->data.c_str() == NULL)
	{
		ROS_ERROR("--- receive null data]\r\n");
		return;
	}
	else if (strlen(msg->data.c_str()) <= 2)
	{
		ROS_ERROR("--- receive null data\r\n");
		return;
	}
	
	char** buf = parse_sentence(msg->data.c_str(), &type);

	if (type == RMC_SENT_NUM && buf != NULL)
	{
		ros_swm_gps::st_SNUDATA msg;
		make_snu_cav_msg(msg, buf);
		ros_snu_msg_pub.publish(msg);
	}

	switch (type) {
	case GGA_SENT_NUM: count = GGA_FIELD_NUM; break;
	case RMC_SENT_NUM: count = RMC_FIELD_NUM; break;
	case HDT_SENT_NUM: count = HDT_FIELD_NUM; break;
	default: count = 0; break;
	}

	for (int i = 0; i < count; i++){
		if (buf[i])
			free(buf[i]);
	}

	if (count)
		free(buf);
}

void v2xCallback(const ros_swm_gps::st_SWMV2X::ConstPtr& msg)
{
	printf("receive ros_swm_v2x_msg\r\n");

	// if msg have SPaT
	if (msg->payloadMask & SWMV2X_PAYLOAD_SPAT_FLAG)
	{
		printf("SPaT mesage - ");
		if (msg->spatMsg.size() > 0)
		{
			ros_swm_gps::st_SPAT spat_data = msg->spatMsg[0];
			printf("isid:%d, lat:%d, long:%d, sign[%d]\r\n", spat_data.isid, spat_data.latitude, spat_data.longitude, spat_data.sign.size());
		}
	}
	// if msg have BSM
	if (msg->payloadMask & SWMV2X_PAYLOAD_BSM_FLAG)
	{
		printf("BSM mesage - ");
		ros_swm_gps::st_BSM bsm_data = msg->bsmMsg[0];
		//printf("id:%d, lat:%d, long:%d, elev:%d, cavbsm is %s", );
	}
	// if msg have RSA
	if (msg->payloadMask & SWMV2X_PAYLOAD_RSA_FLAG)
	{
		ros_swm_gps::st_RSA rsa_data = msg->rsaMsg[0];
		printf("RSA mesage - ");
		printf("itis:%d, latutidue:%d, longitude:%d, cavrsa regional data count:%d\r\n", rsa_data.itis, rsa_data.latitude, rsa_data.longitude, rsa_data.cavrsa.size());
	}
	// if msg have EVA
	if (msg->payloadMask & SWMV2X_PAYLOAD_EVA_FLAG)
	{
		printf("EVA mesage:");
		//printf("");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_snu_emulator");
	ros::NodeHandle nh;

	printf("##### ROS SNU EMULATOR start #####\r\n");

	// register ros_snu_msg publisher
	ros_snu_msg_pub = nh.advertise<ros_swm_gps::st_SNUDATA>("ros_snu_msg", 2000);

	// register ros_swm_gps_msg subscriber for gps
	ros::Subscriber ros_swm_gps_sub = nh.subscribe("ros_swm_gps_msg", 2000, msgCallback);
	// register ros_swm_v2x_msg subscriber for vl-ecu
	ros::Subscriber ros_swm_v2x_sub = nh.subscribe("ros_swm_v2x_msg", 2000, v2xCallback);
	ros::spin();

	printf("##### ROS SNU EMULATOR stop #####\r\n");

	return 0;
}

