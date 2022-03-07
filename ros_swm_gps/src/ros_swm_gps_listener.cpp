#include "ros/ros.h"
#include "ros_swm_gps/MsgGpsSwm.h"

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


//#define ROS_TEST_CODE

// ecu
//#define VL_ECU_IP	"192.168.42.104"
// build pc
#define VL_ECU_IP	"192.168.42.102"

#define SWM_COM_PORT		9101	//9100 // temporary code
#define VL_ECU_SEND_PORT	9200
#define VL_ECU_RECEIVE_PORT	9300

#define BUF_SIZE			1024

int	swmSock = 0;


int openSock()
{
	struct sockaddr_in addr;
	int sock = socket(PF_INET, SOCK_DGRAM, 0);

	if (sock < 0)
	{
		ROS_ERROR("fail to create socket.\r\n");
		return -1;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(SWM_COM_PORT);

	if(bind(sock, (struct sockaddr *) &addr, sizeof(struct sockaddr)) < 0)  {
		ROS_ERROR("socket bind error.\r\n");
		return -1;
	}

	ROS_INFO("success to create socket and to bind.\r\n");

	return sock;
}

void closeSock(int sock)
{
	ROS_INFO("close socket for vl-ecu.\r\n");
	if (close > 0)
		close(sock);
	else
		ROS_INFO("fail to close socket - invalid socket no(%d).\r\n", sock);
}

int reopenSock()
{
	ROS_INFO("reconnect to vl-ecu.\r\n");
	closeSock(swmSock);
	return openSock();
}

int sendSnuMsg(int sock, char* send_buf, int send_size)
{
	struct sockaddr_in swmAddr;

	memset(&swmAddr, 0, sizeof(swmAddr));
	swmAddr.sin_family = AF_INET;
	swmAddr.sin_addr.s_addr = inet_addr(VL_ECU_IP);
	swmAddr.sin_port = htons(VL_ECU_RECEIVE_PORT);

	return sendto(sock, send_buf, send_size, 0, (struct sockaddr*)&swmAddr, sizeof(swmAddr));
}

void msgCallback(const ros_swm_gps::MsgGpsSwm::ConstPtr& msg)
{
#if defined(ROS_TEST_CODE)
	ROS_INFO("received msg [%s]\r\n", msg->data.c_str());
#else
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
		make_cavdat_msg(outbuf, sizeof(outbuf), buf);
		//ROS_INFO("[%s]", outbuf);
		if (sendSnuMsg(swmSock, outbuf, strlen(outbuf)) < 0)
		{
			ROS_ERROR("fail to send snu message.\r\n");
			if ((swmSock = reopenSock()) > 0)
				if (sendSnuMsg(swmSock, outbuf, strlen(outbuf)) < 0)
					ROS_ERROR("fail to resend message.\r\n");
		}
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

#endif
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_swm_gps_listener");
	ros::NodeHandle nh;

#if !defined(ROS_TEST_CODE)
	if ((swmSock = openSock()) < 0)
	{
		ROS_ERROR("fail to open socket for com with vl-ecu.\r\n");
		return -1;
	}
#endif

	ros::Subscriber ros_swm_gps_sub = nh.subscribe("ros_swm_gps_msg", 2000, msgCallback);
	ros::spin();

#if !defined(ROS_TEST_CODE)
	closeSock(swmSock);
#endif

	return 0;
}

