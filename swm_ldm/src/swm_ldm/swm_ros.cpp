/*
 * swm_ros.cpp
 *
 *  Created on: Mar 27, 2021
 *      Author: yhcho
 */

#include "ros/ros.h"
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <fcntl.h>

#include "swm_ros.h"
#include "swm_common.h"
#include "swm_can.h"
#include "swm_shm.h"
#include "swm_ldm_enc.h"
#include "swm_ldm/st_SNUDATA.h"
#include "swm_ldm/st_SWMLDM.h"



static ros::Publisher ros_swm_ldm_pub;
static ros::NodeHandle *ros_swm_ldm_nh = NULL;


void cav_data_callback(const swm_ldm::st_SNUDATA::ConstPtr& msg)
{
	int cv_data_count = 0;

	SWM_INFO(CLI_INFO_CAV, "%s cav_data_callback: receive CAVDATA - id(0x%02X%02X%02X%02X), lat(%d), lon(%d)\r\n", 
							timeval_print(0),
							msg->id[0], msg->id[1], msg->id[2], msg->id[3],
							msg->latitude, msg->longitude);

#if 0
	// if CFM data exist
	if ((cv_data_count = msg->covoyfoamtion.size()) > 0)
	{
		//SWM_LOG("$$$$$ CFM is exist.\r\n");
		if (cv_data_count > 1)
			SWM_LOG("cav_data_callback: CFM count is %d. use only element[0].", cv_data_count);
		if ((cv_data_count = msg->covoyfoamtion[0].convoyacceptresponse.size()) > 0)
		{
			if (cv_data_count > 1)
				SWM_LOG("cav_data_callback: CFM's CVRES count is %d. use only element[0].", cv_data_count);
		}
	}
	//else SWM_LOG("$$$$$ CFM is not exist.\r\n");

	// if CSM data exist
	if ((cv_data_count = msg->covoystatus.size()) > 0)
	{
		//SWM_LOG("$$$$$ CSM is exist.\r\n");
		if (cv_data_count > 1)
			SWM_LOG("cav_data_callback: CSM count is %d. use only element[0].", cv_data_count);
	}
	//else SWM_LOG("$$$$$ CSM is not exist.\r\n");

	// if CLCM data exist
	if ((cv_data_count = msg->convoylanechange.size()) > 0)
	{
		//SWM_LOG("$$$$$ CLCM is exist.\r\n");
		if (cv_data_count > 1)
			SWM_LOG("cav_data_callback: CLCM count is %d. use only element[0].", cv_data_count);
		if ((cv_data_count = msg->convoylanechange[0].lanechangeresponse.size()) > 0)
		{
			if (cv_data_count > 1)
				SWM_LOG("cav_data_callback: CLCM's CVRES count is %d. use only element[0].", cv_data_count);
		}
	}
	//else SWM_LOG("$$$$$ CLCM is not exist.\r\n");
#endif

	// add cavdata to CAVDATA structures.
	if (send_cav_data_msg(msg) < 0)
	{
		SWM_LOG("%s cav_data_callback : fail to send bsm data.\n", timeval_print(0));
	}
}

void *swm_ros_sub_proc(void *args)
{
	static int retval = 0xf1;

	SWM_LOG("%s ###### ROS SUBSCRIBER Proc Start ######\n", timeval_print(0));

	ros::Subscriber ros_cav_sub = ros_swm_ldm_nh->subscribe("ros_snu_msg", 2000, cav_data_callback);
	ros::spin();

	SWM_LOG("%s ###### ROS SUBSCRIBER Proc Stop ######\n", timeval_print(0));
	pthread_exit((void*)&retval);
}

/*void *swm_ros_pub_proc(void *args)
{
	static int retval = 0xf2;
	int chk_msg = 0;
	static u16 timeout = 0, send_seq = 0;

	swm_ldm::st_SWMLDM msg;

	ros::Rate loop_rate(100);
	//ros_swm_ldm_pub = ros_swm_ldm_nh->advertise<swm_ldm::st_SWMLDM>("ros_swm_ldm_msg", 2000);
	swm_ldm_publisher_init();

	SWM_LOG("###### ROS PUBLISHER Proc Start ######\n");

	while (ros::ok())
	{
		msg = swm_ldm::st_SWMLDM();

		if (addSIGtoSWMLDM(msg, send_seq) > 0)
		{
			chk_msg++;
			send_seq++;
			// add it. success or fail. does not consider fail-over.
		}
		chk_msg += addBSMtoSWMLDM(msg, 0);
		chk_msg += addRSAtoSWMLDM(msg, 0);
		chk_msg += addEVAtoSWMLDM(msg, 0);

		if (chk_msg != 0)
		{
			ros_swm_ldm_pub.publish(msg);
			chk_msg = 0;
			// remove it. success or fail. does not consider fail-over.
			//if (SWMLDM_HAVE_SIG_PL(msg.payloadMask)) SPaTRemoveAll();
		}

		if (++timeout > 500)
		{
			//set_cav_continued(0);
			timeout = 0;
		}
		loop_rate.sleep();
	}

	SWM_LOG("###### ROS PUBLISHER Proc Stop ######");
	pthread_exit((void*)&retval);
}*/

void swm_ldm_node_init(ros::NodeHandle* nh)
{
	ros_swm_ldm_nh = nh;
}

void swm_ldm_publisher_init(void)
{
	ros_swm_ldm_pub = ros_swm_ldm_nh->advertise<swm_ldm::st_SWMLDM>("ros_swm_ldm_msg", 2000);
}

ros::NodeHandle* swm_ldm_node_handle(void)
{
	return ros_swm_ldm_nh;
}

void swm_ldm_msg_publish(swm_ldm::st_SWMLDM& msg)
{
	swm_ldm_msg_log(msg);
	ros_swm_ldm_pub.publish(msg);
}

void swm_ldm_node_finalize(void)
{
	ros_swm_ldm_nh->shutdown();
}

