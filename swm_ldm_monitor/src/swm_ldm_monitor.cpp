#include "ros/ros.h"
#include "swm_ldm/st_SWMLDM.h"

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
#include "swm_ldm_data.h"
#include "swm_traffic_light.h"
#include "swm_monitor_window_proc.h"

#define BUF_SIZE			1024

void ldm_call_back(const swm_ldm::st_SWMLDM::ConstPtr& msg)
{
	SWM_INFO("receive ros_swm_ldm_msg\r\n");

	// if msg have SPaT
	if (msg->payloadMask & SWMLDM_PAYLOAD_SIG_FLAG)
	{
		SWM_INFO("##### Traffic Signal mesage info\r\n");
		ldm_monitor_lock();
		for (int i = 0; i < msg->trSignal.size(); i++)
		{
			swm_ldm::st_TRSIGNAL ldm_data = msg->trSignal[i];
			SWM_INFO("\t- Traffic Signal [%d] -\r\n", i + 1);
			/*
			 * string trafficLightId
			 * uint8 evState
			 * uint32 endTime
			 */
			SWM_INFO("\ttraffic light id : %s\r\n", ldm_data.trafficLightId.c_str());
			SWM_INFO("\ttraffic light signal : %s\r\n", get_traffic_signal(ldm_data.evState));
			SWM_INFO("\tremain time : %d\r\n", ldm_data.endTime);
			add_traffic_signal_data_list(ldm_data);
		}
		ldm_monitor_unlock();
		SWM_INFO("##### Traffic Signal mesage info\r\n");
	}
	// if msg have BSM
	if (msg->payloadMask & SWMLDM_PAYLOAD_BSM_FLAG)
	{
		ldm_monitor_lock();
		swm_ldm::st_BSM bsm_data = msg->bsmMsg[0];
		SWM_INFO("BSM mesage - ");
		//printf("id:%d, lat:%d, long:%d, elev:%d, cavbsm is %s", );
		SWM_INFO("\r\n");
		add_bsm_data_list(bsm_data);
		ldm_monitor_unlock();
	}
	// if msg have RSA
	if (msg->payloadMask & SWMLDM_PAYLOAD_RSA_FLAG)
	{
		ldm_monitor_lock();
		swm_ldm::st_RSA rsa_data = msg->rsaMsg[0];
		SWM_INFO("RSA mesage - ");
		SWM_INFO("itis:%d, latutidue:%d, longitude:%d, cavrsa regional data count:%ld\r\n", 
				rsa_data.itis, 
				rsa_data.latitude, 
				rsa_data.longitude, 
				rsa_data.cavrsa.size());
			add_rsa_data_list(rsa_data);
		ldm_monitor_unlock();
	}
	// if msg have EVA
	if (msg->payloadMask & SWMLDM_PAYLOAD_EVA_FLAG)
	{
		ldm_monitor_lock();
		swm_ldm::st_EVA eva_data = msg->evaMsg[0];
		SWM_INFO("EVA mesage:");
		SWM_INFO("\r\n");
		add_eva_data_list(eva_data);
		ldm_monitor_unlock();
	}
}

int main(int argc, char** argv)
{
	pthread_t t_ldm_monitor_window_proc = 0;
	ros::init(argc, argv, "swm_ldm_monitor");
	ros::NodeHandle nh;

	SWM_INFO("##### SWM LDM MONITOR start #####\r\n");

	pthread_create(&t_ldm_monitor_window_proc, NULL, &ldm_monitor_window_proc, NULL);
	
	// register ros_swm_ldm_msg subscriber for vl-ecu
	ros::Subscriber ros_swm_ldm_sub = nh.subscribe("ros_swm_ldm_msg", 2000, ldm_call_back);
	ros::spin();

	ldm_monitor_window_close();
	pthread_join(t_ldm_monitor_window_proc, NULL);
	nh.shutdown();

	SWM_INFO("##### SWM LDM MONITOR stop #####\r\n");

	return 0;
}

