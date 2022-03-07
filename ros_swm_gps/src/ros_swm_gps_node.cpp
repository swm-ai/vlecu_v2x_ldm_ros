#include "ros/ros.h"
#include "ros_swm_gps/MsgGpsSwm.h"
#include <errno.h>
#include <string.h>

#include "swm_termios.h"


//#define GPS_READ_TEST
#define DEFAULT_SERIAL_PORT	"/dev/ttyUSB0"

#if defined(GPS_READ_TEST)
static const char* gps_test_data[] = {
"$GNRMC,001308.95,A,3723.8087191,N,12657.8365004,E,0.077,,010221,8.39,W,D,V*5C",
"$GNGGA,001308.95,3723.8087191,N,12657.8365004,E,2,11,2.42,64.216,M,18.513,M,1.0,0000*55",
"$GNHDT,,*51"
};
#endif


char* readSerialPort(FILE* fp, char* buffer, size_t size)
{
#if defined(GPS_READ_TEST)
	static int count = 0;

	if (count > 2) count = 0;

	return (char*)gps_test_data[count++];
#else
	if (!fgets(buffer, size, fp))
	{
		buffer[0] = 0;
		ROS_INFO("fail to read from serial port");
	} else {
		int len = strlen(buffer);
		if (buffer[len - 2] == '\r')
			buffer[len - 2] = 0;
		else if (buffer[len - 1] == '\n')
			buffer[len - 1] = 0;
		//ROS_INFO("read data from serial port : [%s]", buffer);
	}

	return buffer;
#endif
}

void closeSerialPort(FILE* fp)
{
#if !defined(GPS_READ_TEST)
	fclose(fp);
#endif
}

int main(int argc, char** argv)
{
	FILE* fp = NULL;
	char buffer[512] = {0,};
	ros::init(argc, argv, "ros_swm_gps_node");
	ros::NodeHandle nh;

	ROS_INFO("start ROS SWM GPS node");

#if !defined(GPS_READ_TEST)
	if (!(fp = openSerialPort(DEFAULT_SERIAL_PORT)))
	{
		ROS_ERROR("fail to start ROS SWM GPS node");
		return -1;
	}
#endif


	ros::Publisher ros_swm_gps_pub = nh.advertise<ros_swm_gps::MsgGpsSwm>("ros_swm_gps_msg", 2000);
	ros::Rate loop_rate(100);

	ros_swm_gps::MsgGpsSwm msg;

	ROS_INFO("success to start ROS SWM GPS node");

	while (ros::ok())
	{
		msg.data = readSerialPort(fp, buffer, sizeof(buffer));
		//ROS_INFO("read data from serial port : [%s]", msg.data.c_str());
		if (strlen(buffer) > 0)
		{
			ros_swm_gps_pub.publish(msg);
			loop_rate.sleep();
		}
		/*else
			ROS_ERROR("get null data.\r\n");*/
	}

	closeSerialPort(fp);

	return 0;
}
