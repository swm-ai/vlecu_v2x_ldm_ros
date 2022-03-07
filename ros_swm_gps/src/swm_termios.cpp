/*
 * swm_termios.c
 *
 *  Created on: Mar 8, 2021
 *      Author: yhcho
 */

//#define SWM_TERMIOS_C_

#include "ros/ros.h"
#include "ros_swm_gps/MsgGpsSwm.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "swm_define.h"
#include "swm_termios.h"



#if 0
FILE* openSerialPort(const char* portName)
{
	FILE* fp = fopen(portName, "r");

	if (!fp)
		ROS_INFO("fail to open %s. cause error : %s", portName, strerror(errno));

	return fp;
}
#else
FILE* openSerialPort(const char* portName)
{
	struct termios options;
	int fd;

	ROS_INFO("open serial port %s\r\n", portName);

	if ((fd = open(portName, O_RDONLY | O_NOCTTY )) < 0)
	{
		ROS_ERROR("fail to open serial port(%s) - %s\r\n", portName, strerror(errno));
		return NULL;
	}

	// Configure port for 8N1 transmission, 115200 baud, SW flow control.
	tcgetattr(fd, &options);
	
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~CRTSCTS;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~OPOST;

	// Set the new options for the port "NOW"
	tcsetattr(fd, TCSANOW, &options);

	return fdopen(fd, "r");
}
#endif

