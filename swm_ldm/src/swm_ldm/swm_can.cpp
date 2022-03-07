/*
 * swm_can.c
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#include <fcntl.h>
#include "swm_common.h"
#include "swm_can.h"

 int can_open(const char *port, int *can_id)
 {
//    int can_sock;
    struct ifreq ifr;
    struct sockaddr_can addr;

 	/* open socket */
 	*can_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);
 	if(*can_id < 0)
 	{
 	    return (-1);
 	}

 	addr.can_family = AF_CAN;
 	strcpy(ifr.ifr_name, port);

 	if (ioctl(*can_id, SIOCGIFINDEX, &ifr) < 0)
 	{
 	    return (-1);
 	}

 	addr.can_ifindex = ifr.ifr_ifindex;

 	fcntl(*can_id, F_SETFL, O_NONBLOCK);

 	if (bind(*can_id, (struct sockaddr *)&addr, sizeof(addr)) < 0)   {
 	    return (-1);
    }

 	return 0;
 }

 int can_close(int canfd)
 {
     close(canfd);
     return 0;
 }

 int can_send(int canfd, struct can_frame *wframe)
 {
     int ret;
     ret = write(canfd, wframe, sizeof(struct can_frame));
     if (ret != sizeof(struct can_frame))
         return (-1);
     return 0;
 }

 int cavdata_write(int canfd, int sid, u8 *buf)
 {
     int ret;
     struct can_frame wr_frame;

     memset( &wr_frame, 0, sizeof(struct can_frame));
     wr_frame.can_id = sid;
     switch(sid) {
         case MSG_ID_CAV_COUNT_TIME :
             wr_frame.can_dlc = MSG_SIZE_CAV_COUNT_TIME;
             break;
         case MSG_ID_VID_INFO :
             wr_frame.can_dlc = MSG_SIZE_VID_INFO;
             break;
         case MSG_ID_GPS_INFO :
             wr_frame.can_dlc = MSG_SIZE_GPS_INFO;
             break;
         case MSG_ID_TRANSMISSION :
             wr_frame.can_dlc = MSG_SIZE_TRANSMISSION;
             break;
         case MSG_ID_ACCELERATION :
             wr_frame.can_dlc = MSG_SIZE_ACCELERATION;
             break;
         case MSG_ID_BRAKES :
             wr_frame.can_dlc = MSG_SIZE_BRAKES;
             break;
     }

     memcpy(wr_frame.data, buf, wr_frame.can_dlc);
     ret = can_send(canfd, &wr_frame);

     if(ret < 0) return -1;
     return wr_frame.can_dlc;
 }

 int can_read(int canfd, struct can_frame *rframe)
 {
     int rsize;

     rsize = read(canfd, &rframe, sizeof(struct can_frame));

     return rsize;
 }

 int cavdata_read(int canfd, u8 *buf)
 {
     int cavid = 0;
     struct can_frame rd_frame;

     if(can_read(canfd, &rd_frame)<0)
         return 0;

     memcpy(buf, rd_frame.data, rd_frame.can_dlc);
     return rd_frame.can_id;
 }
