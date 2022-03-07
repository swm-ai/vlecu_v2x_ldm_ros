/*
 * swm_ros.h
 *
 *  Created on: Mar 27, 2021
 *      Author: yhcho
 */

#ifndef SWM_ROS_H_
#define SWM_ROS_H_

#include "ros/ros.h"
#include "swm_ldm/st_SNUDATA.h"
#include "swm_ldm/st_SWMLDM.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWM_ROS_C_
#define SWM_ROS_EXTERN
#else
#define SWM_ROS_EXTERN extern
#endif

#include <fcntl.h>
#include "swm_define.h"


SWM_ROS_EXTERN void swm_ldm_node_init(ros::NodeHandle* nh);
SWM_ROS_EXTERN void swm_ldm_publisher_init(void);
SWM_ROS_EXTERN ros::NodeHandle* swm_ldm_node_handle(void);
SWM_ROS_EXTERN void swm_ldm_msg_publish(swm_ldm::st_SWMLDM& msg);
SWM_ROS_EXTERN void swm_ldm_node_finalize(void);

SWM_ROS_EXTERN void cav_data_callback(const swm_ldm::st_SNUDATA::ConstPtr& msg);
SWM_ROS_EXTERN void *swm_ros_sub_proc(void *args);
//SWM_ROS_EXTERN void *swm_ros_pub_proc(void *args);

#ifdef __cplusplus
}
#endif

#endif /* SWM_ROS_H_ */
