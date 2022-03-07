/*
 * vlecu_main.cpp
 * 
 * Created on: Mar 27, 2021
 *		Author: yhcho
 */

// Header
#include "ros/ros.h"	// ROS 기본 헤더 파일
#include <sys/types.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include "swm_define.h"
#include "swm_common.h"
#include "swm_log.h"
#include "swm_menu.h"
#include "swm_ros.h"
#include "swm_ldm.h"


int main(int argc, char **argv)	// 노드 메인 함수
{
	pthread_t t_comm_proc;
	pthread_t t_ldm_proc, t_data_proc;
	pthread_t t_ros_sub_proc/*, t_ros_pub_proc*/;
	u16 save_flag = 0;

	if (argc > 1 ) {
		for (int i = 1; i < argc; i++)
		{
			int rflag = get_run_flag(argv[i]);
			switch (rflag)
			{
			case RUN_FLAG_SAVE:
				save_flag = 1;
				break;
			case RUN_FLAG_DEBUG:
				break;
			case RUN_FLAG_WPATH:
				set_work_path(argv[i]);
				break;
			default:
				break;
			}
		}
	}

	SWM_LOG("%s ###### SWM LDM node start ######\n", timeval_print(0));

	ros::init(argc, argv, "swm_ldm_node");	//노드명 초기화
	ros::NodeHandle ros_swm_ldm_nh;
	swm_ldm_node_init(&ros_swm_ldm_nh);
	swm_ldm_publisher_init();

	/*
	 swm.config 파일을 읽어들여서 설정하거나, 파일이 없으면 default 값으로 만든다.
	*/
    init_config_file();

	if (save_flag)
		set_data_save(LDM_SAVE_DATA_ALL);

	/*
	 각각의 thread를 실행 시킨다.
	*/
	 // run each threads
    pthread_create(&t_comm_proc, NULL, &common_proc, NULL);
    pthread_create(&t_ldm_proc, NULL, &ldm_proc, NULL);

	pthread_create(&t_ros_sub_proc, NULL, &swm_ros_sub_proc, NULL);
	//pthread_create(&t_ros_pub_proc, NULL, &swm_ros_pub_proc, NULL);
	pthread_join(t_ros_sub_proc, NULL);
	//pthread_join(t_ros_pub_proc, NULL);

	// set_cli_run(0);
    //set_ldm_run(0);
	// set_cav_run(0);
    logmem_close();
	pthread_join(t_comm_proc, NULL);

	swm_ldm_node_finalize();

	SWM_LOG("%s ##### SWM LDM node exit ######\n", timeval_print(0));
	return (EXIT_SUCCESS);
}
