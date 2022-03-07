/*
 * swm_traffic_light.h
 *
 *  Created on: May 22, 2021
 *      Author: yhcho
 */

#ifndef SWM_TRAFFIC_LIGHT_H_
#define SWM_TRAFFIC_LIGHT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// traffic light signal
#define	TRAFFIC_LIGHT_COLOR_UNKNOWN		0
#define	TRAFFIC_LIGHT_COLOR_RED			1
#define	TRAFFIC_LIGHT_COLOR_YELLOW		2
#define	TRAFFIC_LIGHT_COLOR_GREEN		3
#define	TRAFFIC_LIGHT_COLOR_BLACK		4
#define	TRAFFIC_LIGHT_COLOR_RED_LEFT	5
#define	TRAFFIC_LIGHT_COLOR_GREEN_LEFT	6

extern const char* get_traffic_signal(u8 state);
extern void output_traffic_signal_string(int ev_state);

#ifdef __cplusplus
}
#endif

#endif /* SWM_TRAFFIC_LIGHT_H_ */
