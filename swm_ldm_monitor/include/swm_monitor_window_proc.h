/*
 * swm_monitor_window_proc.h
 *
 *  Created on: May 22, 2021
 *      Author: yhcho
 */

#ifndef SWM_MONITOR_WINDOW_PROC_H_
#define SWM_MONITOR_WINDOW_PROC_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void ldm_monitor_window_close(void);
extern bool ldm_monitor_windows_alive(void);
extern void ldm_monitor_windows_display(void);
extern void *ldm_monitor_window_proc(void* args);

extern void display_traffic_light_signal(void);

extern void swm_color_init(void);
extern void cprintf(int color, const char* fmt, ...);

extern int test();
extern void test_traffic_signal_process(void);
extern void test_road_side_alert(void);

#ifdef __cplusplus
}
#endif

#endif /* SWM_MONITOR_WINDOW_PROC_H_ */
