/*
 * swm_menu.h
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#ifndef SWM_MENU_H_
#define SWM_MENU_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWM_MENU_C_
#define SWM_MENU_EXTERN
#else
#define SWM_MENU_EXTERN extern
#endif

typedef enum {
  RUN_FLAG_NONE = 0,
  RUN_FLAG_SAVE,
  RUN_FLAG_DEBUG,
  RUN_FLAG_WPATH,
  RUN_FLAG_MAX
}_RUN_FLAG_OPT;

SWM_MENU_EXTERN int get_run_flag(char *pstr);

#ifdef __cplusplus
}
#endif

#endif /* SWM_MENU_H_ */
