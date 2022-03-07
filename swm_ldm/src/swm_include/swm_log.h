/*
 * swm_log.h
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#ifndef SWM_LOG_H_
#define SWM_LOG_H_

#include "swm_define.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWM_LOG_C_
#define LOG_EXTERN
#else
#define LOG_EXTERN extern
#endif

#ifdef CLI_LOGFILE
#define LOGFILE_SIZE (100*MBYTE)	//	10 MB

#define LOG_FOLDER		"./log"
#define LDM_LOGNAME		"/ldmmsg_0000_000000.log"
#define CAV_LOGNAME		"/cavmsg_0000_000000.log"
#define ERR_LOGNAME		"/errlog_0000_000000.log"

typedef struct {
    // yhcho // add to process work path for ros // 2021.04.20
	char logname[128/*32*/];

    int isopen;
    int flength;
}Logfile_t;

#endif

#ifdef CLI_LOGFILE
LOG_EXTERN int LogFileInit(int type, int info);
LOG_EXTERN int LogFileChange(int type, int info,  FILE *pf);
LOG_EXTERN void AddLogString(char *str, int len, int type, int info);
#endif

//LOG_EXTERN void HelpMessage(void);

//LOG_EXTERN void *cli_proc(void *args);
LOG_EXTERN int getStrEnd(char *buf);
#ifdef SHM_USE
LOG_EXTERN void write_shmem(int wid, char *buf, int size);
#endif
LOG_EXTERN int get_log_init(void);

LOG_EXTERN void logmem_wstring(int wid, int info, const char *str, ...);
LOG_EXTERN int logmem_init(void);
LOG_EXTERN int logmem_close(void);

LOG_EXTERN int common_init(void);
LOG_EXTERN void *common_proc(void *args);
#ifdef __cplusplus
}
#endif

#endif /* SWM_LOG_H_ */
