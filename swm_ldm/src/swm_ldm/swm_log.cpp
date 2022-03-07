/*
 * swm_log.c
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>

// swm ldm interface
#include "swm_common.h"
#include "swm_shm.h"

static int logd_init=0;
static int shm_log_fd, shm_info_fd, shm_cmd_fd;
static pthread_mutex_t 	mutex_log;

#ifdef CLI_LOGFILE
Logfile_t terr_log;
Logfile_t tldm_msg, tcav_msg;
//Logfile_t tmsglog,

int LogFileInit(int type, int info)
{
	if(type==CLI_MSG_INFO)	{
		if(info>CLI_INFO_NONE && info<CLI_INFO_CAV)	{
			memset(&tldm_msg, 0, sizeof(Logfile_t));
//			sprintf(tldm_msg.logname, "%s", "./log/ldmmsg_0000_000000.log");
			// yhcho // add to process work path for ros // 2021.04.20
			if (get_work_path() != NULL)
				snprintf(tldm_msg.logname, sizeof(tldm_msg.logname), "%s/%s%s", get_work_path(), LOG_FOLDER+2, LDM_LOGNAME);
			else
				sprintf(tldm_msg.logname, "%s%s", LOG_FOLDER, LDM_LOGNAME);
			// yhcho // add to process work path for ros // 2021.04.20
			getlogtimestr(tldm_msg.logname + strlen(tldm_msg.logname) -15);
			tldm_msg.isopen = 1;
		}
		else if(info==CLI_INFO_CAV)	{
			memset(&tcav_msg, 0, sizeof(Logfile_t));
//			sprintf(tcav_msg.logname, "%s", "./log/cavmsg_0000_000000.log");
			// yhcho // add to process work path for ros // 2021.04.20
			if (get_work_path() != NULL)
				snprintf(tcav_msg.logname, sizeof(tcav_msg.logname), "%s/%s%s", get_work_path(), LOG_FOLDER+2, CAV_LOGNAME);
			else
				sprintf(tcav_msg.logname, "%s%s", LOG_FOLDER, CAV_LOGNAME);
			// yhcho // add to process work path for ros // 2021.04.20
			getlogtimestr(tcav_msg.logname + strlen(tcav_msg.logname) -15);
			tcav_msg.isopen = 1;
		}

	}
	else if(type==CLI_MSG_LOG)	{
		memset(&terr_log, 0, sizeof(Logfile_t));
//		sprintf(terr_log.logname, "%s", "./log/errlog_0000_000000.log");
		// yhcho // add to process work path for ros // 2021.04.20
		if (get_work_path() != NULL)
			snprintf(terr_log.logname, sizeof(terr_log.logname), "%s/%s%s", get_work_path(), LOG_FOLDER+2, ERR_LOGNAME);
		else
			sprintf(terr_log.logname, "%s%s", LOG_FOLDER, ERR_LOGNAME);
		// yhcho // add to process work path for ros // 2021.04.20
		getlogtimestr(terr_log.logname + strlen(terr_log.logname) -15);
		terr_log.isopen = 1;
	}
	return 0;
}

// getcurtime
int LogFileChange(int type, int info, FILE *pf)
{
	Logfile_t	*plog;

	if(pf<=0)	return -1;
	fclose(pf);

	switch(type)	{
	case CLI_MSG_INFO:
		if(info>CLI_INFO_NONE && info<CLI_INFO_CAV)	{
			memset(&tldm_msg, 0, sizeof(Logfile_t));
			// yhcho // add to process work path for ros // 2021.04.20
			if (get_work_path() != NULL)
				snprintf(tldm_msg.logname, sizeof(tldm_msg.logname), "%s/%s%s", get_work_path(), LOG_FOLDER+2, LDM_LOGNAME);
			else
				sprintf(tldm_msg.logname, "%s%s", LOG_FOLDER, LDM_LOGNAME);
			// yhcho // add to process work path for ros // 2021.04.20
			getlogtimestr(tldm_msg.logname + strlen(tldm_msg.logname) -15);
			plog = &tldm_msg;
		}
		else if(info==CLI_INFO_CAV)	{
			memset(&tcav_msg, 0, sizeof(Logfile_t));
			// yhcho // add to process work path for ros // 2021.04.20
			if (get_work_path() != NULL)
				snprintf(tcav_msg.logname, sizeof(tcav_msg.logname), "%s/%s%s", get_work_path(), LOG_FOLDER+2, CAV_LOGNAME);
			else
				sprintf(tcav_msg.logname, "%s%s", LOG_FOLDER, CAV_LOGNAME);
			// yhcho // add to process work path for ros // 2021.04.20
			getlogtimestr(tcav_msg.logname + strlen(tcav_msg.logname) -15);
			plog = &tcav_msg;
		}
		break;
	case CLI_MSG_LOG:
		memset(&terr_log, 0, sizeof(Logfile_t));
		// yhcho // add to process work path for ros // 2021.04.20
		if (get_work_path() != NULL)
			snprintf(terr_log.logname, sizeof(terr_log.logname), "%s/%s%s", get_work_path(), LOG_FOLDER+2, ERR_LOGNAME);
		else
			sprintf(terr_log.logname, "%s%s", LOG_FOLDER, ERR_LOGNAME);
		// yhcho // add to process work path for ros // 2021.04.20
		getlogtimestr(terr_log.logname + strlen(terr_log.logname) -15);
		plog = &terr_log;
		break;

	default:
	case CLI_MSG_COMMAND:
		return -2;
	}

	plog->isopen = 1;
	pf = fopen(plog->logname, "a+");

	return 0;
}

void AddLogString(char *str, int len, int type, int info)
{
	Logfile_t	*plog;
	FILE *pf;
	unsigned int fsize;

	if(!str) return;
REOPEN:
	switch(type)	{
	case CLI_MSG_INFO:
//		if(!get_asn_log())
//			return;
		if(info>CLI_INFO_NONE && info<CLI_INFO_CAV)	{
			plog = &tldm_msg;
		}
		else if(info==CLI_INFO_CAV)	{
			plog = &tcav_msg;
		}
		else	return;
		break;

	case CLI_MSG_LOG:
		plog = &terr_log;
		break;

	default:
	case CLI_MSG_COMMAND:
		return;
	}
	if(plog->isopen != 1) return;
/*
	if( (plog->flength + len)>LOGFILE_SIZE )	{
		NewLogFile(type);
	}
*/
	pthread_mutex_lock(&mutex_log);
	pf = fopen(plog->logname, "a+");
	if(!pf)	{
		pthread_mutex_unlock(&mutex_log);
		return;
	}

	fseek(pf, 0, SEEK_END);
	fsize = ftell(pf);
	if( (fsize + len)>LOGFILE_SIZE )	{
		plog->isopen = 0;
		fclose(pf);
		pf = 0;
		LogFileInit(type, info);
		pthread_mutex_unlock(&mutex_log);

		goto REOPEN;
	}
	if(type==CLI_MSG_INFO)	{
		switch (info)	{
			case CLI_INFO_POS :
				fwrite("[POS] ", 6, 1, pf);
				break;
			case CLI_INFO_SIG :
				fwrite("[SIG] ", 6, 1, pf);
				break;
			case CLI_INFO_MAP :
				fwrite("[MAP] ", 6, 1, pf);
				break;
			case CLI_INFO_BSM :
				fwrite("[BSM] ", 6, 1, pf);
				break;
			case CLI_INFO_CFM :
				fwrite("[CFM] ", 6, 1, pf);
				break;
  			case CLI_INFO_CSM :
			  	fwrite("[CSM] ", 6, 1, pf);
				break;
  			case CLI_INFO_CLC :
			  	fwrite("[CLC] ", 6, 1, pf);
				break;
			case CLI_INFO_PVD :
				fwrite("[PVD] ", 6, 1, pf);
				break;
			case CLI_INFO_RSA :
				fwrite("[RSA] ", 6, 1, pf);
				break;
			case CLI_INFO_EVA :
				fwrite("[EVA] ", 6, 1, pf);
				break;
			case CLI_INFO_RCP :
				fwrite("[RCP] ", 6, 1, pf);
				break;
			case CLI_INFO_VID :
				fwrite("[VID] ", 6, 1, pf);
				break;
			case CLI_INFO_ETC :
				fwrite("[ETC] ", 6, 1, pf);
				break;
			case CLI_INFO_CAV :
				fwrite("[CAV] ", 6, 1, pf);
				break;
			default :
				fwrite("[ETC] ", 6, 1, pf);
				break;
			break;
		}
	}
	fwrite(str, len, 1, pf);
	plog->flength += len;
	fclose(pf);
	pthread_mutex_unlock(&mutex_log);
}

#endif

int getStrEnd(char *buf)
{
	int count;

	count = 0;
	while(*buf != '\n' && *buf != 0 && ((count+1)<MSG_BUF_SIZE))	{
		++count;
		++buf;
	}
	return count;
}

#ifdef SHM_USE
void write_shmem(int wid, char *buf, int size)
{
	if(wid<CLI_MSG_LOG || wid>CLI_MSG_COMMAND)	return;

	switch(wid)	{
		case CLI_MSG_LOG:
			if(shm_log_fd<0) return;
//			shm_log_fd = shm_connect(SHM_LOG_KEY);
			shm_write(shm_log_fd, buf, size);
		break;
		case CLI_MSG_INFO:
			if(shm_info_fd<0) return;
//			shm_info_fd = shm_connect(SHM_INFO_KEY);
			shm_write(shm_info_fd, buf, size);
		break;
		default:
		case CLI_MSG_COMMAND:
			return;
	}
}
#endif
int get_log_init(void)
{
	return logd_init;
}

void logmem_wstring(int wid, int info, const char *str, ...)
{
	va_list ap;
	int cpos=0;
	char buf[MSG_BUF_SIZE] = {0,};

	va_start(ap, str);
    vsprintf(buf, str, ap);
    va_end(ap);

	if(wid==CLI_MSG_LOG)
		printf("%s", buf);
	cpos=getStrEnd(buf);

	if(!get_log_init())	{

#ifdef CLI_LOGFILE
		AddLogString(buf, cpos+1, wid, info);
#endif
		return;
	}

	if(wid<CLI_MSG_LOG || wid>CLI_MSG_INFO)	return;
#ifdef SHM_USE
	pthread_mutex_lock(&mutex_log);
	write_shmem(wid, buf, cpos+1);
	pthread_mutex_unlock(&mutex_log);
#endif

#ifdef CLI_LOGFILE
	buf[cpos] = '\n';
	AddLogString(buf, cpos+1, wid, info);
#endif
}

int logmem_init(void)
{
	int ret = 0;
	shm_log_fd = shm_create(SHM_LOG_KEY, MSG_BUF_SIZE);
    if(shm_log_fd<0) {
        SWM_LOG("%s shm_create error: %d\n", timeval_print(0), SHM_LOG_KEY);
		ret = -1;
    }
	SWM_LOG("%s shm_create shm_log_fd: %d\n", timeval_print(0), shm_log_fd);

	shm_info_fd = shm_create(SHM_INFO_KEY, MSG_BUF_SIZE);
    if(shm_info_fd<0) {
        SWM_LOG("%s shm_create error %d\n", timeval_print(0), SHM_INFO_KEY);
		ret = -2;
    }
	SWM_LOG("%s shm_create shm_info_fd: %d\n", timeval_print(0), shm_info_fd);

	shm_cmd_fd = shm_create(SHM_COMMAND_KEY, MSG_BUF_SIZE);
    if(shm_cmd_fd<0) {
        SWM_LOG("%s shm_create error %d\n", timeval_print(0), SHM_COMMAND_KEY);
		ret = -2;
    }
	SWM_LOG("%s shm_create shm_cmd_fd: %d\n", timeval_print(0), shm_cmd_fd);

	return ret;
}

int logmem_close(void)
{
	int ret = 0;

	if(!logd_init)
		return -1;

	logd_init = 0;

	ret = shm_free(shm_log_fd);
    if(ret<0) {
        SWM_LOG("%s shm_free:shm_log_fd error: %d\n", timeval_print(0));
    }
	ret = shm_free(shm_info_fd);
    if(ret<0) {
        SWM_LOG("%s shm_free:shm_info_fd error %d\n", timeval_print(0));
    }
	ret = shm_free(shm_cmd_fd);
    if(ret<0) {
        SWM_LOG("%s shm_free:shm_info_fd error %d\n", timeval_print(0));
    }	return ret;
}

int common_init(void)
{
	int ret = 0;
	const char *wpath = NULL;

	logmem_init();
	pthread_mutex_init(&mutex_log, NULL);
#ifdef CLI_LOGFILE
	if ((wpath = get_work_path()) != NULL)
	{
		char log_path[1024] = {0,};
		snprintf(log_path, sizeof(log_path), "%s%s", wpath, LOG_FOLDER + 1);
		ret = mkdir(log_path, 775);
	}
	else
		ret = mkdir(LOG_FOLDER, 775);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_POS);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_SIG);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_MAP);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_BSM);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_CFM);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_CSM);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_CLC);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_PVD);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_RSA);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_EVA);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_RCP);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_VID);
	LogFileInit(CLI_MSG_INFO, CLI_INFO_CAV);
	LogFileInit(CLI_MSG_LOG, 0);
#endif
	logd_init = 1;
	return ret;
}

void *common_proc(void *args)
{
	static int retval=0xf1;
	int command;

	SWM_LOG("%s ###### COMMON Proc Start ######\n", timeval_print(0));
	common_init();

	while(logd_init)	{
		//SWM_FUNC("common_proc---\n");
		usleep(100);
	}

	if(logd_init)	logmem_close();

	SWM_LOG("%s ###### COMMON Proc close ######\n", timeval_print(0));
	pthread_exit((void*)&retval);
}
