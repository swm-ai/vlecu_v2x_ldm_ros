/*
 * swm_shm.c
 *
 *  Created on: May 20, 2020
 *      Author: kurt
 */


#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <fcntl.h>

#include "swm_common.h"
#include "swm_shm.h"

int shm_create(int key, int size)
{
	int shmid;
	if((shmid = shmget((key_t)key, size, IPC_CREAT| IPC_EXCL | 0666)) == -1) {
//	    printf("There was shared memory.");
		SWM_LOG("%s shared memory create: %d, %d\n", timeval_print(0), key, size);
	    shmid = shmget((key_t)key, size, IPC_CREAT| 0666);

	    if(shmid == -1)
	    {
	        SWM_LOG("%s shared memory create: fail\n", timeval_print(0));
	        return -1;
	    }
	    else
	    {
	        shm_free(shmid);
	        shmid = shmget((key_t)key, size, IPC_CREAT| 0666);

	        if(shmid == -1)
	        {
	            SWM_LOG("%s shm_create:shmget: error\n", timeval_print(0));
	            return -2;
	        }
	    }
	}

	return shmid;
}

int shm_connect(int key)
{
	int shmid;
//	void *shmaddr;

    if((shmid = shmget((key_t)key, 0, 0)) == -1)
    {
        SWM_LOG("%s shm_connect:shmget error, %d\n", timeval_print(0), key);
		return -1;
    }
	return shmid;
}

int shm_free(int shm_id)
{
	if(shmctl(shm_id, IPC_RMID, 0) == -1)
    {
        SWM_LOG("%s shm_free:shmctl failed, %d\n", timeval_print(0), shm_id);
        return -1;
    }

    SWM_LOG("%s shared memory free\n", timeval_print(0));
	return 0;
}

int shm_write(int shm_id, char *wbuf, int size)
{
	void *shmaddr;
	shmaddr = shmat(shm_id, (void *)0, 0);

    if(shmaddr == (void *)-1)
    {
        SWM_LOG("%s shm_write:shmat failed, %d\n", timeval_print(0), shm_id);
        return -2;
    }

    memcpy(shmaddr, wbuf, size);

    if(shmdt(shmaddr) == -1)
    {
        SWM_LOG("%s shm_write:shmdt failed, %d\n", timeval_print(0), shm_id);
        return -3;
    }
    return 0;
}
/*
int length;
char *shmaddr;
//	char shm_buffer[SHM_BUFF_SIZE] = {0};

shmaddr = (char *)shmat(shm_id, (void *)0, 0);
if(shmaddr == (void *)-1)
{
	std::cout << "shmat failed" << std::endl;
	return -1;
}

length = sizeof(t_SHMData);
memcpy(rbuf, shmaddr, length);

if(shmdt(shmaddr) == -1)
{
	std::cout << "shmdt failed" << std::endl;
	return -2;
}

return length;
*/
int shm_read(int shm_id, char *rbuf, int size)
{
	void *shmaddr;

	shmaddr = shmat(shm_id, (void *)0, 0);

    if(shmaddr == (void *)-1)
	{
	    SWM_LOG("%s shm_read_cmd:shmat failed, %d\n", timeval_print(0), shm_id);
		return -1;
	}
/*
	sscanf(shmaddr, "%[a-z]", cmd_str);
	for(i=0; i<CLI_COMMAND_MAX; i++)	{
		if(strncmp(cmd_str, cmds[i].cmd, cmds[i].leng)==0)	{
			read_commnad = i;
			break;
		}
	}
*/
	memcpy(rbuf, shmaddr, size);

	if(shmdt(shmaddr) == -1)
	{
	    SWM_LOG("%s shm_read_cmd:shmdt failed, %d\n", timeval_print(0), shm_id);
	    return -2;
	}
	return 0;
}

int shm_clear(int shm_id, int size)
{
	void *shmaddr;
	shmaddr = shmat(shm_id, (void *)0, 0);
    if(shmaddr == (void *)-1)
    {
        SWM_LOG("%s shm_clear:shmat failed, %d\n", timeval_print(0), shm_id);
        return -1;
    }

    memset((char *)shmaddr, 0, size);

    if(shmdt(shmaddr) == -1)
    {
        SWM_LOG("%s shm_clear:shmdt failed, %d\n", timeval_print(0), shm_id);
        return -2;
    }
    return 0;
}
