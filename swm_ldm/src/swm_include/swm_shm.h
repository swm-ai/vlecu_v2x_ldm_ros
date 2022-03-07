/*
 * swm_shm.h
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#ifndef SWM_SHARED_H_
#define SWM_SHARED_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWM_SHARED_C_
#define SHARED_EXTERN
#else
#define SHARED_EXTERN extern
#endif

#include <sys/types.h>
#include <sys/shm.h>
#include <stdint.h>
#include <fcntl.h>
#include "swm_define.h"

#define SHM_BUFF_SIZE  (100*KBYTE)

typedef enum
{
	SHM_DATA_CLEAR		= 0,
	SHM_DATA_CONTINUE	= 1,
	SHM_DATA_SET		= 2,

	SHM_DATA_END
}_SHM_RD_FLAG;

SHARED_EXTERN int shm_create(int key, int size);
SHARED_EXTERN int shm_connect(int key);
SHARED_EXTERN int shm_free(int shm_id);
SHARED_EXTERN int shm_write(int shm_id, char *wbuf, int size);
SHARED_EXTERN int shm_read(int shm_id, char *rbuf, int size);
SHARED_EXTERN int shm_clear(int shm_id, int size);

#ifdef __cplusplus
}
#endif

#endif /* SWM_SHARED_H_ */
