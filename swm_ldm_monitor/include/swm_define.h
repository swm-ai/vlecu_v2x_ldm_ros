/*
 * define.h
 *
 *  Created on: Mar 4, 2021
 *      Author: yhcho
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#if 0
#define SWM_ERROR(str, ...) printf("[ERROR]" str "\n", ##__VA_ARGS__)
#define SWM_FUNC(str, ...)  printf("[%s:%d]" str "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define SWM_INFO(str, ...)  printf("[INFO]" str "\n", ##__VA_ARGS__)
#else
#define SWM_ERROR(str, ...)
#define SWM_FUNC(str, ...)
#define SWM_INFO(str, ...)
#endif

typedef unsigned char 		u8;
typedef unsigned short 		u16;
typedef unsigned int		u32;
typedef unsigned long long 	u64;

typedef signed char 		s8;
typedef signed short 		s16;
typedef signed int			s32;
typedef signed long long 	s64;

#define TRUE	1
#define FALSE	0

#endif /* DEFINE_H_ */
