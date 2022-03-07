/*
 * define.h
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SWM_LITTEL_ENDIAN

#define USE_ROS

#define KBYTE           1024
#define MBYTE           (KBYTE*KBYTE)
#define MSG_BUF_SIZE	(200*KBYTE)

//#define CAN_USE
//#define MQ_USE
#define SHM_USE
//#define HDMAP_USE
//#define DETAIL_LOG
#define USE_DETAIL_SOCKET_LOG
//#define USE_SIGNAL_RSA_TEST
#define USE_BUS_TRAFFIC_LIGHT

typedef enum {
  CLI_MSG_LOG = 0,
  CLI_MSG_INFO,
  CLI_MSG_COMMAND,
  CLI_MSG_MAX,
}_CLI_MSG_TYPE;

typedef enum {
  CLI_INFO_NONE = 0,
  CLI_INFO_POS,
  CLI_INFO_SIG, // it's ldm signal message. this message is information combined with j2735 spat and map messages.
  CLI_INFO_MAP, // not j2735 mapdata in here. it's ldm's layer 1, 2 map info message.
  CLI_INFO_BSM,
  CLI_INFO_CFM,
  CLI_INFO_CSM,
  CLI_INFO_CLC,
  CLI_INFO_PVD,
  CLI_INFO_RSA,
  CLI_INFO_EVA,
  CLI_INFO_RCP,
  CLI_INFO_VID,
  CLI_INFO_ETC,
  CLI_INFO_CAV,
  CLI_INFO_MAX,
}_CLI_MSG_INFO;

// yhcho added // 2021.04.20
#define CLI_LOGFILE

#ifdef CLI_LOGFILE
#define SWM_LOG(str, ...)           logmem_wstring(CLI_MSG_LOG, 0, str, ##__VA_ARGS__)
#define SWM_INFO(info, str, ...)    logmem_wstring(CLI_MSG_INFO, info, str, ##__VA_ARGS__)
#define SWM_STATUS(str, ...)        logmem_wstring(CLI_MSG_COMMAND, 0, str, ##__VA_ARGS__)
#else
#define SWM_LOG(str, ...)           printf("[ERROR]" str, ##__VA_ARGS__)
#define SWM_INFO(info, str, ...)    printf("[%d]" str, info, ##__VA_ARGS__)
#define SWM_STATUS(str, ...)        printf("[STATUS]" str, ##__VA_ARGS__)
#endif

#define SWM_FUNC(str, ...)          printf("[FUNC]%s:%d " str, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define SWM_DEBUG(str, ...)          printf("[DEBUG]" str, ##__VA_ARGS__)

typedef uint8_t             u8;
typedef uint16_t            u16;
typedef uint32_t            u32;
typedef uint64_t            u64;

typedef int8_t          	s8;
typedef int16_t       		s16;
typedef int32_t      		s32;
typedef int64_t         	s64;

#define TRUE	1
#define FALSE	0

#define BITSET(a,b) (a |= (1<<b))
#define BITCLR(a,b) (a &= ~(1<<b))
#define BITGET(a,b) (a & (1<<b))

#define htonll(x)   ((((u64)htonl(x)) << 32) + htonl((u32)((u64)x >> 32)))
#define ntohll(x)   ((((u64)ntohl(x)) << 32) + ntohl((u32)((u64)x >> 32)))

#if defined(SWM_LITTEL_ENDIAN)
#define htondouble(x)  reverse_double(x)
#define ntohdouble(x)  reverse_double(x)
#else   // SWM_LITTEL_ENDIAN
#define htondouble(x)  x
#define ntohdouble(x)  x
#endif  // SWM_LITTEL_ENDIAN

#ifdef __cplusplus
}
#endif

#endif /* DEFINE_H_ */
