/*
 * termios.h
 *
 *  Created on: Mar 8, 2021
 *      Author: yhcho
 */

#ifndef SWM_TERMIOS_H_
#define SWM_TERMIOS_H_

#ifdef SWM_TERMIOS_C_
#define SWM_TERMIOS_EXTERN
#else
#define SWM_TERMIOS_EXTERN extern
#endif

#include "swm_define.h"


FILE* openSerialPort(const char* portName);

#endif /* SWM_TERMIOS_H_ */
