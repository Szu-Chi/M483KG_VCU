/*
 * SYS_Init.h
 *
 *  Created on: 2020¦~1¤ë17¤é
 *      Author: Eric
 */

#ifndef STDDRIVER_INC_SYS_INIT_H_
#define STDDRIVER_INC_SYS_INIT_H_

#include <stdio.h>
#include "NuMicro.h"
#include "RMS_PM100DX.h"
#include "UART0.h"
#include "CANBUS.h"

void SYS_Init();
void TIMER0_Init(uint8_t freq);
void TIMER1_Init(uint8_t freq);

#endif /* STDDRIVER_INC_SYS_INIT_H_ */
