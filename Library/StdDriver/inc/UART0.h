/*
 * UART0.h
 *
 *  Created on: 2020/5/22
 *      Author: Eric
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#include <stdbool.h>
#include "NuMicro.h"
#include "RMS_PM100DX.h"
#include "SYS_Init.h"
#include "CANBUS.h"

#define RXBUFSIZE   1024

void UART_Init(void);

extern volatile uint32_t g_u32comRbytes;
extern volatile uint32_t g_u32comRhead;
extern volatile uint32_t g_u32comRtail;
extern volatile bool ModeWaitFlag;
extern volatile char mode;
extern volatile uint8_t g_u8RecData[RXBUFSIZE];

#endif 
