/*
 * CAN.h
 *
 *  Created on: 2019�~12��4��
 *      Author: Eric
 */

#ifndef CANBUS_H_
#define CANBUS_H_
#include <stdio.h>
#include <stdbool.h>
#include "NuMicro.h"
#include "RMS_PM100DX.h"
#include "SYS_Init.h"
#include "UART0.h"
#include "BMS.h"

void CAN0_Init(uint32_t Bit_Rate);
void CAN0send(uint32_t ID,uint8_t data[8]);

extern STR_CANMSG_T CANRXMSG;
extern STR_CANMSG_T CANTXMSG;
extern volatile uint8_t RpmHi;
extern volatile uint8_t RpmLo;
extern volatile bool ErrorFlag;

#endif /* CANBUS_H_ */
