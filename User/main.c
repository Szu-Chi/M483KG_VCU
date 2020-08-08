/*
 * main.c
 *
 *  Created on: 2020/1/17
 *      Author: Eric
 */
#include <stdio.h>
#include <ctype.h>
#include "NuMicro.h"
#include "SYS_Init.h"
#include "RMS_PM100DX.h"
#include "CANBUS.h"
#include "UART0.h"

void TMR0_IRQHandler(void)
{
    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);
    /* Speed mode */
    RMSSetSpeed(RpmHi,RpmLo);
}

int main(){
    SYS_Init();
    TIMER0_Init(20);
    CAN0_Init(500000);
    UART_Init();
	while (1){
		/* Speed Mode */
		printf("Data(d) Fault Clear(c):\n");
		while(ModeWaitFlag);
		if(mode == 'd' || mode == 'c'){
			if(mode == 'd')
				UARTPrintRMSInfo();
			if(mode == 'c')
				RMSFaultClear();
		}
		else{
			printf("\nInput Error\n");
		}
		ModeWaitFlag = true;
	}
	return 0;
}
