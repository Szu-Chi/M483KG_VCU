/*
 * CAN.c
 *
 *  Created on: 2019/12/4
 *      Author: Eric
 */
#include "CANBUS.h"

STR_CANMSG_T CANRXMSG;
STR_CANMSG_T CANTXMSG;
volatile uint8_t RpmHi;
volatile uint8_t RpmLo;
volatile bool ErrorFlag = false;

void CAN0_IRQHandler(void)
{

    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000) /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /* Status Change interrupt*/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk; /* Clear Rx Ok status*/
//            printf("RX OK INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk; /* Clear Tx Ok status*/
//            printf("TX OK INT\n") ;
        }

        /* Error Status interrupt */
        if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;

            /* Do Init to release busoff pin */
            CAN0->CON = (CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);
            CAN0->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            while(CAN0->CON & CAN_CON_INIT_Msk);
        }

        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;
        }
    }
    else if (u8IIDRstatus != 0)
    {
    	uint8_t Msg_Obj = u8IIDRstatus - 1;
    	CAN_ReadMsgObj(CAN0,Msg_Obj,TRUE,&CANRXMSG);
    	if (Msg_Obj == 0){
    		uint16_t sum;
    		uint8_t Checksum;
    		uint8_t APPSHi;
    		uint8_t APPSLo;
    		APPSLo = CANRXMSG.Data[0];
    		APPSHi = CANRXMSG.Data[1];
    		sum = APPSLo + (APPSHi << 8);
    		Checksum = sum / 1000 + (sum / 100 - sum / 1000 * 10) + 48 * 2;
    		/* Checksum */
    		if (Checksum == CANRXMSG.Data[2]){
    			RpmLo = CANRXMSG.Data[0];
    			RpmHi = CANRXMSG.Data[1];
    		}
    		else{
    			RpmLo = 0;
    			RpmHi = 0;
    			printf("RunCode Checksum: %d, APPS Checksum: %d",Checksum, CANRXMSG.Data[2]);
    			printf("Checksum Error\n");
    		}
    	}
    	if (RMSBoardcastRXOBJIDFirst <= Msg_Obj <= RMSBoardcastRXOBJIDLast){
    	    RMSCANDataUpdate(CANRXMSG);
    	    if (Msg_Obj == 15){
    	    	bool PreErrorFlag = false;
    	    	uint8_t i;
    	    	for (i=0;i<8;i++){
    	    		if (CANRXMSG.Data[i] != 0)
    	    			PreErrorFlag = true;
    	    		ErrorFlag = PreErrorFlag;
    	    	}
    	    }
        }
    	if (Msg_Obj == 17){
    		if(CANRXMSG.Data[2] && (CANRXMSG.Data[0] == 20))
    			printf("Fault Clear Success\n");
    	}

    	if (BMSBoardcastRXOBJIDFirst <= Msg_Obj <= BMSBoardcastRXOBJIDLast){
    	    BMSCANDataUpdate(CANRXMSG);
    	}
    	if (BMSCellRXOBJIDFirst <= Msg_Obj <= BMSCellRXOBJIDLast){
			BMSCANDataUpdate(CANRXMSG);
		}
        CAN_CLR_INT_PENDING_BIT(CAN0, ((CAN0->IIDR) -1)); /* Clear Interrupt Pending */
    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0; /* Write '0' to clear */
    }
}

void CAN0_Init(uint32_t Bit_Rate)
{
    /* Enable IP clock */
	CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;

    /* Set PA multi-function pins for CAN0 RXD(PA.4) and TXD(PA.5) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk)) |
                    (SYS_GPA_MFPL_PA4MFP_CAN0_RXD | SYS_GPA_MFPL_PA5MFP_CAN0_TXD);

    /*Set CAN operation mode and target baud-rate*/
    CAN_Open(CAN0, Bit_Rate, CAN_NORMAL_MODE);

    /*The function is used to configure a receive message object*/

	/* APPS Signal */
	CAN_SetRxMsg(CAN0, 0,CAN_STD_ID, 0x0D0);

    /* Controller Status parameters */
	int i;
	for(i = RMSBoardcastRXOBJIDFirst; i < RMSBoardcastRXOBJIDLast; i++)
		CAN_SetRxMsgAndMsk(CAN0, i,CAN_STD_ID, RMSBroadcastRXID, RMSBoradcastMask);

	/* FaultClear Feedback Signal*/
	CAN_SetRxMsg(CAN0, 17,CAN_STD_ID, 0x0C2);

	/* BMS Status parameters */
	for(i = BMSBoardcastRXOBJIDFirst; i < BMSBoardcastRXOBJIDLast; i++)
		CAN_SetRxMsgAndMsk(CAN0, i,CAN_EXT_ID, BMS_HCU_INFO_ID, BMS_HCU_INFOID_MASK);
	for(i = BMSCellRXOBJIDFirst; i < BMSCellRXOBJIDLast; i++)
		CAN_SetRxMsgAndMsk(CAN0, i,CAN_EXT_ID, BMS_HCU_CELLV_ID, BMS_HCU_CELLID_MASK);


    /* CAN interrupt enabled */
    CAN_EnableInt(CAN0, CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk);
    NVIC_SetPriority(CAN0_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN0_IRQn);
}

void CAN0send(uint32_t ID,uint8_t data[8])
{
	uint8_t i;
    CANTXMSG.FrameType= CAN_DATA_FRAME;
    CANTXMSG.IdType   = CAN_STD_ID;
    CANTXMSG.Id       = ID;
    CANTXMSG.DLC      = 8;
    for(i=0;i<8;i++)
    	*(CANTXMSG.Data+i)=*(data+i);
    CAN_Transmit(CAN0,CAN0TXOBJID, &CANTXMSG);
}

