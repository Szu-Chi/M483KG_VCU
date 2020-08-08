/*
 * BMS.h
 *
 *  Created on: 2019¦~5¤ë18¤é
 *      Author: User
 */

#ifndef BMS_H_
#define BMS_H_
#include <stdint.h>
#include "CANBUS.h"
#include "UART0.h"

#define BMSBoardcastRXOBJIDFirst 18
#define BMSBoardcastRXOBJIDLast  25

#define BMSCellRXOBJIDFirst 26
#define BMSCellRXOBJIDLast  27

#define BMS_HCU_INFO_ID      0x186040F3
#define BMS_HCU_MAXV_ID      0x186140F3
#define BMS_HCU_MAXT_ID      0x186240F3
#define BMS_HCU_RELAY_ID     0x186340F3
#define BMS_HCU_POWER_ID     0x186440F3
#define BMS_HCU_ALARM_ID     0x186540F3
#define BMS_HCU_INFOID_MASK  0xfff8ffff

#define BMS_HCU_CELLV_ID     0x180050F3
#define BMS_HCU_CELLT_ID     0x185050F3
#define BMS_HCU_CELLID_MASK  0xffAfffff

#define BMS_HCU_BMS_CMD_ID 0x1801F340


void UARTprintCellVol(uint8_t num);
void UARTprintCellTemp(uint8_t num);
void UARTprintBMSInfo();

typedef struct KlClear_BMS_cell{
    double voltage;
    int temperature;
}KlClear_BMS_cell;

typedef struct KlClear_BMS_HCU_INFO{
   double batVoltage;
   double batCurrent;
   double batSOC;
   double batSOH;
   uint8_t batState; 
   uint8_t batAlmLv;
   uint8_t batLife;
}KlClear_BMS_HCU_INFO;

typedef struct KlClear_BMS_HCU_MAXV{
   double maxCellVolt;
   double minCellVolt;
   uint8_t maxCellVoltNo;
   uint8_t minCellVoltNo;
}KlClear_BMS_HCU_MAXV;

typedef struct KlClear_BMS_HCU_MAXT{
   double maxTemp;
   double minTemp;
   uint8_t maxTempNo;
   uint8_t minTempNo;
   uint8_t coolingCtl;
   uint8_t heatingCtl;
}KlClear_BMS_HCU_MAXT;

typedef struct KlClear_BMS_HCU_RELAY{
   uint8_t posRlyStr;
   uint8_t negRlyStr;
   uint8_t preRlyStr;
   uint8_t onChrRlyStr;
   uint8_t offChrRlyStr;
   uint8_t chrState;
   uint8_t chrCommunication;
   uint8_t chrOffCC2;
   uint8_t chrOnCC;
   uint16_t chrReqVolt;
   uint16_t chrReqCurr;
}KlClear_BMS_HCU_RELAY;

typedef struct KlClear_BMS_HCU_POWER{
   uint16_t maxChrCurtLimit;
   uint16_t maxDisCurtLimit;
   uint16_t maxChrPowerLimit;
   uint16_t maxDisPowerLimit;
}KlClear_BMS_HCU_POWER;

typedef struct KlClear_BMS_HCU_ALARM{
   uint8_t ALM_CELL_OV;
   uint8_t ALM_CELL_UV;
   uint8_t ALM_CELL_OT;
   uint8_t ALM_CELL_UT;

   uint8_t ALM_CELL_LBK;
   uint8_t ALM_CELL_TBK;
   uint8_t ALM_BATT_DV;
   uint8_t ALM_BATT_DT;
   uint8_t ALM_BATT_OV;

   uint8_t ALM_BATT_UV;
   uint8_t ALM_BATT_OC;
   uint8_t ALM_BATT_UC;
   uint8_t ALM_CHRG_OCS;

   uint8_t ALM_DSCH_OCS;
   uint8_t ALM_CHRG_OCT;
   uint8_t ALM_DSCH_OCT;
   uint8_t ALM_BSU_OFFLINE;

   uint8_t ALM_BSU_FAULT;
   uint8_t ALM_LEAK_OC;
   uint8_t ALM_PRECHRG_FAIL;
   uint8_t ALM_AUX_FAIL;
   uint8_t ALM_BMU_FAIL;
   uint8_t ALM_VCU_OFFLINE;

   uint8_t ALM_HVREL_FAIL;
   uint8_t ALM_HALL_BREAK;

}KlClear_BMS_HCU_ALARM;

typedef struct KlClearBMS{
   KlClear_BMS_cell cell[78];
   KlClear_BMS_HCU_INFO HCU_INFO;
   KlClear_BMS_HCU_MAXV HCU_MAXV;
   KlClear_BMS_HCU_MAXT HCU_MAXT;
   KlClear_BMS_HCU_RELAY HCU_RELAY;
   KlClear_BMS_HCU_POWER HCU_POWER;
   KlClear_BMS_HCU_ALARM HCU_ALARM;
}KlClearBMS;

KlClearBMS BMS;

double decodingBMSVoltage(uint8_t MSB, uint8_t LSB);
double decodingBMSBatVoltage(uint8_t MSB, uint8_t LSB);
double decodingBMSCurrent(uint8_t MSB, uint8_t LSB);
int decodingBMSCurrentLimit(uint8_t MSB, uint8_t LSB);
void BMSCANDataUpdate(STR_CANMSG_T Msg);

#endif /* BMS_H_ */
