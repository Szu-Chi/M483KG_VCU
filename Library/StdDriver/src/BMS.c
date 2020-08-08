/*
 * BMS.c
 *
 *  Created on: 2019�~5��29��
 *      Author: User
 */


#include "BMS.h"
double decodingBMSVoltage(uint8_t MSB, uint8_t LSB){
    int16_t volt = (MSB << 8) | LSB;
    double resolution = 0.001;
    double offset = 0;
    return volt * resolution + offset;
}

double decodingBMSBatVoltage(uint8_t MSB, uint8_t LSB){
    int16_t volt = (MSB << 8) | LSB;
    double resolution = 0.1;
    double offset = 0;
    return volt * resolution + offset;
}

double decodingBMSCurrent(uint8_t MSB, uint8_t LSB){
    int16_t curr = (MSB << 8) | LSB;
    double resolution = 0.1;
    double offset = -1000;
    return curr * resolution + offset;
}

int decodingBMSTemperature(uint8_t data){
    int temperature = data;
    int resolution = 1;
    int offset = -40;
    return temperature * resolution + offset;
}

int decodingBMSCurrentLimit(uint8_t MSB, uint8_t LSB){
    int curr = (MSB << 8) | LSB;
    int resolution = 1;
    int offset = 0;
    return curr * resolution + offset;
}

double decodingBMSPowerLimit(uint8_t MSB, uint8_t LSB){
    double power = (MSB << 8) | LSB;
    double resolution = 0.1;
    int offset = 0;
    return power * resolution + offset;
}

void BMSCANDataUpdate(STR_CANMSG_T Msg){
    int i = 0;
    uint32_t cellNum;
    switch(Msg.Id){
    case BMS_HCU_INFO_ID:
        BMS.HCU_INFO.batVoltage = decodingBMSBatVoltage(Msg.Data[0], Msg.Data[1]);
        BMS.HCU_INFO.batCurrent = decodingBMSCurrent(Msg.Data[2], Msg.Data[3]);
        BMS.HCU_INFO.batSOC = Msg.Data[4];
        BMS.HCU_INFO.batSOH = Msg.Data[5];
        BMS.HCU_INFO.batState = (Msg.Data[6] & 0xf0) >> 4;
        BMS.HCU_INFO.batAlmLv = Msg.Data[6] &0x0f;
        BMS.HCU_INFO.batLife = Msg.Data[7];
        break;
    case BMS_HCU_MAXV_ID:
        BMS.HCU_MAXV.maxCellVolt = decodingBMSVoltage(Msg.Data[0], Msg.Data[1]);
        BMS.HCU_MAXV.minCellVolt = decodingBMSVoltage(Msg.Data[2], Msg.Data[3]);
        BMS.HCU_MAXV.maxCellVoltNo = Msg.Data[4];
        BMS.HCU_MAXV.minCellVoltNo = Msg.Data[5];
        break;
    case BMS_HCU_MAXT_ID:
        BMS.HCU_MAXT.maxTemp = decodingBMSTemperature(Msg.Data[0]);
        BMS.HCU_MAXT.minTemp = decodingBMSTemperature(Msg.Data[1]);
        BMS.HCU_MAXT.maxTempNo = Msg.Data[2];
        BMS.HCU_MAXT.minTempNo = Msg.Data[3];
        BMS.HCU_MAXT.coolingCtl = Msg.Data[4];
        BMS.HCU_MAXT.heatingCtl = Msg.Data[5];
        break;
    case BMS_HCU_RELAY_ID:
        BMS.HCU_RELAY.posRlyStr = (Msg.Data[0]&0xC0)?1:0;
        BMS.HCU_RELAY.negRlyStr = (Msg.Data[0]&0x30)?1:0;
        BMS.HCU_RELAY.preRlyStr = (Msg.Data[0]&0x0C)?1:0;
        BMS.HCU_RELAY.onChrRlyStr = (Msg.Data[0]&0x03)?1:0;
        BMS.HCU_RELAY.offChrRlyStr = (Msg.Data[1]&0xC0)?1:0;
        BMS.HCU_RELAY.chrState = Msg.Data[2] & 0xF0;
        BMS.HCU_RELAY.chrCommunication = Msg.Data[2] & 0x08;
        BMS.HCU_RELAY.chrOffCC2 = Msg.Data[2] & 0x04;
        BMS.HCU_RELAY.chrOnCC = Msg.Data[2] & 0x03;
        break;

    case BMS_HCU_POWER_ID:
        BMS.HCU_POWER.maxChrCurtLimit = decodingBMSCurrentLimit(Msg.Data[0], Msg.Data[1]);
        BMS.HCU_POWER.maxDisCurtLimit = decodingBMSCurrentLimit(Msg.Data[2], Msg.Data[3]);
        BMS.HCU_POWER.maxChrPowerLimit = decodingBMSPowerLimit(Msg.Data[4], Msg.Data[5]);
        BMS.HCU_POWER.maxDisPowerLimit = decodingBMSPowerLimit(Msg.Data[6], Msg.Data[7]);
        break;

    case BMS_HCU_ALARM_ID:
        BMS.HCU_ALARM.ALM_CELL_OV      = (Msg.Data[0]&0XC0) >> 6;
        BMS.HCU_ALARM.ALM_CELL_UV      = (Msg.Data[0]&0X30) >> 4;
        BMS.HCU_ALARM.ALM_CELL_OT      = (Msg.Data[0]&0X0C) >> 2;
        BMS.HCU_ALARM.ALM_CELL_UT      = (Msg.Data[0]&0X03) >> 0;
        
        BMS.HCU_ALARM.ALM_CELL_LBK     = (Msg.Data[1]&0X80) >> 7;
        BMS.HCU_ALARM.ALM_CELL_TBK     = (Msg.Data[1]&0X40) >> 6; 
        BMS.HCU_ALARM.ALM_BATT_DV      = (Msg.Data[1]&0X30) >> 4; 
        BMS.HCU_ALARM.ALM_BATT_DT      = (Msg.Data[1]&0X0C) >> 2; 
        BMS.HCU_ALARM.ALM_BATT_OV      = (Msg.Data[1]&0X03) >> 0;
        
        BMS.HCU_ALARM.ALM_BATT_UV      = (Msg.Data[2]&0XC0) >> 6;        
        BMS.HCU_ALARM.ALM_BATT_OC      = (Msg.Data[2]&0X30) >> 4; 
        BMS.HCU_ALARM.ALM_BATT_UC      = (Msg.Data[2]&0X0C) >> 2; 
        BMS.HCU_ALARM.ALM_CHRG_OCS     = (Msg.Data[2]&0X03) >> 0; 
        
        BMS.HCU_ALARM.ALM_DSCH_OCS     = (Msg.Data[3]&0XC0) >> 6; 
        BMS.HCU_ALARM.ALM_CHRG_OCT     = (Msg.Data[3]&0X30) >> 4; 
        BMS.HCU_ALARM.ALM_DSCH_OCT     = (Msg.Data[3]&0X0C) >> 2; 
        BMS.HCU_ALARM.ALM_BSU_OFFLINE  = (Msg.Data[3]&0X03) >> 0; 
        
        BMS.HCU_ALARM.ALM_BSU_FAULT    = (Msg.Data[4]&0X80) >> 7; 
        BMS.HCU_ALARM.ALM_LEAK_OC      = (Msg.Data[4]&0X60) >> 5; 
        BMS.HCU_ALARM.ALM_PRECHRG_FAIL = (Msg.Data[4]&0X10) >> 4;
        BMS.HCU_ALARM.ALM_AUX_FAIL     = (Msg.Data[4]&0X08) >> 3; 
        BMS.HCU_ALARM.ALM_BMU_FAIL     = (Msg.Data[4]&0X04) >> 2;        
        BMS.HCU_ALARM.ALM_VCU_OFFLINE  = (Msg.Data[4]&0X03) >> 0;
        
        BMS.HCU_ALARM.ALM_HVREL_FAIL   = (Msg.Data[5]&0X80) >> 7;
        BMS.HCU_ALARM.ALM_HALL_BREAK   = (Msg.Data[5]&0X40) >> 6;
        break;

    default:

        if(Msg.Id >= BMS_HCU_CELLV_ID && Msg.Id <= 0x184F50F3){
            cellNum = (Msg.Id - BMS_HCU_CELLV_ID) >> 16;
            for(i = 0; i < 8; i+=2)
                BMS.cell[cellNum + i/2 + 1].voltage = decodingBMSVoltage(Msg.Data[i],Msg.Data[i + 1]);
            break;
        }

        if(Msg.Id >= BMS_HCU_CELLT_ID && Msg.Id <= 0x188050F3){
            cellNum = (Msg.Id - BMS_HCU_CELLT_ID) >> 16;
            for(i = 0; i < 8; i++)
                BMS.cell[cellNum + i + 1].temperature = decodingBMSTemperature(Msg.Data[i]);
            break;
        }
        break;
    }
}
