/*
 * RMS_PM100DX.c
 *
 *  Created on: 2019/12/4
 *      Author: Eric
 */
#include "RMS_PM100DX.h"

volatile uint16_t APPSspeed;

double decodingRMSTemperature(uint8_t byteHi, uint8_t byteLo){
    double Celsius = (byteHi << 8) | (byteLo);
    return Celsius/10.0;
}

double decodingRMSLowVoltage(uint8_t byteHi, uint8_t byteLo){
    double vols = (byteHi << 8) | (byteLo);
    return vols/100.0;
}

double decodingRMSTorque(uint8_t byteHi, uint8_t byteLo){
    double N_m = (byteHi << 8) | (byteLo);
    return N_m/10.0;
}

double decodingRMSHighVoltage(uint8_t byteHi, uint8_t byteLo){
    double vols = (byteHi << 8) | (byteLo);
    return vols/10.0;
}

double decodingRMSCurrent(uint8_t byteHi, uint8_t byteLo){
    double amps = (byteHi << 8) | (byteLo);
    return amps/10.0;
}

double decodingRMSAngle(uint8_t byteHi, uint8_t byteLo){
    double degrees = (byteHi << 8) | (byteLo);
    return degrees/10.0;
}

int16_t decodingRMSSpeed(uint8_t byteHi, uint8_t byteLo){
    int16_t rpm = (byteHi << 8) | (byteLo);
    return rpm;
}

double decodingRMSFrequency(uint8_t byteHi, uint8_t byteLo){
    double Hz = (byteHi << 8) | (byteLo);
    return Hz/10.0;
}

double decodingRMSPower(uint8_t byteHi, uint8_t byteLo){
    double kW = (byteHi << 8) | (byteLo);
    return kW/10.0;
}

double decodingRMSFlux(uint8_t byteHi, uint8_t byteLo){
    double Webers = (byteHi << 8) | (byteLo);
    return Webers/1000.0;
}

RMS_inverterState decodingRMSInverterState(uint8_t* MsgData){
    RMS_inverterState inverter;
    int state = *(MsgData + 2);
    switch(state){
    case 0:
        strcpy(inverter.state,"Power on");
        break;
    case 1:
        strcpy(inverter.state,"Stop");
        break;
    case 2:
        strcpy(inverter.state,"Open Loop");
        break;
    case 3:
        strcpy(inverter.state,"Closed Loop");
        break;
    case 4:
        strcpy(inverter.state,"Wait");
        break;
    case 5:
    case 6:
    case 7:
        strcpy(inverter.state,"Internal");
        break;
    case 8:
        strcpy(inverter.state,"Idle Run");
        break;
    case 9:
        strcpy(inverter.state,"Idle Stop");
        break;
    case 10:
    case 11:
    case 12:
        strcpy(inverter.state,"Internal");
        break;
    default:
        break;
    }

    inverter.runMode = MsgData[4]&1;
    switch((MsgData[4]&0xE0) >> 4){
    case 0:
        strcpy(inverter.activeDischarge, "Discharge Disabled");
        break;
    case 1:
        strcpy(inverter.activeDischarge, "Discharge Enabled, waiting");
        break;
    case 2:
        strcpy(inverter.activeDischarge, "Performing Speed Check");
        break;
    case 3:
        strcpy(inverter.activeDischarge, "Discharge Actively occurring");
        break;
    case 4:
        strcpy(inverter.activeDischarge, "Discharge Completed");
        break;
    default:
        break;
    }
    inverter.cmdMode = MsgData[5];
    inverter.enableState = MsgData[6]&1;
    inverter.enableLockout = (MsgData[6]&0x80)>>7;
    return inverter;
}

char* decodingRMSVSM(uint8_t byteHi, uint8_t byteLo){
    int code = (byteHi << 8) | (byteLo);
    switch(code){
    case 0:
        return "VSM Start";
    case 1:
        return "Pre-charge Init";
    case 2:
        return "Pre-charge Active";
    case 3:
        return "Pre-charge Complete";
    case 4:
        return "VSM Wait";
    case 5:
        return "VSM Ready";
    case 6:
        return "Motor Running";
    case 7:
        return "Blink Fault Code";
    case 14:
        return "Shutdown in Process";
    case 15:
        return "Recycle Power";
    default:
        return "";
    }
}

char* decodingRMSPOSTFault(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0){
    switch(byte0){
    case 1:
        return "Hardware Gate/Desaturation Fault";
    case 2:
        return "HW Over-current Fault";
    case 4:
        return "Accelerator Shorted";
    case 8:
        return "Accelerator Open";
    case 16:
        return "Current Sensor Low";
    case 32:
        return "Current Sensor High";
    case 64:
        return "Module Temperature Low";
    case 128:
        return "Module Temperature High";
    default:
        break;
    }

    switch(byte1){
    case 1:
        return "Control PCB Temperature Low";
    case 2:
        return "Control PCB Temperature High";
    case 4:
        return "Gate Drive PCB Temperature Low";
    case 8:
        return "Gate Drive PCB Temperature High";
    case 16:
        return "5V Sense Voltage Low";
    case 32:
        return "5V Sense Voltage High";
    case 64:
        return "12V Sense Voltage Low";
    case 128:
        return "12V Sense Voltage High";
    default:
        break;
    }

    switch(byte2){
    case 1:
        return "2.5V Sense Voltage Low";
    case 2:
        return "2.5V Sense Voltage High";
    case 4:
        return "1.5V Sense Voltage Low";
    case 8:
        return "1.5V Sense Voltage High";
    case 16:
        return "DC Bus Voltage High";
    case 32:
        return "DC Bus Voltage Low";
    case 64:
        return "Pre-charge Timeout";
    case 128:
        return "Pre-charge Voltage Failure";
    default:
        break;
    }

    switch(byte3){
    case 1:
        return "EEPROM Checksum Invalid";
    case 2:
        return "EEPROM Data Out of Range";
    case 4:
        return "EEPROM Update Required";
    case 8:
        return "Reserved";
    case 16:
        return "Reserved";
    case 32:
        return "Reserved";
    case 64:
        return "Brake Shorted";
    case 128:
        return "Brake Open";
    default:
        break;
    }
    return "";
}


char* decodingRMSRUNFault(uint8_t byte7, uint8_t byte6, uint8_t byte5, uint8_t byte4){
    switch(byte4){
    case 1:
        return "Motor Over-speed Fault";
    case 2:
        return "Over-current Fault ";
    case 4:
        return "Over-voltage Fault";
    case 8:
        return "Inverter Over-temperature Fault";
    case 16:
        return "Accelerator Input Shorted Fault";
    case 32:
        return "Accelerator Input Open Fault";
    case 64:
        return "Direction Command Fault";
    case 128:
        return "Inverter Response Time-out Fault";
    default:
        break;
    }

    switch(byte5){
    case 1:
        return "Hardware Gate/Desaturation Fault";
    case 2:
        return "Hardware Over-current Fault";
    case 4:
        return "Under-voltage Fault";
    case 8:
        return "CAN Command Message Lost Fault";
    case 16:
        return "Motor Over-temperature Fault ";
    case 32:
        return "Reserved";
    case 64:
        return "Reserved";
    case 128:
        return "Reserved";
    default:
        break;
    }

    switch(byte6){
    case 1:
        return "Brake Input Shorted Fault";
    case 2:
        return "Brake Input Open Fault";
    case 4:
        return "Module A Over-temperature Fault";
    case 8:
        return "Module B Over-temperature Fault";
    case 16:
        return "Module C Over-temperature Fault";
    case 32:
        return "PCB Over-temperature Fault";
    case 64:
        return "Gate Drive Board 1 Over-temperature Fault";
    case 128:
        return "Gate Drive Board 2 Over-temperature Fault";
    default:
        break;
    }

    switch(byte7){
    case 1:
        return "Gate Drive Board 3 Over-temperature Fault";
    case 2:
        return "Current Sensor Fault";
    case 4:
        return "Reserved";
    case 8:
        return "Reserved";
    case 16:
        return "Reserved";
    case 32:
        return "Reserved";
    case 64:
        return "Resolver Not Connected";
    case 128:
        return "Inverter Discharge Active";
    default:
        break;
    }
    return "";
}

void RMSCANDataUpdate(STR_CANMSG_T Msg){
    int i = 0;
    switch(Msg.Id - RMSCANIDOffset){
    case RMSTemperatures1:
        RMS.tempratures.phaseA = decodingRMSTemperature(Msg.Data[1],Msg.Data[0]);
        RMS.tempratures.phaseB = decodingRMSTemperature(Msg.Data[3],Msg.Data[2]);
        RMS.tempratures.phaseC = decodingRMSTemperature(Msg.Data[5],Msg.Data[4]);
        RMS.tempratures.gateDriverBoard = decodingRMSTemperature(Msg.Data[7],Msg.Data[6]);
        break;
    case RMSTemperatures2:
        RMS.tempratures.controlBoard = decodingRMSTemperature(Msg.Data[1],Msg.Data[0]);
        RMS.tempratures.RTDInput1 = decodingRMSTemperature(Msg.Data[3],Msg.Data[2]);
        RMS.tempratures.RTDInput2 = decodingRMSTemperature(Msg.Data[5],Msg.Data[4]);
        break;
    case RMSTemperatures3:
        RMS.tempratures.motor = decodingRMSTemperature(Msg.Data[5],Msg.Data[4]);
        RMS.torque.shudder = decodingRMSTorque(Msg.Data[7],Msg.Data[6]);
        break;
    case RMSAnalogInputVoltages:
        for (i = 0; i < 4; i++)
            RMS.analogIn[i] = decodingRMSLowVoltage(Msg.Data[i*2 + 1], Msg.Data[i*2]);
        break;
    case RMSDigitalInputStatus:
        for (i = 0; i < 8; i++)
            RMS.digitalIn[i] = Msg.Data[i];
        break;
    case RMSMotorPosition:
        RMS.motorPosition.motorAngle = decodingRMSAngle(Msg.Data[1],Msg.Data[0]);
        RMS.motorPosition.motorSpeed = decodingRMSSpeed(Msg.Data[3],Msg.Data[2]);
        RMS.motorPosition.electricalFrequency = decodingRMSFrequency(Msg.Data[5],Msg.Data[4]);
        RMS.motorPosition.deltaResolverFiltered = decodingRMSAngle(Msg.Data[5],Msg.Data[4]);
        break;
    case RMSCurrent:
        RMS.current.phaseA = decodingRMSCurrent(Msg.Data[1],Msg.Data[0]);
        RMS.current.phaseB = decodingRMSCurrent(Msg.Data[3],Msg.Data[2]);
        RMS.current.phaseC = decodingRMSCurrent(Msg.Data[5],Msg.Data[4]);
        RMS.current.DCBus = decodingRMSCurrent(Msg.Data[7],Msg.Data[6]);
        break;
    case RMSVoltage:
        RMS.highVolt.DCBus = decodingRMSHighVoltage(Msg.Data[1],Msg.Data[0]);
        RMS.highVolt.output = decodingRMSHighVoltage(Msg.Data[3],Msg.Data[2]);
        RMS.highVolt.phaseAB = decodingRMSHighVoltage(Msg.Data[5],Msg.Data[4]);
        RMS.highVolt.phaseBC = decodingRMSHighVoltage(Msg.Data[7],Msg.Data[6]);
        break;
    case RMSFlux:
        RMS.flux.command = decodingRMSFlux(Msg.Data[1],Msg.Data[0]);
        RMS.flux.feedback = decodingRMSFlux(Msg.Data[3],Msg.Data[2]);
        RMS.current.IdFeedback = decodingRMSCurrent(Msg.Data[5],Msg.Data[4]);
        RMS.current.IqFeedback = decodingRMSCurrent(Msg.Data[7],Msg.Data[6]);
        break;
    case RMSInternalVoltages:
        RMS.internal.voltage.Vref_1_5V = decodingRMSLowVoltage(Msg.Data[1],Msg.Data[0]);
        RMS.internal.voltage.Vref_2_5V = decodingRMSLowVoltage(Msg.Data[3],Msg.Data[2]);
        RMS.internal.voltage.Vref_5_0V = decodingRMSLowVoltage(Msg.Data[5],Msg.Data[4]);
        RMS.internal.voltage.Vref_12_0V = decodingRMSLowVoltage(Msg.Data[7],Msg.Data[6]);
        break;
    case RMSInternalStates:
        strcpy(RMS.internal.state.VSM,decodingRMSVSM(Msg.Data[1],Msg.Data[0]));
        RMS.internal.state.inverter = decodingRMSInverterState(Msg.Data);
        RMS.internal.state.relay = Msg.Data[3];
        if (Msg.Data[7]){
            strcpy(RMS.internal.state.dirCmd,"Forward");
        }else if(RMS.internal.state.inverter.enableState){
            strcpy(RMS.internal.state.dirCmd,"Reverse");
        }else{
            strcpy(RMS.internal.state.dirCmd,"Stopped");
        }
        break;
    case RMSFaultCodes:
        strcpy(RMS.faultCode.POSTFault,decodingRMSPOSTFault(Msg.Data[3],Msg.Data[2],Msg.Data[1],Msg.Data[0]));
        strcpy(RMS.faultCode.RUNFault,decodingRMSRUNFault(Msg.Data[7],Msg.Data[6],Msg.Data[5],Msg.Data[4]));
        break;
    case RMSTorqueAndTimer:
        RMS.torque.cmd = decodingRMSTorque(Msg.Data[1],Msg.Data[0]);
        RMS.torque.feedback = decodingRMSTorque(Msg.Data[3],Msg.Data[2]);
        RMS.powerOnTimer = ((Msg.Data[7] << 24)|(Msg.Data[6] << 16)|(Msg.Data[5] << 8)|(Msg.Data[4]))*0.003;
        break;
    case RMSModulationIndexAndFluxWeakening:
        RMS.modulationIndex =  ((Msg.Data[1] << 8)|(Msg.Data[0]))/100.0;
        RMS.current.fluxWeakeningOutput = decodingRMSCurrent(Msg.Data[3],Msg.Data[2]);
        RMS.current.IdCmd = decodingRMSCurrent(Msg.Data[5],Msg.Data[4]);
        RMS.current.IqCmd = decodingRMSCurrent(Msg.Data[7],Msg.Data[6]);
        break;
/* RMS not use
    case RMSFirmwareInfo:
                break;
    case RMSDiagnosticData:
                break;
*/
    default:
        break;
    }
}

void getRMSCANMessageContent(uint32_t ID, char *content){
    switch(ID - RMSCANIDOffset){
    case RMSTemperatures1:
        strcpy(content, "RMS: Temperature #1");
        break;
    case RMSTemperatures2:
        strcpy(content, "RMS: Temperature #2");
        break;
    case RMSTemperatures3:
        strcpy(content, "RMS: Temperature #3 & Torque Shudder");
        break;
    case RMSAnalogInputVoltages:
        strcpy(content, "RMS: Analog Input Voltages");
        break;
    case RMSDigitalInputStatus:
        strcpy(content, "RMS: Digital Input Status");
        break;
    case RMSMotorPosition:
        strcpy(content, "RMS: Motor Position");
        break;
    case RMSCurrent:
        strcpy(content, "RMS: Current");
        break;
    case RMSVoltage:
        strcpy(content, "RMS: Voltage");
        break;
    case RMSFlux:
        strcpy(content, "RMS: Flux");
        break;
    case RMSInternalVoltages:
        strcpy(content, "RMS: Internal Voltages");
        break;
    case RMSInternalStates:
        strcpy(content, "RMS: Internal States");
        break;
    case RMSFaultCodes:
        strcpy(content, "RMS: Fault Codes");
        break;
    case RMSTorqueAndTimer:
        strcpy(content, "RMS: TorqueAndTimerInformation");
        break;
    case RMSModulationIndexAndFluxWeakening:
        strcpy(content, "RMS: ModulationIndexAndFluxWeakening");
        break;
/* RMS not use
    case RMSFirmwareInfo:
                content = "RMS: ";
                break;
    case RMSDiagnosticData:
                content = "RMS: ";
                break;
*/
    default:
        strcpy(content, "Not definition");
        break;
    }
}

void UARTPrintRMSTemperature(){
    printf("Temperature\n");
    printf("\tPhaseA                          : %d\n",(int)RMS.tempratures.phaseA);
    printf("\tPhaseB                          : %d\n",(int)RMS.tempratures.phaseB);
    printf("\tPhaseC                          : %d\n",(int)RMS.tempratures.phaseC);
    printf("\tGate Driver Board               : %d\n",(int)RMS.tempratures.gateDriverBoard);
    printf("\tControl Board                   : %d\n",(int)RMS.tempratures.controlBoard);
    printf("\tRTD Input #1                    : %d\n",(int)RMS.tempratures.RTDInput1);
    printf("\tRTD Input #2                    : %d\n",(int)RMS.tempratures.RTDInput2);
    printf("\tMotor                           : %d\n",(int)RMS.tempratures.motor);
}

void UARTPrintRMSTorque(){
    printf("Torque\n");
    printf("\tTorque Shudder                  : %d\n",(int)RMS.torque.shudder);
    printf("\tCommanded Torque                : %d\n",(int)RMS.torque.cmd);
    printf("\tTorque Feedback                 : %d\n",(int)RMS.torque.feedback);
}

void UARTPrintRMSAnalogInVolt(){
    int i = 0;
    printf("Analog Input Voltage\n");
    for(i = 0; i < 4; i++)
       printf("\tAnalog Input #%d                 : %d\n", i+1,(int)RMS.analogIn[i]);
}

void UARTPrintRMSDigitalInStatus(){
    int i;
    printf("Digital Input Status\n");
    for(i = 0; i < 8; i++)
       printf("\tDigital Input #%d                : %d\n", i+1,RMS.digitalIn[i]);
}

void UARTPrintRMSMotorPosInfo(){
    printf("Motor Position Information\n");
    printf("\tMotor Angle                     : %d\n",(int)RMS.motorPosition.motorAngle);
    printf("\tMotor Speed                     : %d\n",(int)RMS.motorPosition.motorSpeed);
    printf("\tElectrical Output Frequency     : %d\n",(int)RMS.motorPosition.electricalFrequency);
    printf("\tDelta Resolver Filtered         : %d\n",(int)RMS.motorPosition.deltaResolverFiltered);
}

void UARTPrintRMSCurrent(){
    printf("Current Information\n");
    printf("\tPhase A                         : %d\n",(int)RMS.current.phaseA);
    printf("\tPhase B                         : %d\n",(int)RMS.current.phaseB);
    printf("\tPhase C                         : %d\n",(int)RMS.current.phaseC);
    printf("\tDC Bus                          : %d\n",(int)RMS.current.DCBus);
    printf("\tId feedback                     : %d\n",(int)RMS.current.IdFeedback);
    printf("\tIq feedback                     : %d\n",(int)RMS.current.IqFeedback);
    printf("\tFlux Weakening Output           : %d\n",(int)RMS.current.fluxWeakeningOutput);
    printf("\tId command                      : %d\n",(int)RMS.current.IdCmd);
    printf("\tIq command                      : %d\n",(int)RMS.current.IqCmd);
}

void UARTPrintRMSHighVoltInfo(){
    printf("High Voltage Information\n");
    printf("\tDC Bus                          : %d\n",(int)RMS.highVolt.DCBus);
    printf("\tOutput                          : %d\n",(int)RMS.highVolt.output);
    printf("\tPhaseAB                         : %d\n",(int)RMS.highVolt.phaseAB);
    printf("\tPhaseBC                         : %d\n",(int)RMS.highVolt.phaseBC);
}

void UARTPrintRMSFluxInfo(){
    printf("Flux Information\n");
    printf("\tFlux command                    : %d\n",(int)RMS.flux.command);
    printf("\tFlux feedback                   : %d\n",(int)RMS.flux.feedback);
}

void UARTPrintRMSInternalInfo(){
    printf("Internal\n");
    printf("\t1.5V Reference voltage          : %d\n",(int)RMS.internal.voltage.Vref_1_5V);
    printf("\t2.5V Reference voltage          : %d\n",(int)RMS.internal.voltage.Vref_2_5V);
    printf("\t5.0V Reference voltage          : %d\n",(int)RMS.internal.voltage.Vref_5_0V);
    printf("\t12V Reference voltage           : %d\n",(int)RMS.internal.voltage.Vref_12_0V);
    printf("\tVSM State                       : %s\n",RMS.internal.state.VSM);
    printf("\tInverter State                  : %s\n",RMS.internal.state.inverter.state);
    printf("\tRelay State(1 = active)         : 0x%02x\n",(int)RMS.internal.state.relay);
    printf("\tInverter RUN Mode               : %d\n",(int)RMS.internal.state.inverter.runMode);
    printf("\tInverter Active Discharge State : %s\n",RMS.internal.state.inverter.activeDischarge);
    printf("\tInverter Command Mode           : %d\n",(int)RMS.internal.state.inverter.cmdMode);
    printf("\tInverter Enable State           : %d\n",(int)RMS.internal.state.inverter.enableState);
    printf("\tInverter Enable Lockout         : %d\n",(int)RMS.internal.state.inverter.enableLockout);
    printf("\tDirection Command               : %s\n",RMS.internal.state.dirCmd);
}

void UARTPrintRMSFault(){
    printf("Fault\n");
    printf("\tPOST Fault                      : %s\n",RMS.faultCode.POSTFault);
    printf("\tRun Fault                       : %s\n",RMS.faultCode.RUNFault);
}

void UARTPrintRMSPowerOnTime(){
    printf("\nPower On Timer : %u\n",RMS.powerOnTimer);
};

void UARTPrintRMSInfo(){
    UARTPrintRMSTemperature();
    UARTPrintRMSTorque();
    UARTPrintRMSAnalogInVolt();
    UARTPrintRMSDigitalInStatus();
    UARTPrintRMSMotorPosInfo();
    UARTPrintRMSCurrent();
    UARTPrintRMSHighVoltInfo();
    UARTPrintRMSFluxInfo();
    UARTPrintRMSInternalInfo();
    UARTPrintRMSFault();
	printf("APPSData\n");
	printf("\tRPM                             : %d\n",APPSspeed);
    UARTPrintRMSPowerOnTime();
}

void RMSDisableLockout(){
    uint8_t data[8] = {0};
    CAN0send(RMSCmdTXID,data);
}

void RMSSetSpeed(uint8_t Hi, uint8_t Lo){
    if (ErrorFlag){
    	return;
    }
    if (RMS.internal.state.inverter.enableLockout)
        RMSDisableLockout();
//    uint8_t Speed_Hi = (Speed & 0xff00)>>8;
//    uint8_t Speed_Lo = Speed & 0x00ff;
    APPSspeed = (RpmHi << 8) + RpmLo;
    uint8_t Speed_Hi = Hi;
	uint8_t Speed_Lo = Lo;
    uint8_t data[8] = {0, 0, Speed_Lo, Speed_Hi, direction, 1, 0, 0};
    CAN0send(RMSCmdTXID,data);
}

void RMSFaultClear(){
    uint8_t data[8] = {20, 0, 1, 0, 0, 0, 0, 0};
    printf("FaultClearOrder Send\n");
    CAN0send(0xc1,data);
} 
