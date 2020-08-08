#ifndef RMS_PM100DX_H_
#define RMS_PM100DX_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "NuMicro.h"
#include "SYS_Init.h"
#include "UART0.h"
#include "CANBUS.h"

#define CAN0TXOBJID     1

#define RMSCmdTXID    0X00C0

#define RMSBroadcastRXID    0X00A0
#define RMSBoradcastMask    0xFFF0
#define RMSBoardcastRXOBJIDFirst 1
#define RMSBoardcastRXOBJIDLast  17

#define RMSParameterTXID    0X00C1
#define RMSParameterMask    0xFFFF

#define RMSParameterRXID    0X00C2
#define RMSParameterMask    0xFFFF
#define RMSParameterRXOBJIDFirst 11
#define RMSParameterRXOBJIDLast  12

#define RMSCANIDOffset 0x0A0
#define RMSTemperatures1 0x000
#define RMSTemperatures2 0x001
#define RMSTemperatures3 0x002
#define RMSAnalogInputVoltages 0x003
#define RMSDigitalInputStatus 0x004
#define RMSMotorPosition 0x005
#define RMSCurrent 0x006
#define RMSVoltage 0x007
#define RMSFlux 0x008
#define RMSInternalVoltages 0x009
#define RMSInternalStates  0x00A
#define RMSFaultCodes 0x00B
#define RMSTorqueAndTimer 0x00C
#define RMSModulationIndexAndFluxWeakening 0x00D
#define RMSFirmwareInfo 0x00E
#define RMSDiagnosticData 0x00F

#define direction 1 // Forward:1, Backward:0

typedef struct RMSTemperatures{
    double phaseA;
    double phaseB;
    double phaseC;
    double gateDriverBoard;
    double controlBoard;
    double RTDInput1;
    double RTDInput2;
    double motor;
}RMS_temperatures;

typedef struct RMSTorque{
    double shudder;
    double cmd;// commanded torque
    double feedback;
}RMS_torque;

typedef struct RMSMotorPositionInfo{
    double motorAngle;
    int16_t motorSpeed;
    double electricalFrequency;
    double deltaResolverFiltered;
}RMS_motorPosition;

typedef struct RMSCurrentInfo{
    double phaseA;
    double phaseB;
    double phaseC;
    double DCBus;
    double IdFeedback;
    double IqFeedback;
    double IdCmd;
    double IqCmd;
    double fluxWeakeningOutput;
}RMS_current;

typedef struct RMSHighVoltage{
    double DCBus;
    double output;
    double phaseAB;
    double phaseBC;
}RMS_highVoltage;

typedef struct RMSInternalVoltage{
    double Vref_1_5V;//1.5V
    double Vref_2_5V;//2.5V
    double Vref_5_0V;//5.0V
    double Vref_12_0V;//12V
}RMS_internalVoltage;

typedef struct RMSFluxInfo{
    double command;
    double feedback;
}RMS_flux;

typedef struct RMSInverterState{
    char  state[15];
    uint8_t  runMode;
    char  activeDischarge[35];
    uint8_t  cmdMode;
    uint8_t  enableState;
    uint8_t  enableLockout;
}RMS_inverterState;

typedef struct RMSInternalState{
    char VSM[35];
    int8_t  relay;
    RMS_inverterState inverter;
    char dirCmd[10];// Direction Command
}RMS_internalState;

typedef struct RMSInternal{
    RMS_internalState state;
    RMS_internalVoltage voltage;
}RMS_internal;

typedef struct RMSFaultCode{
    char POSTFault[45];
    char RUNFault[45];
}RMS_faultCode;

typedef struct RMSMotorControllor{
    RMS_temperatures tempratures;
    RMS_torque   torque;
    double analogIn[4];
    uint8_t digitalIn[8];
    RMS_motorPosition motorPosition;
    RMS_current current;
    RMS_highVoltage highVolt;
    RMS_flux flux;
    RMS_internal internal;
    RMS_faultCode faultCode;
    uint32_t powerOnTimer;
    double modulationIndex;
}RMS_motorcontrollor;

RMS_motorcontrollor RMS;

double decodingRMSTemperature(uint8_t byteHi, uint8_t byteLo);
double decodingRMSLowVoltage(uint8_t byteHi, uint8_t byteLo);
double decodingRMSTorque(uint8_t byteHi, uint8_t byteLo);
double decodingRMSHighVoltage(uint8_t byteHi, uint8_t byteLo);
double decodingRMSCurrent(uint8_t byteHi, uint8_t byteLo);
double decodingRMSAngle(uint8_t byteHi, uint8_t byteLo);
int16_t decodingRMSSpeed(uint8_t byteHi, uint8_t byteLo);
double decodingRMSFreqence(uint8_t byteHi, uint8_t byteLo);
double decodingRMSPower(uint8_t byteHi, uint8_t byteLo);
double decodingRMSFlux(uint8_t byteHi, uint8_t byteLo);

RMS_inverterState decodingRMSInverterState(uint8_t* MsgData);
char* decodingRMSVSM(uint8_t byteHi, uint8_t byteLo);
char* decodingRMSPOSTFault(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0);
char* decodingRMSRunFault(uint8_t byte7, uint8_t byte6, uint8_t byte5, uint8_t byte4);
void RMSCANDataUpdate(STR_CANMSG_T Msg);
void getRMSCANMessageContent(uint32_t ID, char *content);
void UARTPrintRMSTemperature();
void UARTPrintRMSTorque();
void UARTPrintRMSAnalogInVolt();
void UARTPrintRMSDigitalInStatus();
void UARTPrintRMSMotorPosInfo();
void UARTPrintRMSCurrent();
void UARTPrintRMSHighVoltInfo();
void UARTPrintRMSFluxInfo();
void UARTPrintRMSInternalInfo();
void UARTPrintRMSFault();
void UARTPrintRMSPowerOnTime();
void UARTPrintRMSInfo();

void RMSDisableLockout();
void RMSSetTorque(int16_t Nm);
void RMSSetSpeed(uint8_t Hi, uint8_t Lo);
void RMSRelayCmd(uint8_t r);
void RMSFaultClear();

extern volatile uint16_t APPSspeed;
#endif
