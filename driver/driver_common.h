#pragma once
#include <stdint.h>

//==============================================
// { UART } Configuration and Status Structures
//==============================================

enum class Uart_Parity: uint8_t{
    NONE = 0,
    EVEN = 1,
    ODD = 2
};

enum class Uart_StopBit: uint8_t{
    ONE = 1,
    TWO = 2
};

struct Uart_Config{
    uint32_t baud;
    Uart_Parity parity;
    Uart_StopBit stopBit;
};

struct Uart_Status{
    bool isConnected;
    bool hasError;
    bool rxComplete;
    bool txComplete;
    uint8_t errorType;
};

struct Uart_Instance{
    Uart_Status status;
};


//================================================
// { MCADCPWM3P } Configuration and Status Structures
//================================================
struct MCADCPWM3P_Config{
    uint32_t pwmFreq_Hz = 10000;
    uint32_t deadtime_nS = 1500;
    float dutyMax_perc = 95;
    float dutyMin_perc = 5;
    bool breakEnabled = false;
};

struct MCADCPWM3P_Instance{
    int16_t timerMax_q15 = 0;
    int16_t dutyMax_q15 = 0;
    int16_t dutyMin_q15 = 0;
};

enum class MCADCPWM3P_PwmScanMode{
    TRAP_SECTOR1 = 0,
    TRAP_SECTOR2 = 0,
    TRAP_SECTOR3 = 0,
    TRAP_SECTOR4 = 0,
    TRAP_SECTOR5 = 0,
    TRAP_SECTOR6 = 0,

    SVM_SECTOR1 = 0,
    SVM_SECTOR2 = 0,
    SVM_SECTOR3 = 0,
    SVM_SECTOR4 = 0,
    SVM_SECTOR5 = 0,
    SVM_SECTOR6 = 0,
};

#define CS_NONE 0
#define CS_SINGLE_SHUNT 1
#define CS_DOUBLE_SHUNT 2
#define CS_TRIPLE_SHUNT 3
#define CS_INLINE 4

//=======================================================
// { Hall and BEMFZcd } Configuration and Status Structures
//=====================================================
struct Hall_Config{
    //Hysteresis voltage for zero cross detection in BEMFZc  
    uint32_t hysteresisVoltage_mV;
    uint32_t commutationDelay_DegElec;
    uint32_t minimumElecVelocity_DegpS;
};


