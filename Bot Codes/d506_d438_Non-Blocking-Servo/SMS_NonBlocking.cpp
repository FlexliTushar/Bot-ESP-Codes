/*
 * SMS_NonBlocking.cpp
 * Non-blocking implementation for SMS servo library
 */

#include "SMS_NonBlocking.h"

SMS_NonBlocking::SMS_NonBlocking() : SMS()
{
    operationInProgress = false;
    timeout = 100;
    cachedResult = 0;
}

int SMS_NonBlocking::WritePosEx_Start(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    if (operationInProgress) {
        return -2; // Busy
    }
    
    uint16_t wDat[4];
    wDat[0] = Position;
    wDat[1] = 1;
    wDat[2] = ACC;
    wDat[3] = Speed;
    
    // Setup telegram
    telegram.u8id = ID;
    telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;
    telegram.u16RegAdd = SMS_GOAL_POSITION;
    telegram.u16CoilsNo = 4;
    telegram.au16reg = wDat;
    
    // Start the query without blocking
    query();
    
    if (u8lastError) {
        return 0; // Error starting
    }
    
    operationInProgress = true;
    startTime = millis();
    isReadOperation = false;
    currentID = ID;
    
    return 1; // Started successfully
}

int SMS_NonBlocking::WritePos_Start(uint8_t ID, int16_t Position)
{
    if (operationInProgress) {
        return -2; // Busy
    }
    
    telegram.u8id = ID;
    telegram.u8fct = MB_FC_WRITE_REGISTER;
    telegram.u16RegAdd = SMS_GOAL_POSITION;
    telegram.u16CoilsNo = 1;
    telegram.au16reg = (uint16_t*)&Position;
    
    query();
    
    if (u8lastError) {
        return 0;
    }
    
    operationInProgress = true;
    startTime = millis();
    isReadOperation = false;
    currentID = ID;
    
    return 1;
}

int SMS_NonBlocking::ReadPos_Start(uint8_t ID)
{
    if (operationInProgress) {
        return -2; // Busy
    }
    
    telegram.u8id = ID;
    telegram.u8fct = MB_FC_READ_REGISTERS;
    telegram.u16RegAdd = SMS_PRESENT_POSITION;
    telegram.u16CoilsNo = 1;
    telegram.au16reg = (uint16_t*)&cachedResult;
    
    query();
    
    if (u8lastError) {
        return 0;
    }
    
    operationInProgress = true;
    startTime = millis();
    isReadOperation = true;
    currentID = ID;
    currentRegAddr = SMS_PRESENT_POSITION;
    
    return 1;
}

int SMS_NonBlocking::EnableTorque_Start(uint8_t ID, uint8_t Enable)
{
    if (operationInProgress) {
        return -2; // Busy
    }
    
    uint16_t enableVal = Enable;
    telegram.u8id = ID;
    telegram.u8fct = MB_FC_WRITE_REGISTER;
    telegram.u16RegAdd = SMS_TORQUE_ENABLE;
    telegram.u16CoilsNo = 1;
    telegram.au16reg = &enableVal;
    
    query();
    
    if (u8lastError) {
        return 0;
    }
    
    operationInProgress = true;
    startTime = millis();
    isReadOperation = false;
    currentID = ID;
    
    return 1;
}

// Non-blocking poll - call this repeatedly in loop
// Returns: 0 = in progress, 1 = success, -1 = timeout/error
int SMS_NonBlocking::Poll()
{
    if (!operationInProgress) {
        return 1; // Already complete
    }
    
    // Check timeout
    if ((millis() - startTime) > timeout) {
        operationInProgress = false;
        u8lastError = ERR_NO_REPLY;
        return -1; // Timeout
    }
    
    // Check if state is still waiting
    if (u8state == COM_WAITING) {
        // Call poll once to process any incoming data
        poll();
        
        // Still waiting?
        if (u8state == COM_WAITING) {
            return 0; // In progress
        }
    }
    
    // Operation completed
    operationInProgress = false;
    
    if (u8lastError) {
        return -1; // Error
    }
    
    // Success
    return 1;
}

bool SMS_NonBlocking::isComplete()
{
    return !operationInProgress;
}

int SMS_NonBlocking::getResult()
{
    return cachedResult;
}

void SMS_NonBlocking::reset()
{
    operationInProgress = false;
    u8state = COM_IDLE;
}
