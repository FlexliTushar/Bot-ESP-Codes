/*
 * SMS_NonBlocking.h
 * Non-blocking extension for SMS servo library
 * Extends SMS class with non-blocking communication methods
 */

#ifndef SMS_NON_BLOCKING_H
#define SMS_NON_BLOCKING_H

#include <SMS.h>

class SMS_NonBlocking : public SMS {
public:
    SMS_NonBlocking();
    
    // Non-blocking versions of commands
    int WritePosEx_Start(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);
    int WritePos_Start(uint8_t ID, int16_t Position);
    int ReadPos_Start(uint8_t ID);
    int EnableTorque_Start(uint8_t ID, uint8_t Enable);
    
    // Poll for completion - returns: 0 = in progress, 1 = success, -1 = timeout/error
    int Poll();
    
    // Check if operation is complete
    bool isComplete();
    
    // Get result of last read operation
    int getResult();
    
    // Reset state machine
    void reset();
    
private:
    unsigned long startTime;
    unsigned long timeout;
    bool operationInProgress;
    int cachedResult;
    uint8_t currentID;
    uint16_t currentRegAddr;
    bool isReadOperation;
};

#endif
