// Non-blocking velocity control state machine
enum VelocitySetState {
  VELOCITY_IDLE,
  VELOCITY_WRITE_TARGET,
  VELOCITY_WRITE_CONTROL
};

VelocitySetState velocityState = VELOCITY_IDLE;
unsigned long lastVelocityTime = 0;
int pendingVelocity = 0;
bool hasQueuedVelocity = false;
int queuedVelocity = 0;

// Non-blocking read state machine for getVelocityActualValueAveraged
enum ReadVelocityState {
  READ_VELOCITY_IDLE,
  READ_VELOCITY_WAITING_RESPONSE
};

ReadVelocityState readVelocityState = READ_VELOCITY_IDLE;
unsigned long readVelocityStartTime = 0;
unsigned long readVelocityResponseTime = 0;
int16_t lastVelocityReadValue = 0;
bool velocityReadComplete = false;
bool velocityReadTimeout = false;
const unsigned long READ_TIMEOUT_MS = 100;

void WriteCAN(uint16_t index, uint8_t subindex, uint32_t decimalData) {
  CAN_FRAME txFrame;
  txFrame.rtr = 0;
  txFrame.extended = false;
  txFrame.length = 8;

  txFrame.id = 0x601;
  txFrame.data.uint8[0] = 0x22;  // Write - 0x22

  // Split 16-bit index into low and high bytes
  txFrame.data.uint8[1] = index & 0xFF;         // Index (Lowebyte)
  txFrame.data.uint8[2] = (index >> 8) & 0xFF;  // Index (Highbyte)

  txFrame.data.uint8[3] = subindex;  // SubIndex

  // Convert decimal data to hexadecimal and split into bytes (assuming little-endian byte order)
  uint32_t hexData = decimalData;                  // Store decimal data for conversion
  txFrame.data.uint8[4] = hexData & 0xFF;          // Data [Byte 0]
  txFrame.data.uint8[5] = (hexData >> 8) & 0xFF;   // Data [Byte 1]
  txFrame.data.uint8[6] = (hexData >> 16) & 0xFF;  // Data [Byte 2]
  txFrame.data.uint8[7] = (hexData >> 24) & 0xFF;  // Data [Byte 3]

  CAN0.sendFrame(txFrame);
}

void ReadCAN(uint16_t index, uint8_t subindex) {
  CAN_FRAME txFrame;
  txFrame.rtr = 0;
  txFrame.extended = false;
  txFrame.length = 8;

  // Read
  txFrame.id = 0x601;
  txFrame.data.uint8[0] = 0x40;                 // Read - 0x40 / Write - 0x22
  txFrame.data.uint8[1] = index & 0xFF;         // Index (Lower byte)
  txFrame.data.uint8[2] = (index >> 8) & 0xFF;  // Index (Higher byte)
  txFrame.data.uint8[3] = subindex;             // SubIndex
  txFrame.data.uint8[4] = 0x00;                 // Reserved
  txFrame.data.uint8[5] = 0x00;                 // Reserved
  txFrame.data.uint8[6] = 0x00;                 // Reserved
  txFrame.data.uint8[7] = 0x00;                 // Reserved

  CAN0.sendFrame(txFrame);
  CAN0.watchFor();  //then let everything else through anyway

  CAN_FRAME message;
  if (CAN0.read(message)) {
    // Serial.println("Message : ");
    // printFrame(&message);
  }
}

void MoveToPosition(long targetPosition, bool absolute, bool immediately) {

  /*
    EPOS4 Application Notes > Refer 7.4 Profile Position Mode
  */
  uint32_t writePosition = targetPosition;

  WriteCAN(TARGET_POSITION, 0x00, writePosition);

  if (absolute && immediately) WriteCAN(CONTROLWORD, 0x00, ABSOLUTE_POSITION_START_IMMEDIATELY);
  else if (!absolute && immediately) WriteCAN(CONTROLWORD, 0x00, RELATIVE_POSITION_START_IMMEDIATELY);
  else if (absolute && !immediately) WriteCAN(CONTROLWORD, 0x00, ABSOLUTE_POSITION);
  else if (!absolute && !immediately) WriteCAN(CONTROLWORD, 0x00, RELATIVE_POSITION);

  delay(CAN_DELAY);
  WriteCAN(CONTROLWORD, 0x00, 15);
}

void printFrame(CAN_FRAME *message) {
  Serial.print(message->id, HEX);
  Serial.print(" ");
  // if (message->extended) Serial.print(" X ");
  // else Serial.print(" S ");
  // Serial.print(message->length, DEC);
  // Serial.print(" ");
  for (int i = 0; i < message->length; i++) {
    Serial.print(message->data.byte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void haltMovement() {
  WriteCAN(CONTROLWORD, 0x00, 271);  // 0x010F - EPOS4 Application Notes PG 122
  delay(CAN_DELAY);                  // Optional delay after setting mode
}

void QuickStop() {
  WriteCAN(CONTROLWORD, 0x00, 11);  // 0x000B - EPOS4 Application Notes PG 122
  delay(CAN_DELAY);                 // Optional delay after setting mode
}

void disable() {
  WriteCAN(CONTROLWORD, 0x00, 6);  // Controlword (Shutdown)
  delay(CAN_DELAY);                // Optional delay after setting mode
}

void enable() {
  WriteCAN(CONTROLWORD, 0x00, 15);  // Controlword (Switch on & Enable)
  delay(CAN_DELAY);                 // Optional delay after setting mode
}

void setOperationMode(int mode) {
  WriteCAN(OPERATION_MODE, 0x00, mode);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

void setMaxProfileVelocity(int MaxProfileVelocity) {
  WriteCAN(MAX_PROFILE_VELOCITY, 0x00, MaxProfileVelocity);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

void setProfileVelocity(int ProfileVelocity) {
  WriteCAN(PROFILE_VELOCITY, 0x00, ProfileVelocity);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

void setProfileAcceleration(int ProfileAcceleration) {
  WriteCAN(PROFILE_ACCELERATION, 0x00, ProfileAcceleration);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

void setProfileDeceleration(int ProfileDeceleration) {
  WriteCAN(PROFILE_DECELERATION, 0x00, ProfileDeceleration);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

void setQuickStopDeceleration(int QuickStopDeceleration) {
  WriteCAN(QUICK_STOP_DECELERATION, 0x00, QuickStopDeceleration);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

void setMotionProfileType(int MotionProfileType) {
  WriteCAN(MOTION_PROFILE_TYPE, 0x00, MotionProfileType);
  delay(CAN_DELAY);  // Optional delay after setting mode
}

// Non-blocking velocity setting - call this to initiate velocity change
// If busy, queues the new velocity to be applied after current command completes
void setTargetVelocity(int TargetVelocity) {
  if (velocityState == VELOCITY_IDLE) {
    pendingVelocity = TargetVelocity;
    velocityState = VELOCITY_WRITE_TARGET;
    WriteCAN(TARGET_VELOCITY, 0x00, TargetVelocity);
    lastVelocityTime = millis();
  } else {
    // State machine is busy, queue this velocity for next cycle
    hasQueuedVelocity = true;
    queuedVelocity = TargetVelocity;
  }
}

// Backwards compatibility - keep old typo function name
void setTagetVelocity(int TagetVelocity) {
  setTargetVelocity(TagetVelocity);
  // WriteCAN(TARGET_VELOCITY, 0x00, TagetVelocity);
  // delay(CAN_DELAY);
  // WriteCAN(CONTROLWORD, 0x00, 15);
  // delay(CAN_DELAY);
}

// Update function - MUST be called in loop() for non-blocking operation
// This progresses the state machine through: IDLE -> WRITE_TARGET -> WRITE_CONTROL -> IDLE
void updateVelocityCommand() {
  unsigned long currentTime = millis();
  
  switch (velocityState) {
    case VELOCITY_WRITE_TARGET:
      if (currentTime - lastVelocityTime >= CAN_DELAY) {
        WriteCAN(CONTROLWORD, 0x00, 15);  // 0x000F Refer EPOS4 Application Notes PG123.
        lastVelocityTime = currentTime;
        velocityState = VELOCITY_WRITE_CONTROL;
      }
      break;
      
    case VELOCITY_WRITE_CONTROL:
      if (currentTime - lastVelocityTime >= CAN_DELAY) {
        velocityState = VELOCITY_IDLE;
        // Check if there's a queued velocity command
        if (hasQueuedVelocity) {
          hasQueuedVelocity = false;
          setTargetVelocity(queuedVelocity);
        }
      }
      break;
      
    case VELOCITY_IDLE:
    default:
      break;
  }
}

// Check if velocity command is still processing
bool isVelocityCommandBusy() {
  return velocityState != VELOCITY_IDLE;
}

uint8_t getModeOfOperation() {
  ReadCAN(0x6060, 0x00);
  delay(CAN_DELAY);
  uint8_t modeOfOperation = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    // Extract mode of operation from the message frame
    modeOfOperation = message.data.byte[4];
  }
  return modeOfOperation;
}

int16_t getCurrentActualValueAveraged() {
  ReadCAN(0x30D1, 0x01);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t CurrentActualValueAveraged = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    // Extract mode of operation from the message frame
    CurrentActualValueAveraged = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return CurrentActualValueAveraged;
}

int16_t getVelocityActualValueAveraged() {
  ReadCAN(0x30D3, 0x01);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t VelocityActualValueAveraged = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    // Extract mode of operation from the message frame
    VelocityActualValueAveraged = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return VelocityActualValueAveraged;
}

// Non-blocking version - call this to start reading velocity
void startReadVelocityActualValueAveraged() {
  if (readVelocityState == READ_VELOCITY_IDLE) {
    ReadCAN(0x30D3, 0x01);  // Send the read request
    readVelocityStartTime = micros();
    readVelocityState = READ_VELOCITY_WAITING_RESPONSE;
    velocityReadComplete = false;
    velocityReadTimeout = false;
  }
}

// Update function - MUST be called in loop() for non-blocking read operation
void updateReadVelocityActualValueAveraged() {
  unsigned long currentTime = micros();
  
  if (readVelocityState == READ_VELOCITY_WAITING_RESPONSE) {
    CAN_FRAME message;
    
    // Try to read response
    if (CAN0.read(message)) {
      // Successfully received response
      lastVelocityReadValue = message.data.byte[4] | (message.data.byte[5] << 8);
      readVelocityResponseTime = currentTime - readVelocityStartTime;
      velocityReadComplete = true;
      velocityReadTimeout = false;
      readVelocityState = READ_VELOCITY_IDLE;
    }
    // Check for timeout
    else if (currentTime - readVelocityStartTime >= READ_TIMEOUT_MS) {
      // Timeout occurred
      readVelocityResponseTime = currentTime - readVelocityStartTime;
      velocityReadComplete = false;
      velocityReadTimeout = true;
      readVelocityState = READ_VELOCITY_IDLE;
    }
  }
}

// Check if read operation is complete (success or timeout)
bool isVelocityReadComplete() {
  return velocityReadComplete;
}

// Check if read operation timed out
bool isVelocityReadTimeout() {
  return velocityReadTimeout;
}

// Get velocity read status and data - returns formatted string for logging
// Returns: "SUCCESS: Velocity=XXX, Time=Xms" or "TIMEOUT: Time=Xms"
String getVelocityReadResult() {
  if (velocityReadComplete) {
    velocityReadComplete = false;
    velocityReadTimeout = false;
    return "CAN Read SUCCESS: Velocity=" + String(lastVelocityReadValue) + 
           ", Time=" + String(readVelocityResponseTime) + "us";
  } else if (velocityReadTimeout) {
    velocityReadComplete = false;
    velocityReadTimeout = false;
    return "CAN Read TIMEOUT: Time=" + String(readVelocityResponseTime) + "us";
  } else {
    return "CAN Read PENDING";
  }
}

// Get the last read velocity value and response time (raw access)
int16_t getLastVelocityReadValue(unsigned long &responseTime) {
  responseTime = readVelocityResponseTime;
  return lastVelocityReadValue;
}

// Check if read operation is still busy
bool isVelocityReadBusy() {
  return readVelocityState != READ_VELOCITY_IDLE;
}

int16_t getCurrentActualValue() {
  ReadCAN(0x30D1, 0x02);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t CurrentActualValue = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    // Extract mode of operation from the message frame
    CurrentActualValue = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return CurrentActualValue;
}

int16_t getTargetPosition() {
  ReadCAN(0x607A, 0x00);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t TargetPosition = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    // Extract mode of operation from the message frame
    // Single byte from the CAN message (message.data.byte[4]), which can hold values from 0 to 255.
    // combine bytes 4 and 5. (value is stored in little-endian format)
    // TargetPosition = message.data.byte[4];
    TargetPosition = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return TargetPosition;
}

int16_t getMaxprofilevelocity() {
  ReadCAN(0x607F, 0x00);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t Maxprofilevelocity = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    Maxprofilevelocity = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return Maxprofilevelocity;
}

void getPositionControlPIDparameters(int32_t &Position_Pgain, int32_t &Position_Igain, int32_t &Position_Dgain) {
  CAN_FRAME message;
  // Read P-gain value
  ReadCAN(0x30A1, 0x01);  // Send request to read P-gain
  delay(CAN_DELAY);       // Delay to prevent CAN Rx queue overflow
  if (CAN0.read(message)) {
    // Position_Pgain = (float)(message.data.byte[4] | (message.data.byte[5] << 8) | (message.data.byte[6] << 16) | (message.data.byte[7] << 24));
    Position_Pgain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8));
  }

  // Read I-gain value
  ReadCAN(0x30A1, 0x02);  // Send request to read I-gain
  delay(CAN_DELAY);       // Delay to prevent CAN Rx queue overflow
  if (CAN0.read(message)) {
    // Position_Igain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8) | (message.data.byte[6] << 16) | (message.data.byte[7] << 24));
    Position_Igain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8));
  }

  // Read D-gain value
  ReadCAN(0x30A1, 0x03);  // Send request to read D-gain
  delay(CAN_DELAY);       // Delay to prevent CAN Rx queue overflow
  if (CAN0.read(message)) {
    // Position_Dgain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8) | (message.data.byte[6] << 16) | (message.data.byte[7] << 24));
    Position_Dgain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8));
  }
}

void getVelocityControlPIDparameters(int32_t &Velocity_Pgain, int32_t &Velocity_Igain) {
  CAN_FRAME message;

  // Read P-gain value
  ReadCAN(0x30A2, 0x01);  // Send request to read P-gain
  delay(CAN_DELAY);       // Delay to prevent CAN Rx queue overflow
  if (CAN0.read(message)) {
    // Velocity_Pgain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8) | (message.data.byte[6] << 16) | (message.data.byte[7] << 24));
    Velocity_Pgain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8));
  }

  // Read I-gain value
  ReadCAN(0x30A2, 0x02);  // Send request to read I-gain
  delay(CAN_DELAY);       // Delay to prevent CAN Rx queue overflow
  if (CAN0.read(message)) {
    // Velocity_Igain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8) | (message.data.byte[6] << 16) | (message.data.byte[7] << 24));
    Velocity_Igain = (int32_t)(message.data.byte[4] | (message.data.byte[5] << 8));
  }
}

int16_t getProfileVelocity() {
  ReadCAN(0x6081, 0x00);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t profileVelocity = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    profileVelocity = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return profileVelocity;
}

int16_t getProfileAcceleration() {
  ReadCAN(0x6083, 0x00);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t profileAcceleration = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    profileAcceleration = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return profileAcceleration;
}

int16_t getProfileDeceleration() {
  ReadCAN(0x6084, 0x00);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t profileDeceleration = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    profileDeceleration = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return profileDeceleration;
}

int16_t getVelocityDemandValue() {
  ReadCAN(0x606B, 0x00);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int16_t VelocityDemandValue = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    VelocityDemandValue = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return VelocityDemandValue;
}

int32_t getEncoderPulse() {
  ReadCAN(0x3010, 0x04);
  delay(CAN_DELAY);  //Delay to prevent CAN Rx queue overflow
  int32_t EncoderPulse = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {
    EncoderPulse = message.data.byte[4] | (message.data.byte[5] << 8);
  }
  return EncoderPulse;
}
