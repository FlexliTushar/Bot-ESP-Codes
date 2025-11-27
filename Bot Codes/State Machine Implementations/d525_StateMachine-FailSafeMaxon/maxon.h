bool WriteCAN(uint16_t index, uint8_t subindex, uint32_t decimalData) {
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
  
  // Wait for response and verify it matches our request
  unsigned long startTime = millis();
  while (millis() - startTime < 50) {  // 50ms timeout
    CAN_FRAME response;
    if (CAN0.read(response)) {
      if (response.id == 0x581) {
        // Check if this is an error response (SDO abort)
        if (response.data.uint8[0] == 0x80) {
          // SDO abort - extract error code for debugging
          // uint32_t errorCode = response.data.uint8[4] | (response.data.uint8[5] << 8) | 
          //                      (response.data.uint8[6] << 16) | (response.data.uint8[7] << 24);
          return false;  // Communication succeeded but command was rejected
        }
        // Check if this is the success response for OUR command
        if (response.data.uint8[0] == 0x60 &&
            response.data.uint8[1] == (index & 0xFF) &&
            response.data.uint8[2] == ((index >> 8) & 0xFF) &&
            response.data.uint8[3] == subindex) {
          return true;  // Got acknowledge for correct index/subindex
        }
      }
    }
  }
  return false;  // Timeout or no matching response
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

bool haltMovement() {
  bool success = WriteCAN(CONTROLWORD, 0x00, 271);  // 0x010F - EPOS4 Application Notes PG 122
  delay(CAN_DELAY);                  // Optional delay after setting mode
  return success;
}

void QuickStop() {
  WriteCAN(CONTROLWORD, 0x00, 11);  // 0x000B - EPOS4 Application Notes PG 122
  delay(CAN_DELAY);                 // Optional delay after setting mode
}

bool disable() {
  bool success = WriteCAN(CONTROLWORD, 0x00, 6);
  delay(CAN_DELAY);
  return success;
}

bool enable() {
  bool success = WriteCAN(CONTROLWORD, 0x00, 15);  // Controlword (Switch on & Enable)
  delay(CAN_DELAY);                 // Optional delay after setting mode
  return success;
}

bool setOperationMode(int mode) {
  bool success = WriteCAN(OPERATION_MODE, 0x00, mode);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setMaxProfileVelocity(int MaxProfileVelocity) {
  bool success = WriteCAN(MAX_PROFILE_VELOCITY, 0x00, MaxProfileVelocity);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setProfileVelocity(int ProfileVelocity) {
  bool success = WriteCAN(PROFILE_VELOCITY, 0x00, ProfileVelocity);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setProfileAcceleration(int ProfileAcceleration) {
  bool success = WriteCAN(PROFILE_ACCELERATION, 0x00, ProfileAcceleration);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setProfileDeceleration(int ProfileDeceleration) {
  bool success = WriteCAN(PROFILE_DECELERATION, 0x00, ProfileDeceleration);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setQuickStopDeceleration(int QuickStopDeceleration) {
  bool success = WriteCAN(QUICK_STOP_DECELERATION, 0x00, QuickStopDeceleration);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setMotionProfileType(int MotionProfileType) {
  bool success = WriteCAN(MOTION_PROFILE_TYPE, 0x00, MotionProfileType);
  delay(CAN_DELAY);  // Optional delay after setting mode
  return success;
}

bool setTagetVelocity(int TagetVelocity) {
  if (!WriteCAN(TARGET_VELOCITY, 0x00, TagetVelocity)) return false;
  delay(CAN_DELAY);
  if (!WriteCAN(CONTROLWORD, 0x00, 15)) return false;  // 0x000F Refer  EPOS4 Application NOtes PG123.
  delay(CAN_DELAY);                 // Optional delay after setting mode
  return true;
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

int32_t getVelocityActualValueAveraged() {
  ReadCAN(0x30D3, 0x01);
  
  unsigned long startTime = millis();
  while (millis() - startTime < 50) {
    CAN_FRAME message;
    if (CAN0.read(message)) {
      if (message.id == 0x581) {
        if (message.data.byte[0] == 0x80) {
          return INT32_MIN;  // Special error value
        }
        if ((message.data.byte[0] == 0x4B || message.data.byte[0] == 0x43) &&
            message.data.byte[1] == 0xD3 &&
            message.data.byte[2] == 0x30 &&
            message.data.byte[3] == 0x01) {
          return (int16_t)(message.data.byte[4] | (message.data.byte[5] << 8));
        }
      }
    }
  }
  return INT32_MIN;  // Timeout - return special error value
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
  
  unsigned long startTime = millis();
  while (millis() - startTime < 50) {
    CAN_FRAME message;
    if (CAN0.read(message)) {
      if (message.id == 0x581) {
        if (message.data.byte[0] == 0x80) {
          return INT16_MIN;  // Special error value
        }
        if ((message.data.byte[0] == 0x4B || message.data.byte[0] == 0x43) &&
            message.data.byte[1] == 0x6B &&
            message.data.byte[2] == 0x60 &&
            message.data.byte[3] == 0x00) {
          return (int16_t)(message.data.byte[4] | (message.data.byte[5] << 8));
        }
      }
    }
  }
  return INT16_MIN;  // Timeout - return special error value
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
