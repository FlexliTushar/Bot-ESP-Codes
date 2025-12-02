/*
 * Simple Library-less Servo Control Test
 * 
 * This sketch demonstrates basic servo communication using Modbus RTU over UART
 * without external libraries. It implements the minimal necessary protocol details.
 * 
 * Hardware:
 * - ESP32
 * - Servo connected to Serial2 (GPIO 25 RX, GPIO 32 TX)
 * - Servo ID: 2 (Front Servo)
 * 
 * Logic:
 * 1. Enable Torque
 * 2. Move to Target Position
 * 3. Loop: Read Position -> Check if reached -> Disable Torque
 */

#include <Preferences.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define RX_PIN              25
#define TX_PIN              32
#define BAUDRATE            115200
#define SERVO_ID            2       // Front Servo ID
#define MOVE_SPEED          1000    // Speed
#define MOVE_ACC            50      // Acceleration

// Register Addresses (Based on SMS/Feetech standard)
#define REG_TORQUE_ENABLE   40
#define REG_GOAL_POSITION   42
#define REG_PRESENT_POS     56

// Modbus Function Codes
#define FC_READ_REGISTERS   0x03
#define FC_WRITE_SINGLE     0x06
#define FC_WRITE_MULTIPLE   0x10

// Global State
bool torqueDisabled = false;

String BOT_ID = "B";
Preferences prefs;

// Servo Limit Config variables.
int front_diverter_left_limit = 0;
int front_diverter_right_limit = 0;
int front_diverter_tolerance = 0;
int front_diverter_left_thresold = 0;
int front_diverter_right_thresold = 0;

int rear_diverter_left_limit = 0;
int rear_diverter_right_limit = 0;
int rear_diverter_tolerance = 0;
int rear_diverter_left_thresold = 0;
int rear_diverter_right_thresold = 0;

int current_position_front_servo = 0;
int current_position_rear_servo = 0;

// ============================================================================
// LOW-LEVEL MODBUS FUNCTIONS
// ============================================================================

// Calculate CRC16 for Modbus
uint16_t calcCRC(uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Send raw bytes to servo
void sendPacket(uint8_t *buffer, uint8_t length) {
  // Add CRC
  uint16_t crc = calcCRC(buffer, length);
  buffer[length] = crc & 0xFF;      // CRC Low
  buffer[length + 1] = crc >> 8;    // CRC High
  
  // Debug: print TX frame
  Serial.print("TX: ");
  for (int i = 0; i < length + 2; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Clear RX buffer before sending
  while (Serial2.available()) Serial2.read();
  
  // Send
  Serial2.write(buffer, length + 2);
  Serial2.flush();
  
  // Small delay for bus turnaround
  delay(3); 
}

// Read response from servo
// Returns number of bytes read, or -1 if timeout/error
int readResponse(uint8_t *buffer, int expectedLen, int timeoutMs = 50) {
  unsigned long startTime = millis();
  int bytesRead = 0;
  
  while ((millis() - startTime) < timeoutMs) {
    if (Serial2.available()) {
      buffer[bytesRead++] = Serial2.read();
      if (bytesRead >= expectedLen) break;
    } else {
      delay(1); // Yield to avoid WDT
    }
  }
  
  if (bytesRead < expectedLen) {
    Serial.print("Error: Timeout/Incomplete. Read ");
    Serial.print(bytesRead);
    Serial.print("/");
    Serial.print(expectedLen);
    if (bytesRead > 0) {
      Serial.print(" RX: ");
      for (int i = 0; i < bytesRead; i++) {
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
    }
    Serial.println();
    return -1;
  }
  
  // Debug: print RX frame
  Serial.print("RX: ");
  for (int i = 0; i < bytesRead; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Verify CRC
  uint16_t receivedCRC = (buffer[bytesRead - 1] << 8) | buffer[bytesRead - 2];
  uint16_t calculatedCRC = calcCRC(buffer, bytesRead - 2);
  
  if (receivedCRC != calculatedCRC) {
    Serial.print("Error: CRC Mismatch. Recv: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(" Calc: 0x");
    Serial.println(calculatedCRC, HEX);
    return -2;
  }
  
  return bytesRead;
}

// ============================================================================
// SERVO COMMANDS
// ============================================================================

// Enable or Disable Torque
bool enableTorque(uint8_t id, bool enable) {
  uint8_t txBuf[8];
  uint8_t rxBuf[8];
  
  // Frame: ID, FC, AddrHi, AddrLo, ValHi, ValLo, CRC_L, CRC_H
  txBuf[0] = id;
  txBuf[1] = FC_WRITE_SINGLE;
  txBuf[2] = 0x00;
  txBuf[3] = REG_TORQUE_ENABLE;
  txBuf[4] = 0x00;
  txBuf[5] = enable ? 1 : 0;
  
  sendPacket(txBuf, 6);
  
  // Expect echo response (same length)
  int result = readResponse(rxBuf, 8);
  return (result == 8);
}

// Move Servo (Write Multiple Registers)
bool setPosition(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
  uint8_t txBuf[20];
  uint8_t rxBuf[8];
  
  // We need to write 4 registers starting at REG_GOAL_POSITION
  // Reg 1: Position
  // Reg 2: 1 (Reserved/Time?)
  // Reg 3: Acceleration
  // Reg 4: Speed
  
  txBuf[0] = id;
  txBuf[1] = FC_WRITE_MULTIPLE;
  txBuf[2] = 0x00;
  txBuf[3] = REG_GOAL_POSITION;
  txBuf[4] = 0x00;
  txBuf[5] = 0x04; // Quantity: 4 registers
  txBuf[6] = 0x08; // Byte count: 8 bytes
  
  // Data 1: Position (Little-Endian: Low byte first)
  txBuf[7] = position & 0xFF;
  txBuf[8] = (position >> 8) & 0xFF;
  
  // Data 2: Fixed value 1 (from SMS library) (Little-Endian)
  txBuf[9] = 0x01;
  txBuf[10] = 0x00;
  
  // Data 3: Acceleration (Little-Endian)
  txBuf[11] = acc & 0xFF;
  txBuf[12] = (acc >> 8) & 0xFF;
  
  // Data 4: Speed (Little-Endian)
  txBuf[13] = speed & 0xFF;
  txBuf[14] = (speed >> 8) & 0xFF;
  
  sendPacket(txBuf, 15); // 15 bytes data + 2 CRC = 17 total sent
  
  // Expect response: ID, FC, AddrHi, AddrLo, QtyHi, QtyLo, CRC_L, CRC_H
  int result = readResponse(rxBuf, 8);
  return (result == 8);
}

// Read Current Position
int16_t readPosition(uint8_t id) {
  uint8_t txBuf[8];
  uint8_t rxBuf[10]; // ID, FC, Bytes, ValHi, ValLo, CRC_L, CRC_H
  
  txBuf[0] = id;
  txBuf[1] = FC_READ_REGISTERS;
  txBuf[2] = 0x00;
  txBuf[3] = REG_PRESENT_POS;
  txBuf[4] = 0x00;
  txBuf[5] = 0x01; // 1 Register
  
  sendPacket(txBuf, 6);
  
  // Response: ID(1), FC(1), Bytes(1), Data(2), CRC(2) = 7 bytes
  int result = readResponse(rxBuf, 7);
  
  if (result == 7) {
    // SMS uses little-endian: low byte first
    int16_t pos = rxBuf[4] | (rxBuf[3] << 8);
    return pos;
  }
  
  return -1; // Error
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  delay(1000);
  Serial.println("\n--- Simple Servo Test ---");
  
  // Load Config
  LoadDiverterConfig();
  if (front_diverter_right_limit == 0) {
    Serial.println("Warning: Config loaded 0 for right limit. Check NVM.");
  } else {
    Serial.print("Config Loaded. Target: ");
    Serial.println(front_diverter_right_limit);
  }
  
  // 1. Enable Torque
  Serial.print("Enabling Torque... ");
  if (enableTorque(SERVO_ID, true)) {
    Serial.println("Success");
  } else {
    Serial.println("Failed (No Response or CRC Error)");
  }
  
  delay(100);
  
  // 2. Move to Target
  Serial.print("Moving to ");
  Serial.print(front_diverter_right_limit);
  Serial.print("... ");
  if (setPosition(SERVO_ID, front_diverter_right_limit, MOVE_SPEED, MOVE_ACC)) {
    Serial.println("Command Sent");
  } else {
    Serial.println("Failed to send move command");
  }
}

void loop() {
  if (torqueDisabled) {
    delay(1000);
    return;
  }
  
  // 3. Read Position
  int16_t currentPos = readPosition(SERVO_ID);
  
  if (currentPos != -1) {
    Serial.print("Current Pos: ");
    Serial.println(currentPos);
    
    // 4. Check Range
    int diff = abs(currentPos - front_diverter_right_limit);
    if (diff <= front_diverter_tolerance) {
      Serial.println("Target Reached! Disabling Torque...");
      
      if (enableTorque(SERVO_ID, false)) {
        Serial.println("Torque Disabled.");
        torqueDisabled = true;
      } else {
        Serial.println("Failed to disable torque.");
      }
    }
  } else {
    Serial.println("Read Error");
  }
  
  delay(200);
}

// Function to load values from NVM
void LoadDiverterConfig() {
  if (!prefs.begin("diverter", true)) {
    Serial.println("Error: Failed to open NVM namespace 'diverter'");
    return;
  }
  
  BOT_ID = prefs.getString("BotId", "Bx");

  front_diverter_left_limit = prefs.getInt("front_left_lim", 1994);
  front_diverter_right_limit = prefs.getInt("front_right_lim", 2525);
  front_diverter_tolerance = prefs.getInt("front_tol", 106);
  front_diverter_left_thresold = prefs.getInt("front_left_thr", 2100);
  front_diverter_right_thresold = prefs.getInt("front_right_thr", 2419);
  
  rear_diverter_left_limit = prefs.getInt("rear_left_lim", 2936);
  rear_diverter_right_limit = prefs.getInt("rear_right_lim", 2387);
  rear_diverter_tolerance = prefs.getInt("rear_tol", 109);
  rear_diverter_left_thresold = prefs.getInt("rear_left_thr", 2827);
  rear_diverter_right_thresold = prefs.getInt("rear_right_thr", 2496);
  prefs.end();
  Serial.println("Diverter config loaded from NVM.");
}