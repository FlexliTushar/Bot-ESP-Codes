#define FIRMWARE_VERSION "d542"
/*
 * Library-less Diverter State Machine Control
 * ============================================
 * 
 * This sketch controls front and rear diverter servos using raw Modbus RTU
 * over UART without external libraries. It implements the diverter state
 * machine with proper synchronization between both servos.
 * 
 * Hardware:
 * - ESP32
 * - Front Servo (ID 2) and Rear Servo (ID 3) on Serial2
 * - GPIO 25 RX, GPIO 32 TX
 * 
 * Features:
 * - Full state machine: STOP, LEFT, RIGHT, SWITCHING, ERROR
 * - Synchronized front/rear movement
 * - Library-less Modbus implementation
 * - Comprehensive error handling and diagnostics
 */

#include <Preferences.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define RX_PIN              25
#define TX_PIN              32
#define BAUDRATE            115200
#define BUZZER              13

#define FRONT_SERVO_ID      2
#define REAR_SERVO_ID       3

#define ENABLE              1
#define DISABLE             0

// Register Addresses (From SMS.h)
#define REG_TORQUE_ENABLE   129
#define REG_GOAL_POSITION   128
#define REG_PRESENT_POS     257

// Modbus Function Codes
#define FC_READ_REGISTERS   3
#define FC_WRITE_SINGLE     6
#define FC_WRITE_MULTIPLE   16

// ============================================================================
// ENUMERATIONS
// ============================================================================

enum DiverterState : uint8_t {
    DIVERTER_STOP = 0,
    DIVERTER_LEFT = 1,
    DIVERTER_RIGHT = 2,
    DIVERTER_SWITCHING = 3,
    DIVERTER_ERROR = 4
};

enum DiverterDirection : uint8_t {
    DIVERTER_DIRECTION_UNKNOWN = 0,
    DIVERTER_DIRECTION_LEFT = 1,
    DIVERTER_DIRECTION_RIGHT = 2
};

enum DiverterPosition : uint8_t {
    DIVERTER_POSITION_UNKNOWN = 0,
    DIVERTER_POSITION_LEFT = 1,
    DIVERTER_POSITION_RIGHT = 2,
    DIVERTER_POSITION_INTERIM = 3
};

enum DiverterStateChangeOptions : uint8_t {
    DIVERTER_STATE_NO_CHANGE = 0,
    DIVERTER_STATE_CHANGED = 1,
    DIVERTER_UNEXPECTED_CONDITION = 2
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

String BOT_ID = "B";
Preferences prefs;

// Config variables
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

// Current positions
int front_diverter_current_position = -1;
int rear_diverter_current_position = -1;

// State machine variables
DiverterState diverter_current_state = DIVERTER_STOP;
DiverterState diverter_previous_state = DIVERTER_STOP;
DiverterDirection diverter_demand_direction = DIVERTER_DIRECTION_RIGHT;
DiverterPosition current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;

bool global_error_status = false;
bool error_front_servo = false;
bool error_rear_servo = false;

// Target positions
int front_target_position = -1;
int rear_target_position = -1;

// Movement control
bool enable_torque_flag = false;
unsigned long last_position_read_time = 0;
const unsigned long POSITION_READ_INTERVAL = 50;  // 50ms between position reads

// Test mode toggle
unsigned long last_toggle_time = 0;
const unsigned long TOGGLE_INTERVAL = 10000;  // 10 seconds

// RTOS Task Handles
TaskHandle_t diverterStateTask;
TaskHandle_t diverterActuationTask;
TaskHandle_t auxiliaryTask;

// ============================================================================
// LOW-LEVEL MODBUS FUNCTIONS
// ============================================================================

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

void sendPacket(uint8_t *buffer, uint8_t length) {
  uint16_t crc = calcCRC(buffer, length);
  buffer[length] = crc & 0xFF;
  buffer[length + 1] = crc >> 8;
  
  while (Serial2.available()) Serial2.read();
  Serial2.write(buffer, length + 2);
  Serial2.flush();
  delay(3);
}

int readResponse(uint8_t *buffer, int expectedLen, int timeoutMs = 100) {
  unsigned long startTime = millis();
  int bytesRead = 0;
  
  while ((millis() - startTime) < timeoutMs) {
    if (Serial2.available()) {
      buffer[bytesRead++] = Serial2.read();
      if (bytesRead >= expectedLen) break;
    } else {
      delay(1);
    }
  }
  
  if (bytesRead < expectedLen) {
    Serial.print("✗ Timeout: ");
    Serial.print(bytesRead);
    Serial.print("/");
    Serial.println(expectedLen);
    return -1;
  }
  
  uint16_t receivedCRC = (buffer[bytesRead - 1] << 8) | buffer[bytesRead - 2];
  uint16_t calculatedCRC = calcCRC(buffer, bytesRead - 2);
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("✗ CRC Error");
    return -2;
  }
  
  return bytesRead;
}

// ============================================================================
// SERVO COMMANDS - FRONT SERVO
// ============================================================================

bool enableTorqueFront(bool enable) {
  uint8_t txBuf[8], rxBuf[8];
  
  txBuf[0] = FRONT_SERVO_ID;
  txBuf[1] = FC_WRITE_SINGLE;
  txBuf[2] = (REG_TORQUE_ENABLE >> 8) & 0xFF;
  txBuf[3] = REG_TORQUE_ENABLE & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = enable ? 0x01 : 0x00;
  
  sendPacket(txBuf, 6);
  return (readResponse(rxBuf, 8, 100) == 8);
}

bool setPositionFront(int16_t position, uint16_t speed, uint8_t acc) {
  uint8_t txBuf[20], rxBuf[8];
  
  txBuf[0] = FRONT_SERVO_ID;
  txBuf[1] = FC_WRITE_MULTIPLE;
  txBuf[2] = (REG_GOAL_POSITION >> 8) & 0xFF;
  txBuf[3] = REG_GOAL_POSITION & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x04;
  txBuf[6] = 0x08;
  
  txBuf[7] = (position >> 8) & 0xFF;
  txBuf[8] = position & 0xFF;
  txBuf[9] = 0x00;
  txBuf[10] = 0x01;
  txBuf[11] = (acc >> 8) & 0xFF;
  txBuf[12] = acc & 0xFF;
  txBuf[13] = (speed >> 8) & 0xFF;
  txBuf[14] = speed & 0xFF;
  
  sendPacket(txBuf, 15);
  return (readResponse(rxBuf, 8, 100) == 8);
}

int16_t readPositionFront() {
  uint8_t txBuf[8], rxBuf[10];
  
  txBuf[0] = FRONT_SERVO_ID;
  txBuf[1] = FC_READ_REGISTERS;
  txBuf[2] = (REG_PRESENT_POS >> 8) & 0xFF;
  txBuf[3] = REG_PRESENT_POS & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x01;
  
  sendPacket(txBuf, 6);
  
  int result = readResponse(rxBuf, 7, 100);
  if (result == 7) {
    return (rxBuf[3] << 8) | rxBuf[4];
  }
  return -1;
}

// ============================================================================
// SERVO COMMANDS - REAR SERVO
// ============================================================================

bool enableTorqueRear(bool enable) {
  uint8_t txBuf[8], rxBuf[8];
  
  txBuf[0] = REAR_SERVO_ID;
  txBuf[1] = FC_WRITE_SINGLE;
  txBuf[2] = (REG_TORQUE_ENABLE >> 8) & 0xFF;
  txBuf[3] = REG_TORQUE_ENABLE & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = enable ? 0x01 : 0x00;
  
  sendPacket(txBuf, 6);
  return (readResponse(rxBuf, 8, 100) == 8);
}

bool setPositionRear(int16_t position, uint16_t speed, uint8_t acc) {
  uint8_t txBuf[20], rxBuf[8];
  
  txBuf[0] = REAR_SERVO_ID;
  txBuf[1] = FC_WRITE_MULTIPLE;
  txBuf[2] = (REG_GOAL_POSITION >> 8) & 0xFF;
  txBuf[3] = REG_GOAL_POSITION & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x04;
  txBuf[6] = 0x08;
  
  txBuf[7] = (position >> 8) & 0xFF;
  txBuf[8] = position & 0xFF;
  txBuf[9] = 0x00;
  txBuf[10] = 0x01;
  txBuf[11] = (acc >> 8) & 0xFF;
  txBuf[12] = acc & 0xFF;
  txBuf[13] = (speed >> 8) & 0xFF;
  txBuf[14] = speed & 0xFF;
  
  sendPacket(txBuf, 15);
  return (readResponse(rxBuf, 8, 100) == 8);
}

int16_t readPositionRear() {
  uint8_t txBuf[8], rxBuf[10];
  
  txBuf[0] = REAR_SERVO_ID;
  txBuf[1] = FC_READ_REGISTERS;
  txBuf[2] = (REG_PRESENT_POS >> 8) & 0xFF;
  txBuf[3] = REG_PRESENT_POS & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x01;
  
  sendPacket(txBuf, 6);
  
  int result = readResponse(rxBuf, 7, 100);
  if (result == 7) {
    return (rxBuf[3] << 8) | rxBuf[4];
  }
  return -1;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

String DiverterStateToString(DiverterState state) {
  switch(state) {
    case DIVERTER_STOP: return "STOP";
    case DIVERTER_LEFT: return "LEFT";
    case DIVERTER_RIGHT: return "RIGHT";
    case DIVERTER_SWITCHING: return "SWITCHING";
    case DIVERTER_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

String DiverterDirectionToString(DiverterDirection dir) {
  switch(dir) {
    case DIVERTER_DIRECTION_LEFT: return "LEFT";
    case DIVERTER_DIRECTION_RIGHT: return "RIGHT";
    case DIVERTER_DIRECTION_UNKNOWN: return "UNKNOWN";
    default: return "UNKNOWN";
  }
}

String DiverterPositionToString(DiverterPosition pos) {
  switch(pos) {
    case DIVERTER_POSITION_LEFT: return "LEFT";
    case DIVERTER_POSITION_RIGHT: return "RIGHT";
    case DIVERTER_POSITION_INTERIM: return "INTERIM";
    case DIVERTER_POSITION_UNKNOWN: return "UNKNOWN";
    default: return "UNKNOWN";
  }
}

void UpdateDiverterPositionHighLevel() {
  // Both servos at RIGHT position
  if (front_diverter_current_position >= (front_diverter_right_thresold - front_diverter_tolerance) && 
      front_diverter_current_position <= front_diverter_right_limit &&
      rear_diverter_current_position <= (rear_diverter_right_thresold + rear_diverter_tolerance) && 
      rear_diverter_current_position >= rear_diverter_right_limit) {
    current_diverter_position_high_level = DIVERTER_POSITION_RIGHT;
  }
  // Both servos at LEFT position
  else if (front_diverter_current_position >= front_diverter_left_limit && 
           front_diverter_current_position <= (front_diverter_left_thresold + front_diverter_tolerance) &&
           rear_diverter_current_position <= rear_diverter_left_limit && 
           rear_diverter_current_position >= (rear_diverter_left_thresold - rear_diverter_tolerance)) {
    current_diverter_position_high_level = DIVERTER_POSITION_LEFT;
  }
  // Both servos in INTERIM (between positions)
  else if (front_diverter_current_position < (front_diverter_right_thresold - front_diverter_tolerance) && 
           front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance) &&
           rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance) && 
           rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
    current_diverter_position_high_level = DIVERTER_POSITION_INTERIM;
  }
  else {
    current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;
  }
}

// ============================================================================
// STATE MACHINE
// ============================================================================

DiverterStateChangeOptions UpdateDiverterState() {
  diverter_previous_state = diverter_current_state;
  global_error_status = error_front_servo || error_rear_servo;
  
  // ----- FROM STOP STATE -----
  if (diverter_current_state == DIVERTER_STOP) {
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (global_error_status == false) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM LEFT STATE -----
  else if (diverter_current_state == DIVERTER_LEFT) {
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (diverter_demand_direction == DIVERTER_DIRECTION_RIGHT ||
        current_diverter_position_high_level != DIVERTER_POSITION_LEFT) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM RIGHT STATE -----
  else if (diverter_current_state == DIVERTER_RIGHT) {
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (diverter_demand_direction == DIVERTER_DIRECTION_LEFT ||
        current_diverter_position_high_level != DIVERTER_POSITION_RIGHT) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM SWITCHING STATE -----
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (diverter_demand_direction == DIVERTER_DIRECTION_LEFT && 
        current_diverter_position_high_level == DIVERTER_POSITION_LEFT) {
      diverter_current_state = DIVERTER_LEFT;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (diverter_demand_direction == DIVERTER_DIRECTION_RIGHT && 
        current_diverter_position_high_level == DIVERTER_POSITION_RIGHT) {
      diverter_current_state = DIVERTER_RIGHT;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM ERROR STATE -----
  else if (diverter_current_state == DIVERTER_ERROR) {
    if (global_error_status == false) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  return DIVERTER_UNEXPECTED_CONDITION;
}

void ImplementDiverterState() {
  if (diverter_current_state == DIVERTER_STOP) {
    enable_torque_flag = false;
    Serial.println("→ STOP: Torque disabled");
  }
  
  else if (diverter_current_state == DIVERTER_LEFT) {
    enable_torque_flag = false;
    Serial.println("→ LEFT: Position reached, torque disabled");
  }
  
  else if (diverter_current_state == DIVERTER_RIGHT) {
    enable_torque_flag = false;
    Serial.println("→ RIGHT: Position reached, torque disabled");
  }
  
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    enable_torque_flag = true;
    
    if (diverter_demand_direction == DIVERTER_DIRECTION_LEFT) {
      front_target_position = front_diverter_left_thresold;
      rear_target_position = rear_diverter_left_thresold;
      Serial.print("→ SWITCHING to LEFT: F=");
      Serial.print(front_target_position);
      Serial.print(" R=");
      Serial.println(rear_target_position);
    }
    else if (diverter_demand_direction == DIVERTER_DIRECTION_RIGHT) {
      front_target_position = front_diverter_right_thresold;
      rear_target_position = rear_diverter_right_thresold;
      Serial.print("→ SWITCHING to RIGHT: F=");
      Serial.print(front_target_position);
      Serial.print(" R=");
      Serial.println(rear_target_position);
    }
  }
  
  else if (diverter_current_state == DIVERTER_ERROR) {
    enable_torque_flag = false;
    Serial.println("→ ERROR: Torque disabled");
  }
}

// ============================================================================
// ACTUATION LOGIC (Now handled by DIVERTER_ACTUATION_TASK)
// ============================================================================
// Note: ActuateDiverter() logic has been moved to DIVERTER_ACTUATION_TASK

// ============================================================================
// RTOS TASK FUNCTIONS
// ============================================================================

void DIVERTER_STATE_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms cycle time
    
    DiverterStateChangeOptions stateChange = UpdateDiverterState();
    
    if (stateChange == DIVERTER_STATE_CHANGED) {
      Serial.print("\n[STATE] ");
      Serial.print(DiverterStateToString(diverter_previous_state));
      Serial.print(" → ");
      Serial.println(DiverterStateToString(diverter_current_state));
      Serial.print("  Demand: ");
      Serial.print(DiverterDirectionToString(diverter_demand_direction));
      Serial.print(" | Position: ");
      Serial.println(DiverterPositionToString(current_diverter_position_high_level));
      Serial.print("  Front: ");
      Serial.print(front_diverter_current_position);
      Serial.print(" | Rear: ");
      Serial.println(rear_diverter_current_position);
      
      ImplementDiverterState();
    } else if (stateChange == DIVERTER_UNEXPECTED_CONDITION) {
      Serial.println("✗ UNEXPECTED STATE CONDITION!");
    }
  }
}

void DIVERTER_ACTUATION_TASK(void* pvParameters) {
  static bool prev_enable_torque_flag = false;
  
  while (true) {
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms cycle time
    
    // Handle torque enable/disable
    if (enable_torque_flag != prev_enable_torque_flag) {
      if (!enable_torque_flag) {
        // Disable torque
        delay(1);
        if (!enableTorqueFront(DISABLE)) {
          Serial.println("✗ Front torque disable failed");
          error_front_servo = true;
        }
        delay(1);
        if (!enableTorqueRear(DISABLE)) {
          Serial.println("✗ Rear torque disable failed");
          error_rear_servo = true;
        }
      } else {
        // Enable torque
        delay(1);
        if (!enableTorqueFront(ENABLE)) {
          Serial.println("✗ Front torque enable failed");
          error_front_servo = true;
        }
        delay(1);
        if (!enableTorqueRear(ENABLE)) {
          Serial.println("✗ Rear torque enable failed");
          error_rear_servo = true;
        }
      }
      prev_enable_torque_flag = enable_torque_flag;
    }
    
    // Send position commands if torque enabled and targets set
    if (enable_torque_flag && front_target_position != -1 && rear_target_position != -1) {
      // Front servo movement
      if (diverter_demand_direction == DIVERTER_DIRECTION_RIGHT) {
        if (front_diverter_current_position < (front_diverter_right_thresold - front_diverter_tolerance)) {
          delay(1);
          if (!setPositionFront(front_target_position, 100, 50)) {
            Serial.println("✗ Front position write failed");
            error_front_servo = true;
          }
        }
      } else if (diverter_demand_direction == DIVERTER_DIRECTION_LEFT) {
        if (front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance)) {
          delay(1);
          if (!setPositionFront(front_target_position, 100, 50)) {
            Serial.println("✗ Front position write failed");
            error_front_servo = true;
          }
        }
      }
      
      // Rear servo movement
      if (diverter_demand_direction == DIVERTER_DIRECTION_RIGHT) {
        if (rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance)) {
          delay(1);
          if (!setPositionRear(rear_target_position, 100, 50)) {
            Serial.println("✗ Rear position write failed");
            error_rear_servo = true;
          }
        }
      } else if (diverter_demand_direction == DIVERTER_DIRECTION_LEFT) {
        if (rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
          delay(1);
          if (!setPositionRear(rear_target_position, 100, 50)) {
            Serial.println("✗ Rear position write failed");
            error_rear_servo = true;
          }
        }
      }
    }
    
    // Read positions periodically
    unsigned long current_time = millis();
    if (current_time - last_position_read_time >= POSITION_READ_INTERVAL) {
      delay(1);
      int16_t front_pos = readPositionFront();
      if (front_pos == -1) {
        Serial.println("✗ Front position read failed");
        error_front_servo = true;
      } else {
        front_diverter_current_position = front_pos;
        error_front_servo = false;  // Clear error on successful read
      }
      
      delay(1);
      int16_t rear_pos = readPositionRear();
      if (rear_pos == -1) {
        Serial.println("✗ Rear position read failed");
        error_rear_servo = true;
      } else {
        rear_diverter_current_position = rear_pos;
        error_rear_servo = false;  // Clear error on successful read
      }
      
      UpdateDiverterPositionHighLevel();
      last_position_read_time = current_time;
    }
  }
}

void AUXILIARY_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(500 / portTICK_PERIOD_MS);  // 500ms cycle time
    
    unsigned long current_time = millis();
    
    // Check if 10 seconds have passed and no errors
    if ((current_time - last_toggle_time >= TOGGLE_INTERVAL) && !global_error_status) {
      // Toggle demand direction
      if (diverter_demand_direction == DIVERTER_DIRECTION_LEFT) {
        diverter_demand_direction = DIVERTER_DIRECTION_RIGHT;
        Serial.println("\n*** AUTO TOGGLE: Demand → RIGHT ***\n");
      } 
      else if (diverter_demand_direction == DIVERTER_DIRECTION_RIGHT) {
        diverter_demand_direction = DIVERTER_DIRECTION_LEFT;
        Serial.println("\n*** AUTO TOGGLE: Demand → LEFT ***\n");
      }
      
      last_toggle_time = current_time;
    }
  }
}

// ============================================================================
// SETUP AND MAIN LOOP
// ============================================================================

void LoadDiverterConfig() {
  if (!prefs.begin("diverter", true)) {
    Serial.println("✗ NVM failed - using defaults");
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
  Serial.println("✓ Config loaded from NVM");
}

void Beep() {
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  delay(1000);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  Library-less Diverter State Machine   ║");
  Serial.println("║            " FIRMWARE_VERSION "                         ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  LoadDiverterConfig();
  Beep();
  
  Serial.print("Bot: ");
  Serial.println(BOT_ID);
  Serial.println(__FILE__);
  
  Serial.println("\nFront Servo Limits:");
  Serial.print("  Left: ");
  Serial.print(front_diverter_left_thresold);
  Serial.print(" (±");
  Serial.print(front_diverter_tolerance);
  Serial.println(")");
  Serial.print("  Right: ");
  Serial.print(front_diverter_right_thresold);
  Serial.print(" (±");
  Serial.print(front_diverter_tolerance);
  Serial.println(")");
  
  Serial.println("Rear Servo Limits:");
  Serial.print("  Left: ");
  Serial.print(rear_diverter_left_thresold);
  Serial.print(" (±");
  Serial.print(rear_diverter_tolerance);
  Serial.println(")");
  Serial.print("  Right: ");
  Serial.print(rear_diverter_right_thresold);
  Serial.print(" (±");
  Serial.print(rear_diverter_tolerance);
  Serial.println(")");
  
  Serial.println("\n✓ System Ready");
  Serial.println("→ Creating RTOS Tasks...");
  
  // Create State Machine Task (Priority 5, Core 1)
  xTaskCreatePinnedToCore(
      DIVERTER_STATE_TASK,
      "diverter_state_task",
      10000,
      NULL,
      5,
      &diverterStateTask,
      1);
  
  vTaskDelay(pdMS_TO_TICKS(20));
  
  // Create Actuation Task (Priority 4, Core 1)
  xTaskCreatePinnedToCore(
      DIVERTER_ACTUATION_TASK,
      "diverter_actuation_task",
      10000,
      NULL,
      4,
      &diverterActuationTask,
      1);
  
  vTaskDelay(pdMS_TO_TICKS(20));
  
  // Create Auxiliary Task for Auto-Toggle (Priority 3, Core 1)
  xTaskCreatePinnedToCore(
      AUXILIARY_TASK,
      "auxiliary_task",
      10000,
      NULL,
      3,
      &auxiliaryTask,
      1);
  
  Serial.println("✓ RTOS Tasks Created");
  Serial.println("→ State Machine Active");
  Serial.println("→ Auto-toggle every 10 seconds\n");
  
  last_toggle_time = millis();
}

void loop() {
  // Empty - all functionality handled by RTOS tasks
}
