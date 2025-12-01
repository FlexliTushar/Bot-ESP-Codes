/*
  Minimal Velocity Control Test - Low Level CAN Implementation
  REFERENCE - EPOS 4 APPLICATION NOTES SECTION - 7.5 (Profile Velocity Mode)
  
  Hardware:
  - ESP32 CAN TX: GPIO 14
  - ESP32 CAN RX: GPIO 15
  - CAN transceiver connected to these pins
  - Maxon EPOS motor controller on CAN bus (Node ID = 1)
  
  What this does:
  Implements setTargetVelocity and haltMovement at low level, then:
  1. Configures motor in Profile Velocity Mode
  2. Cycles through different speed levels every 5 seconds:
     - 267 rpm (0.5 m/s)
     - 535 rpm (1.0 m/s)
     - 802 rpm (1.5 m/s)
     - Halt (stop)
     - Repeat
  
  No external libraries except esp-idf TWAI driver.
*/

#include "driver/twai.h"
#include "driver/gpio.h"

// CAN pins
#define CAN_TX_PIN GPIO_NUM_14
#define CAN_RX_PIN GPIO_NUM_15

// EPOS Node ID
#define NODE_ID 1

// CANopen COB-IDs for node 1
#define SDO_TX  (0x600 + NODE_ID)  // Client->Server (our requests)
#define SDO_RX  (0x580 + NODE_ID)  // Server->Client (EPOS responses)

// EPOS Object Dictionary indices
#define CONTROLWORD                 0x6040
#define OPERATION_MODE              0x6060
#define TARGET_VELOCITY             0x60FF
#define VELOCITY_ACTUAL_VALUE_AVG   0x30D3
#define MAX_PROFILE_VELOCITY        0x607F
#define PROFILE_ACCELERATION        0x6083
#define PROFILE_DECELERATION        0x6084
#define QUICK_STOP_DECELERATION     0x6085
#define MOTION_PROFILE_TYPE         0x6086

// Operation modes
#define PROFILE_VELOCITY_MODE       3

// Controlword values
#define CONTROLWORD_SHUTDOWN        6     // 0x0006 - disable motor
#define CONTROLWORD_ENABLE          15    // 0x000F - enable and start
#define CONTROLWORD_HALT            271   // 0x010F - halt movement (bit 8 set)

// Motor speed levels (from your object_dictionary.h)
#define S0_5          267   // 0.5 m/s
#define S1            535   // 1.0 m/s
#define S1_5          802   // 1.5 m/s

// SDO response timeout (ms)
#define SDO_RESPONSE_TIMEOUT        10

// Simple CAN frame structure
struct SimpleCANFrame {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
  bool     extended;
  bool     rtr;
};

// Status codes
enum CANStatus {
  CAN_OK = 0,
  CAN_TX_TIMEOUT,
  CAN_RX_TIMEOUT,
  CAN_SDO_ABORT,
  CAN_ERROR
};

//=============================================================================
// LOW-LEVEL CAN FUNCTIONS (down to TWAI API)
//=============================================================================

// Initialize CAN hardware
bool CAN_Init() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN, 
    CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  
  if (twai_start() != ESP_OK) {
    return false;
  }
  
  return true;
}

// Send a CAN frame
CANStatus CAN_Send(SimpleCANFrame* frame, uint32_t timeout_ms) {
  twai_message_t tx_msg;
  
  tx_msg.identifier       = frame->id;
  tx_msg.data_length_code = frame->dlc;
  tx_msg.extd             = frame->extended ? 1 : 0;
  tx_msg.rtr              = frame->rtr ? 1 : 0;
  
  for (int i = 0; i < frame->dlc; i++) {
    tx_msg.data[i] = frame->data[i];
  }
  
  esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(timeout_ms));
  
  if (result == ESP_OK) {
    return CAN_OK;
  } else if (result == ESP_ERR_TIMEOUT) {
    return CAN_TX_TIMEOUT;
  } else {
    return CAN_ERROR;
  }
}

// Receive a CAN frame
CANStatus CAN_Receive(SimpleCANFrame* frame, uint32_t timeout_ms) {
  twai_message_t rx_msg;
  
  esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(timeout_ms));
  
  if (result == ESP_OK) {
    frame->id       = rx_msg.identifier;
    frame->dlc      = rx_msg.data_length_code;
    frame->extended = rx_msg.extd;
    frame->rtr      = rx_msg.rtr;
    
    for (int i = 0; i < rx_msg.data_length_code && i < 8; i++) {
      frame->data[i] = rx_msg.data[i];
    }
    
    return CAN_OK;
  } else if (result == ESP_ERR_TIMEOUT) {
    return CAN_RX_TIMEOUT;
  } else {
    return CAN_ERROR;
  }
}

//=============================================================================
// CANOPEN SDO FUNCTIONS
//=============================================================================

// Send SDO Download (write) and wait for response
CANStatus SDO_Write(uint16_t index, uint8_t subindex, uint32_t value) {
  SimpleCANFrame tx_frame;
  
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x22;  // Expedited write
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = (index >> 8) & 0xFF;
  tx_frame.data[3] = subindex;
  tx_frame.data[4] = value & 0xFF;
  tx_frame.data[5] = (value >> 8) & 0xFF;
  tx_frame.data[6] = (value >> 16) & 0xFF;
  tx_frame.data[7] = (value >> 24) & 0xFF;
  
  // Send request
  CANStatus status = CAN_Send(&tx_frame, 4);
  if (status != CAN_OK) {
    return status;
  }
  
  // Wait for SDO response (ACK frame)
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < SDO_RESPONSE_TIMEOUT) {
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      if (rx_frame.id == SDO_RX) {
        uint8_t cmd = rx_frame.data[0];
        
        // SDO abort (0x80)
        if (cmd == 0x80) {
          uint32_t abort_code = rx_frame.data[4] 
                              | (rx_frame.data[5] << 8)
                              | (rx_frame.data[6] << 16)
                              | (rx_frame.data[7] << 24);
          Serial.print("SDO ABORT: 0x");
          Serial.println(abort_code, HEX);
          return CAN_SDO_ABORT;
        }
        
        // Download response (0x60)
        if (cmd == 0x60) {
          return CAN_OK;
        }
      }
    }
  }
  
  return CAN_RX_TIMEOUT;
}

// Send SDO Upload (read) and wait for response
CANStatus SDO_Read(uint16_t index, uint8_t subindex, uint32_t* value) {
  SimpleCANFrame tx_frame;
  
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x40;  // Upload request
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = (index >> 8) & 0xFF;
  tx_frame.data[3] = subindex;
  tx_frame.data[4] = 0x00;
  tx_frame.data[5] = 0x00;
  tx_frame.data[6] = 0x00;
  tx_frame.data[7] = 0x00;
  
  // Send request
  CANStatus status = CAN_Send(&tx_frame, 4);
  if (status != CAN_OK) {
    return status;
  }
  
  // Wait for response
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < SDO_RESPONSE_TIMEOUT) {
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      if (rx_frame.id == SDO_RX) {
        uint8_t cmd = rx_frame.data[0];
        
        // SDO abort (0x80)
        if (cmd == 0x80) {
          return CAN_SDO_ABORT;
        }
        
        // Upload response (0x43 or 0x4B typically)
        if ((cmd & 0xE0) == 0x40) {
          *value = rx_frame.data[4] 
                 | (rx_frame.data[5] << 8)
                 | (rx_frame.data[6] << 16)
                 | (rx_frame.data[7] << 24);
          return CAN_OK;
        }
      }
    }
  }
  
  return CAN_RX_TIMEOUT;
}


//=============================================================================
// VELOCITY CONTROL FUNCTIONS (matching your maxon.h)
//=============================================================================

// setTargetVelocity - Write target velocity and start motion
CANStatus setTargetVelocity(int velocity) {
  CANStatus status;
  
  // 1) Write Target Velocity (0x60FF)
  status = SDO_Write(TARGET_VELOCITY, 0x00, (uint32_t)velocity);
  if (status != CAN_OK) {
    Serial.print("Failed to set velocity: ");
    Serial.println(status);
    return status;
  }
  
  // 2) Write Controlword = 15 (enable & start)
  status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  if (status != CAN_OK) {
    Serial.print("Failed to enable: ");
    Serial.println(status);
    return status;
  }
  
  Serial.print("✓ Set velocity to ");
  Serial.print(velocity);
  Serial.println(" rpm");
  
  return CAN_OK;
}

// haltMovement - Stop motor using halt controlword
CANStatus haltMovement() {
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_HALT);
  
  if (status == CAN_OK) {
    Serial.println("✓ Motor halted");
  } else {
    Serial.print("Failed to halt: ");
    Serial.println(status);
  }
  
  return status;
}

// Read actual velocity
CANStatus getVelocityActualValueAveraged(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(VELOCITY_ACTUAL_VALUE_AVG, 0x01, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;  // Treat as signed
  }
  return status;
}

//=============================================================================
// MOTOR CONTROL FUNCTIONS (matching maxon.h interface)
//=============================================================================

// shutdown() - Send controlword = 6 (shutdown)
CANStatus shutdown() {
  Serial.println("\n--- shutdown() ---");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_SHUTDOWN);
  return status;
}

// enable() - Send controlword = 15 (switch on & enable)
CANStatus enable() {
  Serial.println("\n--- enable() ---");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  return status;
}

// setOperationMode(mode)
CANStatus setOperationMode(int mode) {
  Serial.println("\n--- setOperationMode() ---");
  CANStatus status = SDO_Write(OPERATION_MODE, 0x00, mode);
  return status;
}

// setMaxProfileVelocity(value)
CANStatus setMaxProfileVelocity(int value) {
  Serial.println("\n--- setMaxProfileVelocity() ---");
  CANStatus status = SDO_Write(MAX_PROFILE_VELOCITY, 0x00, value);
  return status;
}

// setProfileAcceleration(value)
CANStatus setProfileAcceleration(int value) {
  Serial.println("\n--- setProfileAcceleration() ---");
  CANStatus status = SDO_Write(PROFILE_ACCELERATION, 0x00, value);
  return status;
}

// setProfileDeceleration(value)
CANStatus setProfileDeceleration(int value) {
  Serial.println("\n--- setProfileDeceleration() ---");
  CANStatus status = SDO_Write(PROFILE_DECELERATION, 0x00, value);
  return status;
}

// setQuickStopDeceleration(value)
CANStatus setQuickStopDeceleration(int value) {
  Serial.println("\n--- setQuickStopDeceleration() ---");
  CANStatus status = SDO_Write(QUICK_STOP_DECELERATION, 0x00, value);
  return status;
}

// setMotionProfileType(value)
CANStatus setMotionProfileType(int value) {
  Serial.println("\n--- setMotionProfileType() ---");
  CANStatus status = SDO_Write(MOTION_PROFILE_TYPE, 0x00, value);
  return status;
}

//=============================================================================
// MOTOR CONFIGURATION
//=============================================================================

void MotorConfig() {
  delay(5000);
  Serial.println("\n========================================");
  Serial.println("    MOTOR CONFIGURATION SEQUENCE");
  Serial.println("========================================");
  
  CANStatus status;
  int failed_count = 0;
  
  // Step 1: Set operation mode to Profile Velocity (3)
  status = setOperationMode(PROFILE_VELOCITY_MODE);
  if (status != CAN_OK) failed_count++;
  
  // Step 2: Set max profile velocity
  status = setMaxProfileVelocity(1800);
  if (status != CAN_OK) failed_count++;
  
  // Step 3: Set profile acceleration
  status = setProfileAcceleration(800);
  if (status != CAN_OK) failed_count++;
  
  // Step 4: Set profile deceleration
  status = setProfileDeceleration(800);
  if (status != CAN_OK) failed_count++;
  
  // Step 5: Set quick stop deceleration
  status = setQuickStopDeceleration(800);
  if (status != CAN_OK) failed_count++;
  
  // Step 6: Set motion profile type
  status = setMotionProfileType(0);
  if (status != CAN_OK) failed_count++;
  
  // Step 7: shutdown
  status = shutdown();
  if (status != CAN_OK) failed_count++;
  
  // Step 8: enable again
  status = enable();
  if (status != CAN_OK) failed_count++;
  
  Serial.println("\n========================================");
  Serial.println("    CONFIGURATION COMPLETE");
  Serial.println("========================================");
  Serial.print("Total commands: 8\n");
  Serial.print("Failed: ");
  Serial.println(failed_count);
  Serial.print("Success: ");
  Serial.println(8 - failed_count);
  
  if (failed_count == 0) {
    Serial.println("\n✓ Motor configured successfully!");
  } else {
    Serial.println("\n✗ Some commands failed - check EPOS state");
  }
}

//=============================================================================
// ARDUINO SETUP & LOOP
//=============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n========================================");
  Serial.println("  Minimal Velocity Control Test");
  Serial.println("========================================");
  
  if (!CAN_Init()) {
    Serial.println("✗ FAILED to initialize CAN!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ CAN initialized at 500 kbit/s");
  Serial.println("  TX: GPIO 14, RX: GPIO 15\n");
  
  delay(1000);
  
  // Configure motor
  MotorConfig();

  delay(1000);
  
  Serial.println("Starting velocity cycling...\n");
}

void loop() {
  static int state = 0;
  int32_t actual_velocity;
  
  switch (state) {
    case 0:
      Serial.println("\n========== Speed: 0.5 m/s (267 rpm) ==========");
      setTargetVelocity(S0_5);
      break;
      
    case 1:
      Serial.println("\n========== Speed: 1.0 m/s (535 rpm) ==========");
      setTargetVelocity(S1);
      break;
      
    case 2:
      Serial.println("\n========== Speed: 1.5 m/s (802 rpm) ==========");
      setTargetVelocity(S1_5);
      break;
      
    case 3:
      Serial.println("\n========== HALT ==========");
      haltMovement();
      break;
  }
  
  // Wait and monitor actual velocity
  for (int i = 0; i < 5; i++) {
    delay(4000);
    
    if (getVelocityActualValueAveraged(&actual_velocity) == CAN_OK) {
      Serial.print("  Actual velocity: ");
      Serial.print(actual_velocity);
      Serial.println(" rpm");
    } else {
      Serial.println("  Failed to read velocity");
    }
  }
  
  // Move to next state
  state = (state + 1) % 4;
}
