/*
  Minimal Drive Motor Control with Fault Detection - Low Level CAN Implementation
  REFERENCE - EPOS 4 APPLICATION NOTES SECTION - 7.5 (Profile Velocity Mode)
  
  Hardware:
  - ESP32 CAN TX: GPIO 14
  - ESP32 CAN RX: GPIO 15
  - CAN transceiver connected to these pins
  - Maxon EPOS motor controller on CAN bus (Node ID = 1)
  
  What this does:
  1. Waits 5 seconds for motor/driver power-up
  2. Configures motor in Profile Velocity Mode with full error detection
  3. Cycles through different speed levels with fault monitoring:
     - 267 rpm (0.5 m/s)
     - 535 rpm (1.0 m/s)
     - 802 rpm (1.5 m/s)
     - Halt (stop)
  4. Logs all SDO abort codes, TWAI alerts, and ESP errors with diagnostics
  
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

// Operation modes
#define PROFILE_VELOCITY_MODE       3

// Controlword values
#define CONTROLWORD_SHUTDOWN        6     // 0x0006 - disable motor
#define CONTROLWORD_ENABLE          15    // 0x000F - enable and start
#define CONTROLWORD_HALT            271   // 0x010F - halt movement (bit 8 set)

// Motor speed levels
// Motor Speed Levels
#define S0_03 15   // 0.03 m/s
#define S0_05 26   // 0.05 m/s
#define S0_1 53    // 0.1 m/s
#define S0_2 106   // 0.2 m/s
#define S0_3 160   // 0.3 m/s
#define S0_5 267   // 0.5 m/s
#define S1 535     // 1 m/s
#define S1_5 802   // 1.5 m/s
#define S2 1070    // 2 m/s
#define S2_5 1337  // 2.5 m/s
#define S3 1604    // 3 m/s

// GPIO Pin Assignments
#define BUZZER 13
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35

// SDO response timeout (ms)
#define SDO_RESPONSE_TIMEOUT        10

// Motor power-up delay (ms)
#define MOTOR_POWERUP_DELAY         5000

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
// DIAGNOSTIC FUNCTIONS
//=============================================================================

// Diagnose SDO abort codes (CANopen standard + EPOS specific)
void diagnoseSDOAbort(uint32_t abort_code, uint16_t index) {
  Serial.print("\n┌─ SDO ABORT DIAGNOSTIC ─────────────────");
  Serial.print("\n│ Abort Code: 0x");
  Serial.println(abort_code, HEX);
  Serial.print("│ Object Index: 0x");
  Serial.println(index, HEX);
  Serial.println("│");
  
  switch (abort_code) {
    // Standard CANopen abort codes
    case 0x05030000:
      Serial.println("│ Error: Toggle bit not alternated");
      Serial.println("│ Cause: SDO protocol error");
      Serial.println("│ Action: Retry command, check CAN bus integrity");
      break;
      
    case 0x05040000:
      Serial.println("│ Error: SDO protocol timed out");
      Serial.println("│ Cause: EPOS didn't respond in time");
      Serial.println("│ Action: Check EPOS power, CAN connection, node ID");
      break;
      
    case 0x05040001:
      Serial.println("│ Error: Client/server command specifier not valid");
      Serial.println("│ Cause: Invalid SDO command byte");
      Serial.println("│ Action: Check SDO frame format (0x22 for write, 0x40 for read)");
      break;
      
    case 0x06010000:
      Serial.println("│ Error: Unsupported access to an object");
      Serial.println("│ Cause: Object cannot be accessed (read-only, write-only, etc.)");
      Serial.println("│ Action: Check EPOS documentation for object access rights");
      break;
      
    case 0x06010001:
      Serial.println("│ Error: Attempt to read a write-only object");
      Serial.println("│ Cause: Trying to read from write-only parameter");
      Serial.println("│ Action: Use correct access direction for this object");
      break;
      
    case 0x06010002:
      Serial.println("│ Error: Attempt to write a read-only object");
      Serial.println("│ Cause: Trying to write to read-only parameter");
      Serial.println("│ Action: Use correct access direction for this object");
      break;
      
    case 0x06020000:
      Serial.println("│ Error: Object does not exist in object dictionary");
      Serial.println("│ Cause: Invalid object index or not supported by this EPOS");
      Serial.println("│ Action: Check object index, verify EPOS firmware version");
      break;
      
    case 0x06040041:
      Serial.println("│ Error: Object cannot be mapped to PDO");
      Serial.println("│ Cause: This object doesn't support PDO mapping");
      Serial.println("│ Action: Use SDO access instead of PDO");
      break;
      
    case 0x06040042:
      Serial.println("│ Error: PDO length exceeded");
      Serial.println("│ Cause: Too many objects mapped to PDO");
      Serial.println("│ Action: Reduce PDO mapping or split into multiple PDOs");
      break;
      
    case 0x06040043:
      Serial.println("│ Error: General parameter incompatibility");
      Serial.println("│ Cause: Parameter conflict or invalid configuration");
      Serial.println("│ Action: Check parameter dependencies and valid ranges");
      break;
      
    case 0x06040047:
      Serial.println("│ Error: General internal incompatibility");
      Serial.println("│ Cause: Internal EPOS state prevents this operation");
      Serial.println("│ Action: Check EPOS state machine, may need reset");
      break;
      
    case 0x06060000:
      Serial.println("│ Error: Access failed due to hardware error");
      Serial.println("│ Cause: Hardware fault detected");
      Serial.println("│ Action: Check motor connections, power supply, encoder");
      break;
      
    case 0x06070010:
      Serial.println("│ Error: Data type does not match");
      Serial.println("│ Cause: Wrong data size for this object");
      Serial.println("│ Action: Check object data type (16-bit vs 32-bit)");
      break;
      
    case 0x06070012:
      Serial.println("│ Error: Data type length too high");
      Serial.println("│ Cause: Sending too many data bytes");
      Serial.println("│ Action: Use correct SDO command specifier (0x22 vs 0x23)");
      break;
      
    case 0x06070013:
      Serial.println("│ Error: Data type length too low");
      Serial.println("│ Cause: Not enough data bytes sent");
      Serial.println("│ Action: Use correct SDO command specifier");
      break;
      
    case 0x06090011:
      Serial.println("│ Error: Sub-index does not exist");
      Serial.println("│ Cause: Invalid subindex for this object");
      Serial.println("│ Action: Check object dictionary for valid subindices");
      break;
      
    case 0x06090030:
      Serial.println("│ Error: Value range exceeded");
      Serial.println("│ Cause: Parameter value out of valid range");
      Serial.println("│ Action: Check min/max limits in EPOS documentation");
      if (index == TARGET_VELOCITY) {
        Serial.println("│ Note: Target velocity may exceed configured limits");
        Serial.println("│       Check Max Profile Velocity (0x607F)");
      }
      break;
      
    case 0x06090031:
      Serial.println("│ Error: Value too high");
      Serial.println("│ Cause: Parameter exceeds maximum limit");
      Serial.println("│ Action: Reduce value or increase configured maximum");
      break;
      
    case 0x06090032:
      Serial.println("│ Error: Value too low");
      Serial.println("│ Cause: Parameter below minimum limit");
      Serial.println("│ Action: Increase value or decrease configured minimum");
      break;
      
    case 0x08000000:
      Serial.println("│ Error: General error");
      Serial.println("│ Cause: Unspecified error condition");
      Serial.println("│ Action: Check EPOS error register, may need reset");
      break;
      
    case 0x08000020:
      Serial.println("│ Error: Data cannot be transferred or stored");
      Serial.println("│ Cause: EPOS cannot process this command now");
      Serial.println("│ Action: Check EPOS state, wait for current operation to finish");
      break;
      
    case 0x08000021:
      Serial.println("│ Error: Data cannot be transferred (local control)");
      Serial.println("│ Cause: EPOS in local control mode");
      Serial.println("│ Action: Disable local control, switch to remote mode");
      break;
      
    case 0x08000022:
      Serial.println("│ Error: Data cannot be transferred (device state)");
      Serial.println("│ Cause: EPOS not in correct state for this operation");
      Serial.println("│ Action: Check state machine, may need to enable device first");
      if (index == TARGET_VELOCITY || index == CONTROLWORD) {
        Serial.println("│ Note: Motor may not be enabled - try shutdown() then enable()");
      }
      break;
      
    // EPOS-specific abort codes
    case 0x0F00FFC0:
      Serial.println("│ Error: EPOS password protected");
      Serial.println("│ Cause: Object requires password to access");
      Serial.println("│ Action: Unlock EPOS using password command");
      break;
      
    default:
      Serial.println("│ Error: Unknown abort code");
      Serial.println("│ Cause: Undocumented or vendor-specific error");
      Serial.println("│ Action: Check EPOS manual, consider firmware update");
      break;
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

// Diagnose TWAI alerts
void diagnoseTWAIAlert(uint32_t alerts) {
  Serial.println("\n┌─ TWAI ALERT DIAGNOSTIC ────────────────");
  Serial.print("│ Alert Flags: 0x");
  Serial.println(alerts, HEX);
  Serial.println("│");
  
  if (alerts & TWAI_ALERT_TX_IDLE) {
    Serial.println("│ ℹ TX Idle: No transmission in progress");
  }
  
  if (alerts & TWAI_ALERT_TX_SUCCESS) {
    Serial.println("│ ✓ TX Success: Frame transmitted successfully");
  }
  
  if (alerts & TWAI_ALERT_RX_DATA) {
    Serial.println("│ ℹ RX Data: Frame received");
  }
  
  if (alerts & TWAI_ALERT_BELOW_ERR_WARN) {
    Serial.println("│ ✓ Below Error Warning: Bus healthy");
  }
  
  if (alerts & TWAI_ALERT_ERR_ACTIVE) {
    Serial.println("│ ⚠ Error Active: Error counters elevated");
    Serial.println("│ Cause: Occasional transmission errors");
    Serial.println("│ Action: Check CAN bus termination, wiring, baud rate");
  }
  
  if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    Serial.println("│ ⚠ Recovery in Progress: Attempting bus-off recovery");
    Serial.println("│ Cause: Controller was in bus-off, now recovering");
    Serial.println("│ Action: Wait for recovery, check bus health");
  }
  
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    Serial.println("│ ✓ Bus Recovered: Successfully recovered from bus-off");
  }
  
  if (alerts & TWAI_ALERT_ARB_LOST) {
    Serial.println("│ ⚠ Arbitration Lost: Lost bus arbitration");
    Serial.println("│ Cause: Another node transmitted at same time with higher priority");
    Serial.println("│ Action: Normal in multi-node networks, will retry automatically");
  }
  
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
    Serial.println("│ ⚠ Above Error Warning: Error counters high (>96)");
    Serial.println("│ Cause: Frequent bus errors, nearing bus-off");
    Serial.println("│ Action: URGENT - Check termination, baud rate, wiring quality");
  }
  
  if (alerts & TWAI_ALERT_BUS_ERROR) {
    Serial.println("│ ✗ Bus Error: CAN protocol error detected");
    Serial.println("│ Cause: Bit/stuff/CRC/form/ACK error");
    Serial.println("│ Action: Check bus quality, termination, EMI");
  }
  
  if (alerts & TWAI_ALERT_TX_FAILED) {
    Serial.println("│ ✗ TX Failed: Transmission failed after retries");
    Serial.println("│ Cause: No ACK from any node, or bus-off");
    Serial.println("│ Action: Check if other nodes present, check EPOS power");
  }
  
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("│ ✗ RX Queue Full: Receive buffer overflow");
    Serial.println("│ Cause: Not reading frames fast enough");
    Serial.println("│ Action: Increase RX queue size or read more frequently");
  }
  
  if (alerts & TWAI_ALERT_ERR_PASS) {
    Serial.println("│ ✗ Error Passive: Error counters critical (>127)");
    Serial.println("│ Cause: Many bus errors, one step from bus-off");
    Serial.println("│ Action: CRITICAL - Fix bus issues immediately");
    Serial.println("│        Check: termination resistors (120Ω), baud rate match,");
    Serial.println("│               cable quality, EMI shielding, ground loops");
  }
  
  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.println("│ ✗✗✗ BUS-OFF: Controller shut down due to errors");
    Serial.println("│ Cause: Error count exceeded 255, bus completely unreliable");
    Serial.println("│ Action: CRITICAL FAILURE");
    Serial.println("│   1. Check CAN transceiver power and connections");
    Serial.println("│   2. Verify 120Ω termination at both bus ends");
    Serial.println("│   3. Confirm baud rate matches all nodes (500 kbit/s)");
    Serial.println("│   4. Check CANH/CANL not swapped or shorted");
    Serial.println("│   5. Measure bus voltage (recessive: 2.5V, dominant: 1.5V/3.5V)");
    Serial.println("│   6. Will auto-recover, but root cause MUST be fixed");
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

// Diagnose ESP_ERR codes
void diagnoseESPError(esp_err_t err, const char* operation) {
  Serial.print("\n┌─ ESP ERROR DIAGNOSTIC ─────────────────");
  Serial.print("\n│ Operation: ");
  Serial.println(operation);
  Serial.print("│ Error Code: ");
  Serial.println(err);
  Serial.println("│");
  
  switch (err) {
    case ESP_OK:
      Serial.println("│ Status: Success");
      break;
      
    case ESP_ERR_TIMEOUT:
      Serial.println("│ Error: Timeout");
      Serial.println("│ Cause: Operation didn't complete in time");
      if (strcmp(operation, "twai_transmit") == 0) {
        Serial.println("│ Details: TX queue full or bus stuck");
        Serial.println("│ Action: Check if EPOS responding, verify bus not saturated");
      } else if (strcmp(operation, "twai_receive") == 0) {
        Serial.println("│ Details: No frame received within timeout");
        Serial.println("│ Action: Normal if no traffic, check if EPOS responding");
      }
      break;
      
    case ESP_ERR_INVALID_STATE:
      Serial.println("│ Error: Invalid State");
      Serial.println("│ Cause: TWAI driver not started or in wrong mode");
      Serial.println("│ Action: Check twai_start() was called, not in bus-off");
      break;
      
    case ESP_ERR_INVALID_ARG:
      Serial.println("│ Error: Invalid Argument");
      Serial.println("│ Cause: Bad parameter passed to TWAI function");
      Serial.println("│ Action: Check message structure (ID, DLC, data)");
      break;
      
    case ESP_FAIL:
      Serial.println("│ Error: General Failure");
      Serial.println("│ Cause: Unspecified error in TWAI driver");
      Serial.println("│ Action: Check TWAI initialization, may need restart");
      break;
      
    case ESP_ERR_NOT_SUPPORTED:
      Serial.println("│ Error: Not Supported");
      Serial.println("│ Cause: Feature not available on this ESP32 variant");
      Serial.println("│ Action: Check ESP32 model supports TWAI/CAN");
      break;
      
    default:
      Serial.print("│ Error: Unknown (");
      Serial.print(err);
      Serial.println(")");
      Serial.println("│ Action: Check ESP-IDF documentation");
      break;
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

//=============================================================================
// LOW-LEVEL CAN FUNCTIONS (down to TWAI API)
//=============================================================================

// Initialize CAN hardware with alert monitoring
bool CAN_Init() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN, 
    CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  
  // Enable all alerts for comprehensive monitoring
  g_config.alerts_enabled = TWAI_ALERT_ALL;
  
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    diagnoseESPError(err, "twai_driver_install");
    return false;
  }
  
  err = twai_start();
  if (err != ESP_OK) {
    diagnoseESPError(err, "twai_start");
    return false;
  }
  
  return true;
}

// Check for TWAI alerts
void checkTWAIAlerts() {
  uint32_t alerts;
  esp_err_t err = twai_read_alerts(&alerts, 0);  // Non-blocking check
  
  if (err == ESP_OK && alerts != 0) {
    diagnoseTWAIAlert(alerts);
    
    // Auto-recovery from bus-off
    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("⚠ Initiating bus-off recovery...");
      twai_initiate_recovery();
    }
  }
}

// Send a CAN frame with error diagnostics
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
  } else {
    diagnoseESPError(result, "twai_transmit");
    checkTWAIAlerts();  // Check for bus issues
    
    if (result == ESP_ERR_TIMEOUT) {
      return CAN_TX_TIMEOUT;
    } else {
      return CAN_ERROR;
    }
  }
}

// Receive a CAN frame with error diagnostics
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
    diagnoseESPError(result, "twai_receive");
    return CAN_ERROR;
  }
}

//=============================================================================
// CANOPEN SDO FUNCTIONS
//=============================================================================

// Send SDO Download (write) and wait for response with full diagnostics
CANStatus SDO_Write(uint16_t index, uint8_t subindex, uint32_t value) {
  SimpleCANFrame tx_frame;
  
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x22;
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
    Serial.print("✗ TX failed for object 0x");
    Serial.println(index, HEX);
    return status;
  }
  
  // Wait for SDO response
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
          diagnoseSDOAbort(abort_code, index);
          return CAN_SDO_ABORT;
        }
        
        // Download response (0x60)
        if (cmd == 0x60) {
          return CAN_OK;
        }
      }
    }
  }
  
  Serial.print("✗ SDO response timeout for object 0x");
  Serial.print(index, HEX);
  Serial.println();
  Serial.println("│ Possible causes:");
  Serial.println("│   - EPOS not powered or not responding");
  Serial.println("│   - Wrong node ID (check NODE_ID = 1)");
  Serial.println("│   - CAN bus disconnected");
  Serial.println("│   - EPOS in fault state");
  checkTWAIAlerts();
  
  return CAN_RX_TIMEOUT;
}

// Send SDO Upload (read) and wait for response
CANStatus SDO_Read(uint16_t index, uint8_t subindex, uint32_t* value) {
  SimpleCANFrame tx_frame;
  
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x40;
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = (index >> 8) & 0xFF;
  tx_frame.data[3] = subindex;
  tx_frame.data[4] = 0x00;
  tx_frame.data[5] = 0x00;
  tx_frame.data[6] = 0x00;
  tx_frame.data[7] = 0x00;
  
  CANStatus status = CAN_Send(&tx_frame, 4);
  if (status != CAN_OK) {
    return status;
  }
  
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < SDO_RESPONSE_TIMEOUT) {
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      if (rx_frame.id == SDO_RX) {
        uint8_t cmd = rx_frame.data[0];
        
        if (cmd == 0x80) {
          uint32_t abort_code = rx_frame.data[4] 
                              | (rx_frame.data[5] << 8)
                              | (rx_frame.data[6] << 16)
                              | (rx_frame.data[7] << 24);
          diagnoseSDOAbort(abort_code, index);
          return CAN_SDO_ABORT;
        }
        
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
// MOTOR CONTROL FUNCTIONS
//=============================================================================

CANStatus shutdown() {
  Serial.print("→ Shutdown... ");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_SHUTDOWN);
  if (status == CAN_OK) {
    Serial.println("✓");
  }
  return status;
}

CANStatus enable() {
  Serial.print("→ Enable... ");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  if (status == CAN_OK) {
    Serial.println("✓");
  }
  return status;
}

CANStatus setOperationMode(int mode) {
  Serial.print("→ Set operation mode ");
  Serial.print(mode);
  Serial.print("... ");
  CANStatus status = SDO_Write(OPERATION_MODE, 0x00, mode);
  if (status == CAN_OK) {
    Serial.println("✓");
  }
  return status;
}

CANStatus setTargetVelocity(int velocity) {
  Serial.print("→ Set velocity ");
  Serial.print(velocity);
  Serial.print(" rpm... ");
  
  CANStatus status = SDO_Write(TARGET_VELOCITY, 0x00, (uint32_t)velocity);
  if (status != CAN_OK) {
    Serial.println("✗");
    return status;
  }
  
  status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  if (status == CAN_OK) {
    Serial.println("✓");
  } else {
    Serial.println("✗");
  }
  
  return status;
}

CANStatus haltMovement() {
  Serial.print("→ Halt... ");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_HALT);
  if (status == CAN_OK) {
    Serial.println("✓");
  }
  return status;
}

CANStatus getTargetVelocity(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(TARGET_VELOCITY, 0x00, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;
  }
  return status;
}

//=============================================================================
// MOTOR CONFIGURATION
//=============================================================================

bool MotorConfig() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     MOTOR CONFIGURATION SEQUENCE       ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  if (setOperationMode(PROFILE_VELOCITY_MODE) != CAN_OK) {
    Serial.println("✗ FATAL: Could not set operation mode");
    return false;
  }
  
  Serial.print("→ Set max velocity 1800... ");
  if (SDO_Write(MAX_PROFILE_VELOCITY, 0x00, 1800) != CAN_OK) {
    Serial.println("✗");
    return false;
  }
  Serial.println("✓");
  
  Serial.print("→ Set acceleration 800... ");
  if (SDO_Write(PROFILE_ACCELERATION, 0x00, 800) != CAN_OK) {
    Serial.println("✗");
    return false;
  }
  Serial.println("✓");
  
  Serial.print("→ Set deceleration 800... ");
  if (SDO_Write(PROFILE_DECELERATION, 0x00, 800) != CAN_OK) {
    Serial.println("✗");
    return false;
  }
  Serial.println("✓");
  
  if (shutdown() != CAN_OK) {
    return false;
  }
  
  if (enable() != CAN_OK) {
    return false;
  }
  
  Serial.println("\n✓ Motor configured successfully\n");
  return true;
}

//=============================================================================
// ARDUINO SETUP & LOOP
//=============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  Drive Motor with Fault Detection     ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  Serial.println("⏳ Waiting for motor/driver power-up...");
  Serial.print("   ");
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println("Ready!\n");
  
  Serial.println("→ Initializing CAN (500 kbit/s)...");
  if (!CAN_Init()) {
    Serial.println("✗ FATAL: CAN initialization failed");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("✓ CAN initialized (TX: GPIO 14, RX: GPIO 15)\n");
  
  if (!MotorConfig()) {
    Serial.println("\n✗ FATAL: Motor configuration failed");
    Serial.println("   System halted. Fix errors and reset.\n");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║     Starting Velocity Control Cycle   ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

void loop() {
  static int state = 0;
  int32_t actual_velocity;
  
  checkTWAIAlerts();  // Monitor bus health
  
  switch (state) {
    case 0:
      Serial.println("\n═══ Speed: 0.5 m/s (267 rpm) ═══");
      setTargetVelocity(S0_5);
      break;
      
    case 1:
      Serial.println("\n═══ Speed: 1.0 m/s (535 rpm) ═══");
      setTargetVelocity(S1);
      break;
      
    case 2:
      Serial.println("\n═══ Speed: 1.5 m/s (802 rpm) ═══");
      setTargetVelocity(S1_5);
      break;
      
    case 3:
      Serial.println("\n═══ HALT ═══");
      haltMovement();
      break;
  }
  
  // Monitor target velocity
  for (int i = 0; i < 5; i++) {
    delay(1000);
    
    if (getTargetVelocity(&actual_velocity) == CAN_OK) {
      Serial.print("  │ Target: ");
      Serial.print(actual_velocity);
      Serial.println(" rpm");
    } else {
      Serial.println("  │ ✗ Failed to read velocity");
      checkTWAIAlerts();
    }
  }
  
  state = (state + 1) % 4;
}
