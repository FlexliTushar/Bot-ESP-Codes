#define FIRMWARE_VERSION "d531"
/*
  Diverter State Machine Test Sketch
  ====================================
  Standalone test for diverter state machine control
  
  This sketch tests the diverter state machine by automatically toggling
  the demand direction between LEFT and RIGHT every 5 seconds.
*/

#include <SMS.h>
#include <Adafruit_MCP23X17.h>
#include "driver/twai.h"
#include "driver/gpio.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
using namespace std;

//=============================================================================
// CAN HARDWARE CONFIGURATION
//=============================================================================
#define CAN_TX_PIN                  GPIO_NUM_14
#define CAN_RX_PIN                  GPIO_NUM_15
#define NODE_ID                     1

// CANopen COB-IDs for node 1
#define SDO_TX                      (0x600 + NODE_ID)  // Client->Server (our requests)
#define SDO_RX                      (0x580 + NODE_ID)  // Server->Client (EPOS responses)

//=============================================================================
// EPOS OBJECT DICTIONARY INDICES
//=============================================================================
#define CONTROLWORD                 0x6040
#define OPERATION_MODE              0x6060
#define TARGET_VELOCITY             0x60FF
#define VELOCITY_ACTUAL_VALUE_AVG   0x30D3
#define MAX_PROFILE_VELOCITY        0x607F
#define PROFILE_ACCELERATION        0x6083
#define PROFILE_DECELERATION        0x6084
#define FOLLOWING_ERROR_WINDOW      0x6065

//=============================================================================
// MOTOR CONTROL CONSTANTS
//=============================================================================
// Operation modes
#define PROFILE_VELOCITY_MODE       3

// Controlword values
#define CONTROLWORD_SHUTDOWN        6     // 0x0006 - disable motor
#define CONTROLWORD_ENABLE          15    // 0x000F - enable and start
#define CONTROLWORD_HALT            271   // 0x010F - halt movement (bit 8 set)

// Motor speed levels (RPM)
#define S0_03         15    // 0.03 m/s
#define S0_05         26    // 0.05 m/s
#define S0_1          53    // 0.1 m/s
#define S0_2          106   // 0.2 m/s
#define S0_3          160   // 0.3 m/s
#define S0_5          267   // 0.5 m/s
#define S1            535   // 1.0 m/s
#define S1_5          802   // 1.5 m/s
#define S2            1070  // 2.0 m/s
#define S2_5          1337  // 2.5 m/s
#define S3            1604  // 3.0 m/s

//=============================================================================
// TIMING CONSTANTS
//=============================================================================
#define SDO_RESPONSE_TIMEOUT        10    // SDO response timeout (ms)
#define MOTOR_POWERUP_DELAY         5000  // Motor power-up delay (ms)

// GPIO Pin Assignments
#define BUZZER 13
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define PANEL_LED_BUTTON 7
#define EMG_PIN 12
#define AFC_PIN_1 19
#define AFC_PIN_2 18

#define ENABLE 1
#define DISABLE 0

// CAN operation status codes
enum CANStatus {
  CAN_OK = 0,
  CAN_TX_TIMEOUT,
  CAN_RX_TIMEOUT,
  CAN_SDO_ABORT,
  CAN_ERROR
};

// Health monitoring
enum HealthStatus {
  SAFE,
  WARNING,
  CRITICAL
};

// Column detection state machine variables
enum WTMColumnState {
  WTM_BETWEEN_COLUMNS,  // In 00 zone between columns
  WTM_IN_COLUMN         // On a column (01, 10, or 11)
};

enum DriveMotorState {
    DRIVE_MOTOR_STOP = 1,
    DRIVE_MOTOR_STOPPING = 2,
    DRIVE_MOTOR_RUNNING = 3,
    DRIVE_MOTOR_SWEEPING_COLUMN = 4,
    DRIVE_MOTOR_ERROR = 5
};

enum DriveMotorDirection {
    DRIVE_MOTOR_FORWARD = 1,
    DRIVE_MOTOR_REVERSE = -1
};

enum DriveMotorStateOutput {
    DRIVE_MOTOR_STATE_NO_CHANGE = 0,
    DRIVE_MOTOR_STATE_CHANGED = 1,
    DRIVE_MOTOR_UNKNOWN_STATE = 2
};

enum DiverterState : uint8_t {
    DIVERTER_LEFT = 1,
    DIVERTER_RIGHT = 2,
    DIVERTER_SWITCHING = 3,
    DIVERTER_ERROR = 4,
    DIVERTER_STOP = 5
};

enum DiverterDirection : uint8_t {
    DIVERTER_DIRECTION_LEFT = 1,
    DIVERTER_DIRECTION_RIGHT = 2,
    DIVERTER_DIRECTION_UNKNOWN = 0
};

enum DiverterPosition : uint8_t {
    DIVERTER_POSITION_LEFT = 1,
    DIVERTER_POSITION_RIGHT = 2,
    DIVERTER_POSITION_INTERIM = 3,
    DIVERTER_POSITION_UNKNOWN = 0
};

enum DiverterStateChangeOptions : uint8_t {
    DIVERTER_STATE_NO_CHANGE = 0,
    DIVERTER_STATE_CHANGED = 1,
    DIVERTER_UNEXPECTED_CONDITION = 2
};

//=============================================================================
// STRUCTURES
//=============================================================================

// Simple CAN frame structure (maps to TWAI message)
struct SimpleCANFrame {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
  bool     extended;
  bool     rtr;
};

SemaphoreHandle_t xDebugLogMutex;

//=============================================================================
// DIAGNOSTIC FUNCTIONS
//=============================================================================

// Diagnose SDO abort codes (CANopen standard + EPOS specific)
void diagnoseSDOAbort(uint32_t abort_code, uint16_t index) {
  String log_msg = "SDO_ABORT:0x" + String(abort_code, HEX) + "@0x" + String(index, HEX);
  add_log(log_msg);
  
  add_log("SDO ABORT DIAGNOSTIC - Abort Code: 0x" + String(abort_code, HEX) + ", Object Index: 0x" + String(index, HEX));
  // Serial.print("\n│ Abort Code: 0x");
  // Serial.println(abort_code, HEX);
  // Serial.print("│ Object Index: 0x");
  // Serial.println(index, HEX);
  // Serial.println("│");
  
  switch (abort_code) {
    // Standard CANopen abort codes
    case 0x05030000:
      add_log("│ Error: Toggle bit not alternated");
      add_log("│ Cause: SDO protocol error");
      add_log("│ Action: Retry command, check CAN bus integrity");
      break;
      
    case 0x05040000:
      add_log("│ Error: SDO protocol timed out");
      add_log("│ Cause: EPOS didn't respond in time");
      add_log("│ Action: Check EPOS power, CAN connection, node ID");
      break;
      
    case 0x05040001:
      add_log("│ Error: Client/server command specifier not valid");
      add_log("│ Cause: Invalid SDO command byte");
      add_log("│ Action: Check SDO frame format (0x22 for write, 0x40 for read)");
      break;
      
    case 0x06010000:
      add_log("│ Error: Unsupported access to an object");
      add_log("│ Cause: Object cannot be accessed (read-only, write-only, etc.)");
      add_log("│ Action: Check EPOS documentation for object access rights");
      break;
      
    case 0x06010001:
      add_log("│ Error: Attempt to read a write-only object");
      add_log("│ Cause: Trying to read from write-only parameter");
      add_log("│ Action: Use correct access direction for this object");
      break;
      
    case 0x06010002:
      add_log("│ Error: Attempt to write a read-only object");
      add_log("│ Cause: Trying to write to read-only parameter");
      add_log("│ Action: Use correct access direction for this object");
      break;
      
    case 0x06020000:
      add_log("│ Error: Object does not exist in object dictionary");
      add_log("│ Cause: Invalid object index or not supported by this EPOS");
      add_log("│ Action: Check object index, verify EPOS firmware version");
      break;
      
    case 0x06040041:
      add_log("│ Error: Object cannot be mapped to PDO");
      add_log("│ Cause: This object doesn't support PDO mapping");
      add_log("│ Action: Use SDO access instead of PDO");
      break;
      
    case 0x06040042:
      add_log("│ Error: PDO length exceeded");
      add_log("│ Cause: Too many objects mapped to PDO");
      add_log("│ Action: Reduce PDO mapping or split into multiple PDOs");
      break;
      
    case 0x06040043:
      add_log("│ Error: General parameter incompatibility");
      add_log("│ Cause: Parameter conflict or invalid configuration");
      add_log("│ Action: Check parameter dependencies and valid ranges");
      break;
      
    case 0x06040047:
      add_log("│ Error: General internal incompatibility");
      add_log("│ Cause: Internal EPOS state prevents this operation");
      add_log("│ Action: Check EPOS state machine, may need reset");
      break;
      
    case 0x06060000:
      add_log("│ Error: Access failed due to hardware error");
      add_log("│ Cause: Hardware fault detected");
      add_log("│ Action: Check motor connections, power supply, encoder");
      break;
      
    case 0x06070010:
      add_log("│ Error: Data type does not match");
      add_log("│ Cause: Wrong data size for this object");
      add_log("│ Action: Check object data type (16-bit vs 32-bit)");
      break;
      
    case 0x06070012:
      add_log("│ Error: Data type length too high");
      add_log("│ Cause: Sending too many data bytes");
      add_log("│ Action: Use correct SDO command specifier (0x22 vs 0x23)");
      break;
      
    case 0x06070013:
      add_log("│ Error: Data type length too low");
      add_log("│ Cause: Not enough data bytes sent");
      add_log("│ Action: Use correct SDO command specifier");
      break;
      
    case 0x06090011:
      add_log("│ Error: Sub-index does not exist");
      add_log("│ Cause: Invalid subindex for this object");
      add_log("│ Action: Check object dictionary for valid subindices");
      break;
      
    case 0x06090030:
      add_log("│ Error: Value range exceeded");
      add_log("│ Cause: Parameter value out of valid range");
      add_log("│ Action: Check min/max limits in EPOS documentation");
      if (index == TARGET_VELOCITY) {
        add_log("│ Note: Target velocity may exceed configured limits");
        add_log("│       Check Max Profile Velocity (0x607F)");
      }
      break;
      
    case 0x06090031:
      add_log("│ Error: Value too high");
      add_log("│ Cause: Parameter exceeds maximum limit");
      add_log("│ Action: Reduce value or increase configured maximum");
      break;
      
    case 0x06090032:
      add_log("│ Error: Value too low");
      add_log("│ Cause: Parameter below minimum limit");
      add_log("│ Action: Increase value or decrease configured minimum");
      break;
      
    case 0x08000000:
      add_log("│ Error: General error");
      add_log("│ Cause: Unspecified error condition");
      add_log("│ Action: Check EPOS error register, may need reset");
      break;
      
    case 0x08000020:
      add_log("│ Error: Data cannot be transferred or stored");
      add_log("│ Cause: EPOS cannot process this command now");
      add_log("│ Action: Check EPOS state, wait for current operation to finish");
      break;
      
    case 0x08000021:
      add_log("│ Error: Data cannot be transferred (local control)");
      add_log("│ Cause: EPOS in local control mode");
      add_log("│ Action: Disable local control, switch to remote mode");
      break;
      
    case 0x08000022:
      add_log("│ Error: Data cannot be transferred (device state)");
      add_log("│ Cause: EPOS not in correct state for this operation");
      add_log("│ Action: Check state machine, may need to enable device first");
      if (index == TARGET_VELOCITY || index == CONTROLWORD) {
        add_log("│ Note: Motor may not be enabled - try shutdown() then enable()");
      }
      break;
      
    // EPOS-specific abort codes
    case 0x0F00FFC0:
      add_log("│ Error: EPOS password protected");
      add_log("│ Cause: Object requires password to access");
      add_log("│ Action: Unlock EPOS using password command");
      break;
      
    default:
      add_log("│ Error: Unknown abort code");
      add_log("│ Cause: Undocumented or vendor-specific error");
      add_log("│ Action: Check EPOS manual, consider firmware update");
      break;
  }
  
  add_log("└────────────────────────────────────────\n");
}

// Diagnose TWAI alerts
void diagnoseTWAIAlert(uint32_t alerts) {
  // Log critical alerts only
  if (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR)) {
    add_log("TWAI_ALERT:0x" + String(alerts, HEX));
  }
  
  add_log("TWAI ALERT DIAGNOSTIC - Alert Flags: 0x" + String(alerts, HEX));
  // Serial.print("│ Alert Flags: 0x");
  // Serial.println(alerts, HEX);
  // Serial.println("│");
  
  if (alerts & TWAI_ALERT_TX_IDLE) {
    add_log("│ ℹ TX Idle: No transmission in progress");
  }
  
  if (alerts & TWAI_ALERT_TX_SUCCESS) {
    add_log("│ ✓ TX Success: Frame transmitted successfully");
  }
  
  if (alerts & TWAI_ALERT_RX_DATA) {
    add_log("│ ℹ RX Data: Frame received");
  }
  
  if (alerts & TWAI_ALERT_BELOW_ERR_WARN) {
    add_log("│ ✓ Below Error Warning: Bus healthy");
  }
  
  if (alerts & TWAI_ALERT_ERR_ACTIVE) {
    add_log("│ ⚠ Error Active: Error counters elevated");
    add_log("│ Cause: Occasional transmission errors");
    add_log("│ Action: Check CAN bus termination, wiring, baud rate");
  }
  
  if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    add_log("│ ⚠ Recovery in Progress: Attempting bus-off recovery");
    add_log("│ Cause: Controller was in bus-off, now recovering");
    add_log("│ Action: Wait for recovery, check bus health");
  }
  
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    add_log("│ ✓ Bus Recovered: Successfully recovered from bus-off");
  }
  
  if (alerts & TWAI_ALERT_ARB_LOST) {
    add_log("│ ⚠ Arbitration Lost: Lost bus arbitration");
    add_log("│ Cause: Another node transmitted at same time with higher priority");
    add_log("│ Action: Normal in multi-node networks, will retry automatically");
  }
  
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
    add_log("│ ⚠ Above Error Warning: Error counters high (>96)");
    add_log("│ Cause: Frequent bus errors, nearing bus-off");
    add_log("│ Action: URGENT - Check termination, baud rate, wiring quality");
  }
  
  if (alerts & TWAI_ALERT_BUS_ERROR) {
    add_log("│ ✗ Bus Error: CAN protocol error detected");
    add_log("│ Cause: Bit/stuff/CRC/form/ACK error");
    add_log("│ Action: Check bus quality, termination, EMI");
  }
  
  if (alerts & TWAI_ALERT_TX_FAILED) {
    add_log("│ ✗ TX Failed: Transmission failed after retries");
    add_log("│ Cause: No ACK from any node, or bus-off");
    add_log("│ Action: Check if other nodes present, check EPOS power");
  }
  
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    add_log("│ ✗ RX Queue Full: Receive buffer overflow");
    add_log("│ Cause: Not reading frames fast enough");
    add_log("│ Action: Increase RX queue size or read more frequently");
  }
  
  if (alerts & TWAI_ALERT_ERR_PASS) {
    add_log("│ ✗ Error Passive: Error counters critical (>127)");
    add_log("│ Cause: Many bus errors, one step from bus-off");
    add_log("│ Action: CRITICAL - Fix bus issues immediately");
    add_log("│        Check: termination resistors (120Ω), baud rate match,");
    add_log("│               cable quality, EMI shielding, ground loops");
  }
  
  if (alerts & TWAI_ALERT_BUS_OFF) {
    add_log("│ ✗✗✗ BUS-OFF: Controller shut down due to errors");
    add_log("│ Cause: Error count exceeded 255, bus completely unreliable");
    add_log("│ Action: CRITICAL FAILURE");
    add_log("│   1. Check CAN transceiver power and connections");
    add_log("│   2. Verify 120Ω termination at both bus ends");
    add_log("│   3. Confirm baud rate matches all nodes (500 kbit/s)");
    add_log("│   4. Check CANH/CANL not swapped or shorted");
    add_log("│   5. Measure bus voltage (recessive: 2.5V, dominant: 1.5V/3.5V)");
    add_log("│   6. Will auto-recover, but root cause MUST be fixed");
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

// Diagnose ESP_ERR codes
void diagnoseESPError(esp_err_t err, const char* operation) {
  if (err != ESP_OK) {
    add_log("ESP_ERR:" + String(err) + "@" + String(operation));
  }
  
  add_log("ESP ERROR DIAGNOSTIC - Operation: " + String(operation) + ", Err Code: " + String(err));
  // Serial.print("\n│ Operation: ");
  // Serial.println(operation);
  // Serial.print("│ Error Code: ");
  // Serial.println(err);
  // Serial.println("│");
  switch (err) {
    case ESP_OK:
      Serial.println("│ Status: Success");
      break;
      
    case ESP_ERR_TIMEOUT:
      add_log("│ Error: Timeout");
      add_log("│ Cause: Operation didn't complete in time");
      if (strcmp(operation, "twai_transmit") == 0) {
        add_log("│ Details: TX queue full or bus stuck");
        add_log("│ Action: Check if EPOS responding, verify bus not saturated");
      } else if (strcmp(operation, "twai_receive") == 0) {
        add_log("│ Details: No frame received within timeout");
        add_log("│ Action: Normal if no traffic, check if EPOS responding");
      }
      break;
      
    case ESP_ERR_INVALID_STATE:
      add_log("│ Error: Invalid State");
      add_log("│ Cause: TWAI driver not started or in wrong mode");
      add_log("│ Action: Check twai_start() was called, not in bus-off");
      break;
      
    case ESP_ERR_INVALID_ARG:
      add_log("│ Error: Invalid Argument");
      add_log("│ Cause: Bad parameter passed to TWAI function");
      add_log("│ Action: Check message structure (ID, DLC, data)");
      break;
      
    case ESP_FAIL:
      add_log("│ Error: General Failure");
      add_log("│ Cause: Unspecified error in TWAI driver");
      add_log("│ Action: Check TWAI initialization, may need restart");
      break;
      
    case ESP_ERR_NOT_SUPPORTED:
      add_log("│ Error: Not Supported");
      add_log("│ Cause: Feature not available on this ESP32 variant");
      add_log("│ Action: Check ESP32 model supports TWAI/CAN");
      break;
      
    default:
      add_log("Error: Unknown (" + String(err) + ")");
      add_log("│ Action: Check ESP-IDF documentation");
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
      add_log("⚠ BUS-OFF recovery");
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
    add_log("✗ TX failed 0x" + String(index, HEX));
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
  
  add_log("✗ SDO timeout 0x" + String(index, HEX));
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
  add_log("→ Shutdown...");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_SHUTDOWN);
  if (status == CAN_OK) {
    add_log("✓ Shutdown complete");
  }
  return status;
}

CANStatus enable() {
  add_log("→ Enable...");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  if (status == CAN_OK) {
    add_log("✓ Enable complete");
  }
  return status;
}

CANStatus setOperationMode(int mode) {
  add_log("→ Set operation mode " + String(mode));
  CANStatus status = SDO_Write(OPERATION_MODE, 0x00, mode);
  if (status == CAN_OK) {
    add_log("✓ Operation mode set");
  }
  return status;
}

CANStatus setTargetVelocity(int velocity) {
  add_log("→ Set velocity " + String(velocity) + " rpm");
  
  CANStatus status = SDO_Write(TARGET_VELOCITY, 0x00, (uint32_t)velocity);
  if (status != CAN_OK) {
    add_log("✗ Failed to set velocity");
    return status;
  }
  
  status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  if (status == CAN_OK) {
    add_log("✓ Velocity set");
  } else {
    add_log("✗ Failed to enable after velocity set");
  }
  
  return status;
}

CANStatus haltMovement() {
  add_log("→ Halt...");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_HALT);
  if (status == CAN_OK) {
    add_log("✓ Halt complete");
  }
  return status;
}

CANStatus getTargetVelocity(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(TARGET_VELOCITY, 0x00, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;
  } else {
    *velocity = -1;
  }
  return status;
}

CANStatus getVelocityActualValueAveraged(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(VELOCITY_ACTUAL_VALUE_AVG, 0x01, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;
  } else {
    *velocity = -1;
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
  add_log("Motor config start");
  
  // Additional delay for EPOS internal initialization
  Serial.println("→ Allowing EPOS internal boot time...");
  delay(500);
  
  // Step 1: Set operation mode
  Serial.println("→ Setting Profile Velocity Mode...");
  int retries = 3;
  CANStatus status = CAN_ERROR;
  
  for (int i = 0; i < retries; i++) {
    status = setOperationMode(PROFILE_VELOCITY_MODE);
    if (status == CAN_OK) break;
    Serial.println("   Retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ FATAL: Operation mode failed after retries");
    Serial.println("✗ FATAL: Could not set operation mode");
    return false;
  }
  delay(50);  // Allow EPOS to process mode change
  
  // Step 2: Set max velocity
  add_log("→ Set max velocity 1800");
  Serial.println("→ Setting max velocity to 1800 RPM...");
  if (SDO_Write(MAX_PROFILE_VELOCITY, 0x00, 1800) != CAN_OK) {
    add_log("✗ Max velocity failed");
    return false;
  }
  add_log("✓ Max velocity set");
  delay(50);
  
  // Step 3: Set acceleration
  add_log("→ Set acceleration 800");
  Serial.println("→ Setting acceleration to 800 RPM/s...");
  if (SDO_Write(PROFILE_ACCELERATION, 0x00, 800) != CAN_OK) {
    add_log("✗ Acceleration failed");
    return false;
  }
  add_log("✓ Acceleration set");
  delay(50);
  
  // Step 4: Set deceleration
  add_log("→ Set deceleration 800");
  Serial.println("→ Setting deceleration to 800 RPM/s...");
  if (SDO_Write(PROFILE_DECELERATION, 0x00, 800) != CAN_OK) {
    add_log("✗ Deceleration failed");
    return false;
  }
  add_log("✓ Deceleration set");
  delay(50);
  
  // Step 5: Shutdown sequence (transition to Ready to Switch On)
  Serial.println("→ Executing shutdown sequence...");
  for (int i = 0; i < retries; i++) {
    status = shutdown();
    if (status == CAN_OK) break;
    Serial.println("   Shutdown retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ Shutdown failed");
    return false;
  }
  delay(100);  // Critical: Allow EPOS state transition time
  
  // Step 6: Enable sequence (transition to Operation Enabled)
  Serial.println("→ Executing enable sequence...");
  for (int i = 0; i < retries; i++) {
    status = enable();
    if (status == CAN_OK) {
      delay(50);
      // Verify enable was successful by checking statusword
      uint32_t statusword = 0;
      if (SDO_Read(0x6041, 0x00, &statusword) == CAN_OK) {
        Serial.print("   Statusword: 0x");
        Serial.println(statusword, HEX);
        // Bit pattern for Operation Enabled: xxxx x0xx x010 0111 (0x0027 masked with 0x006F)
        if ((statusword & 0x006F) == 0x0027) {
          add_log("✓ Motor enabled and verified");
          Serial.println("   ✓ Motor in Operation Enabled state");
          break;
        } else {
          Serial.println("   ⚠ Motor not in expected state, retrying...");
          status = CAN_ERROR;  // Force retry
        }
      }
    }
    
    if (i < retries - 1) {
      Serial.println("   Enable retry " + String(i + 1) + "/" + String(retries));
      delay(200);  // Longer delay between enable retries
    }
  }
  
  if (status != CAN_OK) {
    add_log("✗ Enable failed");
    return false;
  }
  
  add_log("✓ Motor configured");
  Serial.println("\n✓ Motor configured successfully\n");
  return true;
}

// Constant variables
const String CODE_ID = "d541";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/log";
const String ENTITY_TYPE = "bot";

Adafruit_MCP23X17 mcp;
SMS sm;
Preferences prefs;

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

WTMColumnState wtm_column_state = WTM_BETWEEN_COLUMNS;

// Frame counts for each possible column state
int frameCount00 = 0;
int frameCount01 = 0;
int frameCount10 = 0;
int frameCount11 = 0;

// Confirmation variables
int exitConfirmationCount = 0;  // Count of consecutive 00 samples
const int EXIT_CONFIRMATION_THRESHOLD = 5;  // Need 5 consecutive 00s to confirm exit (relaxed from 10)

// Column tracking
String detectedColumnCode = "";  // The actual column code (01, 10, or 11)
String currentColumnCode = "";    // Current column code being read
String previousColumnCode = "";   // Previous column code that was detected

// Health monitoring thresholds
const float MAJORITY_SAFE_THRESHOLD = 80.0;      // > 80% is safe
const float MAJORITY_WARNING_THRESHOLD = 60.0;   // 60-80% is warning, < 60% is critical
const float MINORITY_SAFE_THRESHOLD = 20.0;      // < 20% is safe
const float MINORITY_WARNING_THRESHOLD = 40.0;   // 20-40% is warning, > 40% is critical
const int DEFAULT_COLUMN_FRAME_THRESHOLD = 5;    // Conservative threshold for column width
const int MIN_COLUMN_FRAMES_THRESHOLD = 3;       // Minimum frames to consider valid column (filter noise)
const int DEFAULT_EMPTY_SPACE_THRESHOLD = 20;    // Expected minimum frames for empty space (relaxed from 30)
const float EMPTY_SPACE_NOISE_THRESHOLD = 5.0;   // < 5% non-00 readings is safe for empty space

HealthStatus majorityHealthWTM = SAFE;
HealthStatus minorityHealthWTM = SAFE;
HealthStatus overallHealthWTM = SAFE;

// Empty space tracking for health monitoring
int emptySpaceFrameCount = 0;        // Frame count for current empty space (pure 00s)
int emptySpaceNoiseFrameCount = 0;   // Count of non-00 frames detected during empty space (for health)
int emptySpaceTotalFrameCount = 0;   // Total frames in empty space including noise
HealthStatus emptySpaceHealth = SAFE; // Health of empty space (should have minimal non-00 readings)
HealthStatus emptySpaceMajorityHealth = SAFE;  // Majority health (00 frames)
HealthStatus emptySpaceMinorityHealth = SAFE;  // Minority health (noise frames)
int totalEmptySpacesDetected = 0;    // Total empty spaces detected
int totalFramesBetweenSpaces = 0;

// Column completion flag and data - set when column successfully read
bool columnJustCompleted = false;      // Flag: true when column just finished reading
String lastCompletedColumnCode = "";   // The column code that was just read (01, 10, 11)
HealthStatus lastCompletedColumnHealth = SAFE;  // Health of the column that was just read

// Tracking variables for station-to-station segment
String previousStation = "X";
int uniqueColumnsDetectedInSegment = 0;  // Count of unique columns (01, 10, 11) between stations
int emptySpacesDetectedInSegment = 0;    // Count of 00 zones between stations

// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

bool traffic_permission = true;
bool current_assignment = true;
int permitted_edge_speed = S0_5;
bool column_detected_in_sweep = false;

int drive_motor_current_speed = 0;
int drive_motor_previous_set_speed = 0;
DriveMotorState drive_motor_current_state = DRIVE_MOTOR_STOP;
DriveMotorState drive_motor_previous_state = DRIVE_MOTOR_STOP;
DriveMotorDirection drive_motor_current_direction = DRIVE_MOTOR_FORWARD;
DriveMotorDirection drive_motor_previous_direction = DRIVE_MOTOR_FORWARD;
int drive_motor_sweep_speed = S0_05;
int drive_motor_set_speed = 0;
int drive_motor_direction_value = 1;
bool drive_motor_error_status = false;
int totalColumnsDetected = 0;

volatile bool firstTimeError = false;

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

TaskHandle_t readingSensorAndUpdateStateTask;
TaskHandle_t actuationTask;

bool check_diverter_permission = true;

// ==============================================================================
// DIVERTER STATE MACHINE DEFINITIONS
// ==============================================================================

bool _photoTransistorCurrentStateArray[3] = { 0, 0, 0 };       // Array to store current state of sensors, either 0 or 1 based on threshold during station reading
bool _photoTransistorPreviousStateArray[3] = { 0, 0, 0 };      // Array to store previous state of sensors, mainily to check if state has changed or not.

uint16_t _photoTransistorCurrentValueArray[3] = { 0, 0, 0 };   // Array to store the analog value sent by the sensor currently.

String _intermediateColumnCode = "000";                        // String to store intermediate column codes in String format, mainly to detect when bot is changing columns.
bool _readingInProcess = false;                                // Flag variable to keep track wether station reading has started and is in progress.
String _finalColumnCodeArray[3] = { "000", "000", "000" };     // Array used to store the 3 columns of the station in string format.
bool _rightStationDetected = false;                            // Flag variable to check if a station has been detected so that it can perform its next set of operations on the obtained data.
String _previousIntermediateColumnCode = "000";
String stationCode = "";      // String variable to store entire stationCode in string format
String currentStation = "X";  // String variable to store the station Id, after converting the station code read into stationId

String StationsArray[25] = { "000000000" };
unordered_map<string, string> STATION_CODE_TO_STATION_ID_MAP = {
  { "100011110", "D1" }, { "101110011", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, { "101011010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010101", "D9" }, { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100110100", "D13" }, { "100101100", "D14" }, { "100101110", "D15" }, { "100110001", "D16" }, { "100110010", "D17" }, { "100110011", "D18" }, { "110010100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }
};

unordered_map<string, bool> STATION_ID_TO_DIRECTION_MAP = {
  { "D1", 0 }, { "D2", 0 }, { "D3", 1 }, { "D4", 1 }, { "D5", 1 }, { "D6", 1 }, { "D7", 1 }, { "D9", 1 }, { "D10", 1 }, { "D11", 1 }, { "D12", 1 }, { "D13", 1 }, { "D14", 1 }, { "D15", 1 }, { "D16", 1 }, { "D17", 1 }, { "D18", 1 }, { "I1", 1 }, { "I11", 0 }, { "I12", 1 }
};

unordered_map<string, string> STATION_ID_TO_EXPECTED_STATION_ID_MAP = {
  { "D1", "I1" }, { "D2", "D3" }, { "D3", "D4" }, { "D4", "D5" }, { "D5", "D6" }, { "D6", "D7" }, { "D7", "D9" }, { "D9", "D10" }, { "D10", "D11" }, { "D11", "D12" }, { "D12", "D13" }, { "D13", "D14" }, { "D14", "D15" }, { "D15", "D16" }, { "D16", "D17" }, { "D17", "D18" }, { "D18", "D1" }, { "I1", "I12" }, { "I11", "D2" }, { "I12", "I11" }
};

unordered_map<string, int> STATION_ID_TO_SPEED_MAP = {
  { "D1", S1 }, { "D2", S2 }, { "D3", S2 }, { "D4", S2 }, { "D5", S2 }, { "D6", S2 }, { "D7", S2 }, { "D9", S2 }, { "D10", S2 }, { "D11", S2 }, { "D12", S2 }, { "D13", S2 }, { "D14", S2 }, { "D15", S2 }, { "D16", S2 }, { "D17", S2 }, { "D18", S2 }, { "I1", S0_3 }, { "I11", S1 }, { "I12", S0_05 }
};

String expectedStation = "X";
bool localisationFlag = true;
volatile bool errorMode = false;
volatile int errorCode = -1;

int _intermediateColumnFramesCount = 0;
int _instantaneousFramesCount = 0;
uint16_t _instantaneousIntensityMaxArray[3] = { 0 };
uint16_t _instantaneousIntensityMinArray[3] = { 1023 };
unsigned long stationReadStartTime;
int columnNo = 0;

const static int SR_D1 = 36;  // ESP32 pin attached to Photo diode 1
const static int SR_D2 = 39;  // ESP32 pin attached to Photo diode 2
const static int SR_D3 = 34;  // ESP32 pin attached to Photo diode 3
const static uint16_t PT_UPPER_THR_LIMIT = 600;  // Threshold limit for rising photo diode reading.
const static uint16_t PT_LOWER_THR_LIMIT = 400;  // Threshold limit for falling photo diode reading

portMUX_TYPE setSpeedMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE checkForCorrectionMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE checkForStopConditionMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool diverterDirection;
bool previousDirection;
portMUX_TYPE diverterDirectionMux = portMUX_INITIALIZER_UNLOCKED;
int previousSetSpeed;
portMUX_TYPE currentStationMux = portMUX_INITIALIZER_UNLOCKED;
bool beep_flag = false;

String dropoffStation = "X";
bool getDestinationAPIFlag = false;
bool clearInfeedAPIFlag = false;
bool startInfeedAPIColumnCounter = false;
int infeedAPIcolumnCounter = 0;
bool previousCIStateForInfeedAPI = false;

bool closingFlap = false;
unsigned long startClosingTime;

bool updateParcelDroppedInTheDumpAPIFlag = false;
bool updateParcelDroppedAPIFlag = false;

bool startColumnCounter = false;
int columnCounter = 0;
bool previousCIState = 0;

TaskHandle_t apiTask;
const String DMS_URL_PREFIX = "http://192.168.2.109:8443/m-sort/m-sort-distribution-server/";
HTTPClient _http;

void CloseFlap() {
  digitalWrite(EMG_PIN, HIGH);    // Turn ON the electromagnets
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, HIGH);
}

void OpenFlap() {
  digitalWrite(EMG_PIN, LOW);
}

void StageFlap() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, LOW);
}

void ReadStationViaPhotoDiode() {
  for (int i = 0; i < 3; i++) {
    _photoTransistorPreviousStateArray[i] = _photoTransistorCurrentStateArray[i];
    switch (i) {
      case 0:
        _photoTransistorCurrentValueArray[i] = map(analogRead(SR_D1), 0, 4096, 0, 1024);
        break;
      case 1:
        _photoTransistorCurrentValueArray[i] = map(analogRead(SR_D2), 0, 4096, 0, 1024);
        break;
      case 2:
        _photoTransistorCurrentValueArray[i] = map(analogRead(SR_D3), 0, 4096, 0, 1024);
        break;
    }
    if (_photoTransistorCurrentValueArray[i] > PT_UPPER_THR_LIMIT) {
      _photoTransistorCurrentStateArray[i] = 1;
    } else if (_photoTransistorCurrentValueArray[i] < PT_LOWER_THR_LIMIT) {
      _photoTransistorCurrentStateArray[i] = 0;
    }
  }
}

void CheckColumnIntensityWarningLevel(String segment) {
  // Loop through each bit of the segment
  for (int i = 0; i < 3; i++) {
    int bit = segment[i] - '0';
    int intensityMax = _instantaneousIntensityMaxArray[i];
    int intensityMin = _instantaneousIntensityMinArray[i];
    String status = "Healthy";

    if (bit == 1) {
      if (intensityMax >= 800)
        status = "Healthy";
      else if (intensityMax >= 700 && intensityMax < 800)
        status = "Warning";
      else
        status = "Critical";
    } else { // bit == 0
      if (intensityMin <= 200)
        status = "Healthy";
      else if (intensityMin > 200 && intensityMin <= 300)
        status = "Warning";
      else
        status = "Critical";
    }

    if (status != "Healthy") {
      // add_log("Photodiode " + String(i + 1) + ": " + status);
    }
  }
}

void ReinitialiseVariables() {
  for (int i = 0 ; i < 3 ; i++) {
    _photoTransistorPreviousStateArray[i] = 0;
    _finalColumnCodeArray[i] = "000";
    _photoTransistorCurrentValueArray[i] = 0;
    _photoTransistorCurrentStateArray[i] = 0;
  }
  _intermediateColumnCode = "000";
  _previousIntermediateColumnCode = "000";
  columnNo = 0;
}

String GetStationCodeFromDataReadByStationSensor() {
  String instColumnCode = String(_photoTransistorCurrentStateArray[0]) + String(_photoTransistorCurrentStateArray[1]) + String(_photoTransistorCurrentStateArray[2]);
  String prevInstColumnCode = String(_photoTransistorPreviousStateArray[0]) + String(_photoTransistorPreviousStateArray[1]) + String(_photoTransistorPreviousStateArray[2]);
  if (instColumnCode == prevInstColumnCode) {
    _instantaneousFramesCount++;
    for (int i = 0 ; i < 3 ; i++) {
      _instantaneousIntensityMaxArray[i] = max(_instantaneousIntensityMaxArray[i], _photoTransistorCurrentValueArray[i]);
      _instantaneousIntensityMinArray[i] = min(_instantaneousIntensityMinArray[i], _photoTransistorCurrentValueArray[i]);
    }
  } else {
    // add_log("Inst. Col: " + prevInstColumnCode);
    // add_log("Inst. FC: " + String(_instantaneousFramesCount));
    // add_log("Inst. Max. Intensity: { " + String(_instantaneousIntensityMaxArray[0]) + ", " + String(_instantaneousIntensityMaxArray[1]) + ", " + String(_instantaneousIntensityMaxArray[2]) + " }");
    // add_log("Inst. Min. Intensity: { " + String(_instantaneousIntensityMinArray[0]) + ", " + String(_instantaneousIntensityMinArray[1]) + ", " + String(_instantaneousIntensityMinArray[2]) + " }");
    CheckColumnIntensityWarningLevel(prevInstColumnCode);
    _instantaneousFramesCount = 1;
    for (int i = 0 ; i < 3 ; i++) {
      _instantaneousIntensityMaxArray[i] = _photoTransistorCurrentValueArray[i];
      _instantaneousIntensityMinArray[i] = _photoTransistorCurrentValueArray[i];
    }
  }

  if (instColumnCode == "111") {
    _intermediateColumnCode = instColumnCode;
  }

  // START STATION READING
  if (!_readingInProcess && instColumnCode == "111") {
    _readingInProcess = true;
    stationReadStartTime = millis();
    // // Serial.println("Detection Started");
  }

  // STATION READING COMPLETED
  String stationCode;
  if (instColumnCode == "000" && _readingInProcess) {
    if (_finalColumnCodeArray[2] != "000" && _finalColumnCodeArray[1] != "000" && _finalColumnCodeArray[0] != "000") {
      stationCode = _finalColumnCodeArray[2] + _finalColumnCodeArray[1] + _finalColumnCodeArray[0];
      _rightStationDetected = true;
      _readingInProcess = false;
      // // Serial.println("Detection Completed");
    }
    // add_log("Read time: " + String(millis() - stationReadStartTime));
    // if (columnNo < 3) add_log("Bot has skipped " + String(3 - columnNo) + " columns");
    ReinitialiseVariables();
  }

  // OPEN ZONE TRAVEL
  if (instColumnCode == "000") {
    _readingInProcess = false;
  }

  if (_readingInProcess) {
    int currentStateSum = _photoTransistorCurrentStateArray[0] + _photoTransistorCurrentStateArray[1] + _photoTransistorCurrentStateArray[2];
    int prevStateSum = _photoTransistorPreviousStateArray[0] + _photoTransistorPreviousStateArray[1] + _photoTransistorPreviousStateArray[2];
    if (currentStateSum < prevStateSum) {
      _intermediateColumnCode = instColumnCode;
      _intermediateColumnFramesCount = 1;
    } else if (_intermediateColumnCode == instColumnCode) {
      // Nothing
      if (currentStateSum > 0 && currentStateSum < 3) {
        _intermediateColumnFramesCount++;
      }
    }

    if (_previousIntermediateColumnCode != "000" && _previousIntermediateColumnCode != "111" && _intermediateColumnCode == "111" && _intermediateColumnCode != "000" && _finalColumnCodeArray[0] != _previousIntermediateColumnCode) {
      if (_intermediateColumnFramesCount >= 5) {
        _finalColumnCodeArray[2] = _finalColumnCodeArray[1];
        _finalColumnCodeArray[1] = _finalColumnCodeArray[0];
        _finalColumnCodeArray[0] = _previousIntermediateColumnCode;
        columnNo++;
        // add_log("Col No.: " + String(columnNo));
        // if (columnNo > 3) add_log("Bot is reading more than 3 columns");
      } else {
        // add_log("Potential fluke column. No. of frames: " + String(_intermediateColumnFramesCount));
      }
      // add_log("Inter. Col: " + _previousIntermediateColumnCode);
      // add_log("Inter. FC: " + String(_intermediateColumnFramesCount));
    }
    _previousIntermediateColumnCode = _intermediateColumnCode;
  }

  String stationCodeMain = "";
  if (_rightStationDetected) {
    _rightStationDetected = false;
    stationCodeMain = stationCode;
  }
  return stationCodeMain;
}

String TransformStationCodeIntoStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_CODE_TO_STATION_ID_MAP.find(stationCodeString) != STATION_CODE_TO_STATION_ID_MAP.end()) {
    string stationID = STATION_CODE_TO_STATION_ID_MAP.at(stationCodeString);
    return String(stationID.c_str());
  }
  return "";
}

bool CheckStationValidity(const String &stationCode) {
  // Check if the station-code is present in the keys of STATION_CODE_TO_STATION_ID hash-map
  string stationCodeString = string(stationCode.c_str());
  return STATION_CODE_TO_STATION_ID_MAP.find(stationCodeString) != STATION_CODE_TO_STATION_ID_MAP.end();
}

String GetExpectedStationFromStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_ID_TO_EXPECTED_STATION_ID_MAP.find(stationCodeString) != STATION_ID_TO_EXPECTED_STATION_ID_MAP.end()) {
    string stationID = STATION_ID_TO_EXPECTED_STATION_ID_MAP.at(stationCodeString);
    return String(stationID.c_str());
  }
  return "";
}

bool GetDirectionFromStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_ID_TO_DIRECTION_MAP.find(stationCodeString) != STATION_ID_TO_DIRECTION_MAP.end()) {
    bool direction = STATION_ID_TO_DIRECTION_MAP.at(stationCodeString);
    return direction;
  }
  return 1;
}

int GetSetSpeedAsPerStation(const String &stationId) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationId.c_str());
  if (STATION_ID_TO_SPEED_MAP.find(stationCodeString) != STATION_ID_TO_SPEED_MAP.end()) {
    int speed = STATION_ID_TO_SPEED_MAP.at(stationCodeString);
    return speed;
  }
  return S0_5;
}

// ==============================================================================
// GLOBAL VARIABLES - DIVERTER STATE MACHINE
// ==============================================================================

// Position thresholds (encoder counts)
int left_threshold = 1000;
int right_threshold = 2000;
int left_position_tolerance = 100;   // +/- tolerance for left position
int right_position_tolerance = 100;  // +/- tolerance for right position

// Shared variables
bool global_error_status = false;
DiverterDirection diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;
DiverterDirection previous_diverter_track_edge_direction = DIVERTER_DIRECTION_UNKNOWN;
volatile DiverterPosition current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;

// State machine variables
DiverterState diverter_current_state = DIVERTER_STOP;
DiverterState diverter_previous_state = DIVERTER_STOP;
volatile bool error_diverter_status = false;
bool diverter_set_torque_flag = true;
bool prev_diverter_set_torque_flag = true;
int front_diverter_set_position_value = -1;
int rear_diverter_set_position_value = -1;
int prev_front_diverter_set_position_value = -1;
int prev_rear_diverter_set_position_value = -1;


// Actual position from encoder
int front_diverter_current_position = -1;
volatile int front_diverter_previous_set_position = -1;
int rear_diverter_current_position = -1;
volatile int rear_diverter_previous_set_position = -1;

// Auxiliary task variables
unsigned long last_toggle_time = 0;
const unsigned long TOGGLE_INTERVAL = 10000;  // 10 s

portMUX_TYPE current_diverter_position_high_level_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE error_diverter_status_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE diverter_set_position_value_mux = portMUX_INITIALIZER_UNLOCKED;

int previous_permitted_edge_speed = -1;

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// Convert enum to string functions
String DiverterStateToString(DiverterState state) {
  switch(state) {
    case DIVERTER_LEFT: return "LEFT";
    case DIVERTER_RIGHT: return "RIGHT";
    case DIVERTER_SWITCHING: return "SWITCHING";
    case DIVERTER_ERROR: return "ERROR";
    case DIVERTER_STOP: return "STOP";
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

// Function to read TI and CI sensor combo
String ReadSensorCombo() {
  int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
  int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);
  return String(trafficDetected) + String(columnDetected);
}

// Function to handle column detection and processing
void ProcessColumnDetection(String combo) {
  // Column detection state machine
  switch (wtm_column_state) {
    case WTM_BETWEEN_COLUMNS:
      // We're in the 00 zone between columns
      if (combo == "00") {
        emptySpaceFrameCount++;
      } else {
        // Detected non-00 in empty space - transition to column
        wtm_column_state = WTM_IN_COLUMN;
        totalFramesBetweenSpaces = 0;
        emptySpaceNoiseFrameCount = 0;
        frameCount00 = 0;
        frameCount01 = 0;
        frameCount10 = 0;
        frameCount11 = 0;
        exitConfirmationCount = 0;
        
        // Count the first frame that triggered the transition
        if (combo == "01") {
          frameCount01 = 1;
          totalFramesBetweenSpaces = 1;
        } else if (combo == "10") {
          frameCount10 = 1;
          totalFramesBetweenSpaces = 1;
        } else if (combo == "11") {
          frameCount11 = 1;
          totalFramesBetweenSpaces = 1;
        }
      }
      break;

    case WTM_IN_COLUMN:
      // Count all frames by type
      if (combo == "00") {
        frameCount00++;
      } else if (combo == "01") {
        frameCount01++;
        totalFramesBetweenSpaces++;
        if (frameCount01 > max(frameCount10, frameCount11)) {
          detectedColumnCode = "01";
          previousColumnCode = currentColumnCode;
          currentColumnCode = "01";
        }
      } else if (combo == "10") {
        frameCount10++;
        totalFramesBetweenSpaces++;
        if (frameCount10 > max(frameCount01, frameCount11)) {
          detectedColumnCode = "10";
          previousColumnCode = currentColumnCode;
          currentColumnCode = "10";
        }
      } else if (combo == "11") {
        frameCount11++;
        totalFramesBetweenSpaces++;
        if (frameCount11 > max(frameCount01, frameCount10)) {
          detectedColumnCode = "11";
          previousColumnCode = currentColumnCode;
          currentColumnCode = "11";
        }
      }
      
      // Check if we're exiting the column
      if (combo == "00") {
        exitConfirmationCount++;
        if (exitConfirmationCount >= EXIT_CONFIRMATION_THRESHOLD) {
          // Validate minimum frames threshold
          if (totalFramesBetweenSpaces < MIN_COLUMN_FRAMES_THRESHOLD) {
            // add_log("Fluke: " + String(totalFramesBetweenSpaces) + "f (01:" + String(frameCount01) + ",10:" + String(frameCount10) + ",11:" + String(frameCount11) + ") Rec:" + String(emptySpaceFrameCount + exitConfirmationCount));
            
            emptySpaceFrameCount += EXIT_CONFIRMATION_THRESHOLD;
            emptySpaceTotalFrameCount = emptySpaceFrameCount + totalFramesBetweenSpaces;
            emptySpaceNoiseFrameCount += totalFramesBetweenSpaces;
            wtm_column_state = WTM_BETWEEN_COLUMNS;
            return;
          } else {
            emptySpaceTotalFrameCount = emptySpaceFrameCount;
          }
          
          // Valid column detected - calculate empty space health
          if (emptySpaceFrameCount > 0) {
            totalEmptySpacesDetected++;
            emptySpacesDetectedInSegment++;
            
            // Calculate empty space health using majority and minority approach
            emptySpaceMajorityHealth = CalculateMajorityHealth(emptySpaceFrameCount, DEFAULT_EMPTY_SPACE_THRESHOLD);
            emptySpaceMinorityHealth = CalculateMinorityHealth(emptySpaceNoiseFrameCount, emptySpaceTotalFrameCount);
            emptySpaceHealth = GetWorstHealth(emptySpaceMajorityHealth, emptySpaceMinorityHealth);
            
            // add_log("TES:" + String(totalEmptySpacesDetected) + " ES" + String(emptySpacesDetectedInSegment) + ":" + String(emptySpaceFrameCount) + "f " + HealthToString(emptySpaceHealth));
            emptySpaceNoiseFrameCount = 0;
          }
          
          totalColumnsDetected++;
          uniqueColumnsDetectedInSegment++;
          
          // Update previous and current column codes
          previousColumnCode = detectedColumnCode;
          currentColumnCode = "00";
          
          columnJustCompleted = true;
          lastCompletedColumnCode = detectedColumnCode;
          lastCompletedColumnHealth = AnalyzeWTMColumnHealth();
          
          wtm_column_state = WTM_BETWEEN_COLUMNS;
          emptySpaceFrameCount = EXIT_CONFIRMATION_THRESHOLD;
        }
      } else {
        exitConfirmationCount = 0;
      }
      break;
  }
}

// Function to handle column-based decisions
void HandleColumnDecisions() {
  if (columnJustCompleted) {
    if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
      column_detected_in_sweep = true;
    }
    if (lastCompletedColumnCode == "11") {
      // add_log("Traffic Detected");
      traffic_permission = false;
    } else if (lastCompletedColumnCode == "01") {
      // add_log("Traffic Not Detected");
      traffic_permission = true;
    } else if (lastCompletedColumnCode == "10") {
      // add_log("WARNING: Column Indicator malfunction detected! Reading '10' (CI=0, TI=1)");
    }
    
    if (lastCompletedColumnHealth == CRITICAL) {
      // Column health is critical - may need maintenance
    }
    
    columnJustCompleted = false;
  }
}

unsigned long last_set_sweep_time = 0;

// ==============================================================================
// DRIVE MOTOR STATE LOGGING - SMART PARAMETER TRACKING
// ==============================================================================

// Previous values for change detection
struct DriveMotorStateSnapshot {
    bool current_assignment;
    bool traffic_permission;
    int drive_motor_current_speed;
    bool drive_motor_error_status;
    bool global_error_status;
    bool column_detected_in_sweep;
    int permitted_edge_speed;
    DriveMotorState current_state;
    unsigned long timestamp;
    bool error_mode;
    bool check_diverter_permission;
};

DriveMotorStateSnapshot prev_dm_snapshot = {true, true, 0, false, false, false, 0, DRIVE_MOTOR_STOP, 0, false, true};
unsigned long last_periodic_log_time = 0;
const unsigned long PERIODIC_LOG_INTERVAL = 5000; // Log every 5 seconds

const char* GetDriveMotorStateName(DriveMotorState state) {
    switch(state) {
        case DRIVE_MOTOR_STOP: return "STOP";
        case DRIVE_MOTOR_STOPPING: return "STOPPING";
        case DRIVE_MOTOR_RUNNING: return "RUNNING";
        case DRIVE_MOTOR_SWEEPING_COLUMN: return "SWEEP";
        case DRIVE_MOTOR_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void LogDriveMotorStateParams(bool forceLog = false) {
    // Check if any critical parameter changed
    bool paramChanged = (
        // prev_dm_snapshot.current_assignment != current_assignment ||
        // prev_dm_snapshot.traffic_permission != traffic_permission ||
        // prev_dm_snapshot.drive_motor_current_speed != drive_motor_current_speed ||
        // prev_dm_snapshot.drive_motor_error_status != drive_motor_error_status ||
        // prev_dm_snapshot.global_error_status != global_error_status ||
        // prev_dm_snapshot.column_detected_in_sweep != column_detected_in_sweep ||
        // prev_dm_snapshot.permitted_edge_speed != permitted_edge_speed ||
        // prev_dm_snapshot.current_state != drive_motor_current_state ||
        // prev_dm_snapshot.error_mode != errorMode
        prev_dm_snapshot.check_diverter_permission != check_diverter_permission
    );
    
    // Periodic logging (every 5 seconds) or on parameter change
    unsigned long currentTime = millis();
    if (forceLog || paramChanged || (currentTime - last_periodic_log_time >= PERIODIC_LOG_INTERVAL)) {
        String logMsg = "DM[" + String(GetDriveMotorStateName(drive_motor_current_state));
        if (drive_motor_current_state != drive_motor_previous_state) {
            logMsg += "←" + String(GetDriveMotorStateName(drive_motor_previous_state));
        }
        logMsg += "] Assn:" + String(current_assignment) + 
                       " Traf:" + String(traffic_permission) + 
                       " Spd:" + String(drive_motor_current_speed) + 
                       " Err:" + String(drive_motor_error_status) + 
                       " GErr:" + String(global_error_status) + 
                       " ColSwp:" + String(column_detected_in_sweep) + 
                       " EdgeSpd:" + String(permitted_edge_speed) + 
                       " EMode:" + String(errorMode) +
                       " ChkDivPerm:" + String(check_diverter_permission);
        
        // Only add change indicator if it's due to parameter change, not periodic
        if (paramChanged && !forceLog) {
            logMsg += " [CHG]";
        } else if (!paramChanged && !forceLog) {
            logMsg += " [PER]";
        }
        
        add_log(logMsg);
        
        // Update snapshot
        prev_dm_snapshot.current_assignment = current_assignment;
        prev_dm_snapshot.traffic_permission = traffic_permission;
        prev_dm_snapshot.drive_motor_current_speed = drive_motor_current_speed;
        prev_dm_snapshot.drive_motor_error_status = drive_motor_error_status;
        prev_dm_snapshot.global_error_status = global_error_status;
        prev_dm_snapshot.column_detected_in_sweep = column_detected_in_sweep;
        prev_dm_snapshot.permitted_edge_speed = permitted_edge_speed;
        prev_dm_snapshot.current_state = drive_motor_current_state;
        prev_dm_snapshot.timestamp = currentTime;
        prev_dm_snapshot.error_mode = errorMode;
        prev_dm_snapshot.check_diverter_permission = check_diverter_permission;
        
        last_periodic_log_time = currentTime;
    }
}

DriveMotorStateOutput UpdateDriveMotorState() {
    /*
    Evaluates global variables and determines if state should change
    Updates Current_State based on conditions and valid transitions
    Returns: true if state was changed, false if no change
    */
    drive_motor_previous_state = drive_motor_current_state;
    
    // Log parameters before evaluation (smart logging - only when changed or periodic)
    LogDriveMotorStateParams();    
    // ----- FROM STOP STATE -----
    if (drive_motor_current_state == DRIVE_MOTOR_STOP) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to RUNNING
        if (current_assignment == true && traffic_permission == true && global_error_status == false) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to SWEEPING_COLUMN
        if (current_assignment == true && drive_motor_current_speed == 0 && traffic_permission == false && global_error_status == false && millis() - last_set_sweep_time >= 1000) {
            drive_motor_current_state = DRIVE_MOTOR_SWEEPING_COLUMN;
            last_set_sweep_time = millis();
            column_detected_in_sweep = false;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // No valid transition condition met and remain in STOP
        drive_motor_current_state = DRIVE_MOTOR_STOP;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM STOPPING STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOP (when vehicle has stopped)
        if (drive_motor_current_speed == 0) {
            drive_motor_current_state = DRIVE_MOTOR_STOP;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to RUNNING (if assignment and permission restored)
        if (current_assignment == true && traffic_permission == true && global_error_status == false) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Still stopping - remain in STOPPING
        drive_motor_current_state = DRIVE_MOTOR_STOPPING;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM RUNNING STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOPPING (traffic permission removed)
        if (global_error_status == true || (current_assignment == true && traffic_permission == false)) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        if (previous_permitted_edge_speed != permitted_edge_speed) {
          drive_motor_current_state = DRIVE_MOTOR_RUNNING;
          previous_permitted_edge_speed = permitted_edge_speed;
          return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Continue moving - remain in RUNNING
        drive_motor_current_state = DRIVE_MOTOR_RUNNING;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM SWEEPING_COLUMN STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOPPING (column crossed)
        if (global_error_status == true || (current_assignment == true && traffic_permission == false && column_detected_in_sweep == true)) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to RUNNING (if traffic permission granted)
        if (current_assignment == true && traffic_permission == true && global_error_status == false) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Continue sweeping - remain in SWEEPING_COLUMN
        drive_motor_current_state = DRIVE_MOTOR_SWEEPING_COLUMN;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM ERROR_MOTOR STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_ERROR) {
        // Transition to STOP (error cleared and vehicle stopped)
        if (drive_motor_error_status == false && drive_motor_current_speed == 0) {
            drive_motor_current_state = DRIVE_MOTOR_STOP;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOPPING (error cleared but vehicle still moving)
        if (drive_motor_error_status == false && drive_motor_current_speed > 0) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Still in error state - remain in ERROR_MOTOR
        drive_motor_current_state = DRIVE_MOTOR_ERROR;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    return DRIVE_MOTOR_UNKNOWN_STATE;  // No state change
}

// ==============================================================================
// STATE IMPLEMENTER / ACTUATOR
// ==============================================================================

// Execute state actions when update_state() returns true
void ImplementDriveMotorState() {
    /*
    Executes the actions required for the current state
    */
    
    if (drive_motor_current_state == DRIVE_MOTOR_STOP) {
        // ===== STOP STATE =====
        drive_motor_set_speed = 0;
        // add_log("Stopping the bot.");
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
        // ===== STOPPING STATE =====
        drive_motor_set_speed = 0;
        // add_log("Stopping the bot.");
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
        // ===== RUNNING STATE =====
        drive_motor_set_speed = permitted_edge_speed;
        drive_motor_current_direction = DRIVE_MOTOR_FORWARD; // Reset current direction
        // add_log("Running the bot at speed: " + drive_motor_set_speed);
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
        // ===== SWEEPING COLUMN STATE =====
        
        // Toggle Direction
        if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
            drive_motor_current_direction = DRIVE_MOTOR_REVERSE;
        }
        else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
            drive_motor_current_direction = DRIVE_MOTOR_FORWARD;
        }
        
        // Set Direction Flag
        if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
            drive_motor_direction_value = 1;
        }
        else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
            drive_motor_direction_value = -1;
        }
        
        // Set speed with direction
        drive_motor_set_speed = drive_motor_sweep_speed * drive_motor_direction_value;
        // add_log("Sweeping column at speed: " + String(drive_motor_sweep_speed) + " in direction: " + String(drive_motor_direction_value));
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_ERROR) {
        // ===== ERROR_MOTOR STATE =====
        drive_motor_set_speed = 0;
        // add_log("Drive motor error detected! Halting the bot.");
    }
}

// Update high-level position based on current encoder position
void UpdateDiverterPositionHighLevel() {
  if (front_diverter_current_position >= (front_diverter_right_thresold - front_diverter_tolerance) && front_diverter_current_position <= front_diverter_right_limit &&
  rear_diverter_current_position <= (rear_diverter_right_thresold + rear_diverter_tolerance) && rear_diverter_current_position >= rear_diverter_right_limit) {

    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_RIGHT;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  else if (front_diverter_current_position >= front_diverter_left_limit && front_diverter_current_position <= (front_diverter_left_thresold + front_diverter_tolerance)  &&
  rear_diverter_current_position <= rear_diverter_left_limit && rear_diverter_current_position >= (rear_diverter_left_thresold - rear_diverter_tolerance)) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_LEFT;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  else if (front_diverter_current_position < (front_diverter_right_thresold - front_diverter_tolerance) && front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance) &&
  rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance) && rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_INTERIM;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  else {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }
}

// ==============================================================================
// STATE UPDATE FUNCTION
// ==============================================================================

DiverterStateChangeOptions UpdateDiverterState() {
  /*
  Evaluates global variables and determines if state should change
  Updates Current_State based on conditions and valid transitions
  Returns: state change status
  */
  diverter_previous_state = diverter_current_state;

  if (currentStation == "D1" || currentStation == "I1" || currentStation == "I11") {
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      if (current_diverter_position_high_level == DIVERTER_POSITION_RIGHT) {
        check_diverter_permission = true;
      } else {
        check_diverter_permission = false;
      }
    } else if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      if (current_diverter_position_high_level == DIVERTER_POSITION_LEFT) {
        check_diverter_permission = true;
      } else {
        check_diverter_permission = false;
      }
    }
  } else {
    check_diverter_permission = true;
  }
  
  // ----- FROM LEFT STATE -----
  if (diverter_current_state == DIVERTER_LEFT) {
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to STOP_DIVERTER
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to SWITCHING
    if (error_diverter_status == false && 
      global_error_status == false && 
      (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT ||
      current_diverter_position_high_level != DIVERTER_POSITION_LEFT)) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in LEFT
    diverter_current_state = DIVERTER_LEFT;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM RIGHT STATE -----
  else if (diverter_current_state == DIVERTER_RIGHT) {
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
      
    // Transition to STOP_DIVERTER
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to SWITCHING
    if (error_diverter_status == false && 
      global_error_status == false && 
      (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT ||
      current_diverter_position_high_level != DIVERTER_POSITION_RIGHT)) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in RIGHT
    diverter_current_state = DIVERTER_RIGHT;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM SWITCHING STATE -----
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    // add_log("Track edge direction: " + DiverterDirectionToString(diverter_track_edge_direction) + ", pos high level: " + DiverterPositionToString(current_diverter_position_high_level));
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to STOP_DIVERTER
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to LEFT
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT && 
      current_diverter_position_high_level == DIVERTER_POSITION_LEFT) {
      diverter_current_state = DIVERTER_LEFT;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to RIGHT
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT && 
      current_diverter_position_high_level == DIVERTER_POSITION_RIGHT) {
      diverter_current_state = DIVERTER_RIGHT;
      return DIVERTER_STATE_CHANGED;
    }

    if (previous_diverter_track_edge_direction != diverter_track_edge_direction) {
      diverter_current_state = DIVERTER_SWITCHING;
      previous_diverter_track_edge_direction = diverter_track_edge_direction;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Continue switching - remain in SWITCHING
    diverter_current_state = DIVERTER_SWITCHING;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM STOP_DIVERTER STATE -----
  else if (diverter_current_state == DIVERTER_STOP) {
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to SWITCHING
    if (error_diverter_status == false && 
      global_error_status == false) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in STOP_DIVERTER
    diverter_current_state = DIVERTER_STOP;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM ERROR_DIVERTER STATE -----
  else if (diverter_current_state == DIVERTER_ERROR) {
    // Transition to STOP_DIVERTER
    if (error_diverter_status == false) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in ERROR_DIVERTER
    diverter_current_state = DIVERTER_ERROR;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  return DIVERTER_UNEXPECTED_CONDITION;
}

// ==============================================================================
// STATE IMPLEMENTER / ACTUATOR
// ==============================================================================

void UpdateDiverterActuatorInputVariables() {
  /*
  Executes the actions required for the current state
  */
  
  if (diverter_current_state == DIVERTER_LEFT) {
    // ===== LEFT STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter in " + DiverterStateToString(DIVERTER_LEFT) + " position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_RIGHT) {
    // ===== RIGHT STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter in " + DiverterStateToString(DIVERTER_RIGHT) + " position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    // ===== SWITCHING STATE =====
    diverter_set_torque_flag = true;  // Enable torque for movement
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_left_limit;
      rear_diverter_set_position_value = rear_diverter_left_limit;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      add_log("Switching to " + DiverterDirectionToString(DIVERTER_DIRECTION_LEFT) + " - moving to front limit: " + String(front_diverter_left_limit) + " and rear limit: " + String(rear_diverter_left_limit));
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_right_limit;
      rear_diverter_set_position_value = rear_diverter_right_limit;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      add_log("Switching to " + DiverterDirectionToString(DIVERTER_DIRECTION_RIGHT) + " - moving to front limit: " + String(front_diverter_right_limit) + " and rear limit: " + String(rear_diverter_right_limit));
    }
  }
  
  else if (diverter_current_state == DIVERTER_ERROR) {
    // ===== ERROR_DIVERTER STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter error detected! Torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_STOP) {
    // ===== STOP_DIVERTER STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter stopped - torque disabled.");
  }
}

// ==============================================================================
// RTOS TASKS
// ==============================================================================

// Diverter State Machine Task
void READING_SENSOR_AND_UPDATE_STATE_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);  // 10ms cycle time
    global_error_status = error_diverter_status || drive_motor_error_status || errorMode;

    ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

    String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

    if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
      portENTER_CRITICAL(&currentStationMux);
      currentStation = TransformStationCodeIntoStationId(stationCodeMain);
      portEXIT_CRITICAL(&currentStationMux);
      add_log("Current station: " + currentStation);
      if (CheckStationValidity(stationCodeMain)) {
        portENTER_CRITICAL(&diverterDirectionMux);
        // changeDirection = true;
        diverterDirection = GetDirectionFromStationId(currentStation);
        if (diverterDirection) diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;
        else diverter_track_edge_direction = DIVERTER_DIRECTION_LEFT;
        portEXIT_CRITICAL(&diverterDirectionMux);
        add_log("Diverter direction changed to " + String(diverterDirection));
        if (!localisationFlag) {
          if (expectedStation != currentStation) {
            errorCode = 3;
            errorMode = true;
            firstTimeError = true;
          }
        }
        else {
          localisationFlag = false;
        }
        if (!errorMode) {
          int getSpeed = GetSetSpeedAsPerStation(currentStation);
          permitted_edge_speed = getSpeed;
          add_log("Speed should be changed to " + String(getSpeed));
          expectedStation = GetExpectedStationFromStationId(currentStation);
        }
      } else {
        add_log("Invalid station code: " + stationCodeMain);
        errorCode = 4;
        errorMode = true;
        firstTimeError = true;
      }
    }

    // Read TI/CI sensors and process column detection
    int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
    int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);
    String combo = String(trafficDetected) + String(columnDetected);
    ProcessColumnDetection(combo);
    HandleColumnDecisions();

    // ---------------------------------------------------------------- Infeed Controller --------------------------------------------------------
    if (previousStation != currentStation) {
      if (currentStation == "I12") {
        startColumnCounter = true;
      }
    }
    
    if (columnDetected) {
      if(!previousCIState) {
        if (startColumnCounter) {
          columnCounter++;
          if(columnCounter >= 3) {
            add_log("Actual speed changed after 3 columns");
            permitted_edge_speed = S1;
            // add_log("SET SPEED UPDATE TO: " + String(setSpeed) + " BCZ IT WILL START FROM INFEED STOP POINT");
            beep_flag = true;
            startColumnCounter = false;
            columnCounter = 0;
            startInfeedAPIColumnCounter = true;
          }
        }
      }
    }
    previousCIState = columnDetected;
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------- Destination Controller --------------------------------------------------------
    if (previousStation != currentStation) {
      if(currentStation == dropoffStation) {
        if (dropoffStation == "D18") {
          updateParcelDroppedInTheDumpAPIFlag = true;
        } else {
          updateParcelDroppedAPIFlag = true;
        }
      }
    }
    
    if (columnDetected) {
      if(!previousCIStateForInfeedAPI) {
        if (startInfeedAPIColumnCounter) {
          if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
            infeedAPIcolumnCounter++;
          } else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
            infeedAPIcolumnCounter -= 2;
          }
          if(infeedAPIcolumnCounter >= 10) {
            beep_flag = true;
            getDestinationAPIFlag = true;
            infeedAPIcolumnCounter = 0;
            startInfeedAPIColumnCounter = false;
          }
        }
      }
    }
    previousCIStateForInfeedAPI = columnDetected;
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // --------------------------------------------------------------- AFC Controller ------------------------------------------------------------
    if (previousStation != currentStation) {
      if(currentStation == dropoffStation && dropoffStation != "X") {
        add_log("Opening the flap");
        OpenFlap();
        closingFlap = true;
        startClosingTime = millis();
      } else if (currentStation == "I11") {
        add_log("Staging the flap");
        StageFlap();
      }
    }

    if (closingFlap && millis() - startClosingTime >= 1000) {
      add_log("Closing the flap");
      CloseFlap();
      closingFlap = false;
    }
    // -------------------------------------------------------------------------------------------------------------------------------------------

    previousStation = currentStation;

    if (UpdateDriveMotorState() == DRIVE_MOTOR_STATE_CHANGED) {
      add_log("State changed to: " + String(drive_motor_current_state) + " from " + String(drive_motor_previous_state));
      // add_log("All shared variables - Traffic_Permission: " + String(traffic_permission) + ", Global_Error_Status: " + String(global_error_status) + ", Current_Assignment: " + String(current_assignment) + ", Permitted_Edge_Speed: " + String(permitted_edge_speed) + ", column_detected_in_sweep: ," + 
                // String(column_detected_in_sweep));
      ImplementDriveMotorState();
    }
    
    DiverterStateChangeOptions stateChange = UpdateDiverterState();
    if (stateChange != DIVERTER_STATE_NO_CHANGE) Serial.println(stateChange);
    if (stateChange == DIVERTER_STATE_CHANGED) {
      add_log("State changed to: " + DiverterStateToString(diverter_current_state) + " from " + DiverterStateToString(diverter_previous_state));
      // add_log("Variables - Global_Error: " + String(global_error_status) + ", Demand_Dir: " + DiverterDirectionToString(diverter_track_edge_direction) + ", Pos_HL: " + DiverterPositionToString(current_diverter_position_high_level) + ", Error_Status: " + String(error_diverter_status));
      // add_log("Front current position: " + String(front_diverter_current_position) + ", Rear current position: " + String(rear_diverter_current_position));
      UpdateDiverterActuatorInputVariables();
    } else if (stateChange == DIVERTER_UNEXPECTED_CONDITION) {
      add_log("Diverter State Machine in UNEXPECTED CONDITION!");
    }
  }
}

unsigned long lastDemandVelocityCheckTime = 0;

// Actuation Task
void ACTUATION_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Update motor speed if changed
    if (!diverter_set_torque_flag ) {
      if (prev_diverter_set_torque_flag)
      {
        delay(1);
        sm.EnableTorque(FRONT_SERVO, DISABLE);
        if (sm.getLastError()) {
          add_log("Front torque disabling communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        delay(1);
        sm.EnableTorque(REAR_SERVO, DISABLE);
        if (sm.getLastError()) {
          add_log("Rear torque disabling communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }
    } else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      if (front_diverter_set_position_value != -1 && front_diverter_current_position < (front_diverter_right_thresold - front_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(FRONT_SERVO, front_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Front position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }

      if (rear_diverter_set_position_value != -1 && rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Rear position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }
    } else if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      if (front_diverter_set_position_value != -1 && front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(FRONT_SERVO, front_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Front position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }

      if (rear_diverter_set_position_value != -1 && rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Rear position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }
    }
    prev_diverter_set_torque_flag = diverter_set_torque_flag;
    delay(1);
    front_diverter_current_position = sm.ReadPos(FRONT_SERVO);
    if (sm.getLastError()) {
      add_log("Front position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }
    delay(1);
    rear_diverter_current_position = sm.ReadPos(REAR_SERVO);
    if (sm.getLastError()) {
      add_log("Rear position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }

    UpdateDiverterPositionHighLevel();

    // Update motor speed if changed
    if (drive_motor_set_speed != drive_motor_previous_set_speed) {
      CANStatus success = setTargetVelocity(drive_motor_set_speed);
      if (success == CAN_OK) {
        int32_t getSetVelocity;
        if (getTargetVelocity(&getSetVelocity) == CAN_OK) {
          if (getSetVelocity == drive_motor_set_speed) {
            drive_motor_previous_set_speed = drive_motor_set_speed;
          }
        }
      }
    }

    // Monitor drive motor current speed
    int32_t current_velocity;
    if (getVelocityActualValueAveraged(&current_velocity) == CAN_OK) {
      drive_motor_current_speed = current_velocity;
    }

    if (beep_flag) {
      Beep();
      beep_flag = false;
    }
  }
}

// ==============================================================================
// SETUP AND INITIALIZATION
// ==============================================================================

void setup() {
  // ===== 1. SERIAL COMMUNICATION (Foundation for debugging) =====
  Serial.begin(115200);
  delay(2000);  // Allow serial to stabilize
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  Library-less State Machine - d540     ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // ===== 2. THREAD-SAFE PRIMITIVES (Before any multi-threaded operations) =====
  xDebugLogMutex = xSemaphoreCreateMutex();
  if (xDebugLogMutex == NULL) {
    Serial.println("✗ FATAL: Failed to create debug log mutex");
    while(1) delay(1000);
  }
  Serial.println("✓ Mutex created for thread-safe logging\n");
  
  // ===== 3. NON-VOLATILE MEMORY (Load config before using BOT_ID) =====
  LoadDiverterConfig();
  Serial.println("✓ Configuration loaded from NVM\n");
  
  // ===== 4. INITIALIZE GLOBAL VARIABLES =====
  debugLoggingString = BOT_ID + " " + CODE_ID + ": ";
  
  // ===== 5. NETWORK CONNECTIVITY (For remote logging) =====
  WiFiConfig();
  
  // ===== 6. GPIO CONFIGURATION (Hardware pins) =====
  PinConfig();
  Serial.println("✓ GPIO pins configured\n");
  
  // ===== 7. HARDWARE POWER-UP DELAY (Critical for EPOS stability) =====
  Serial.println("⏳ Waiting for motor/driver power-up...");
  Serial.print("   ");
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println("Ready!\n");

  // ===== 8. SERVO CONFIGURATION =====
  ServoConfig();
  Serial.println("✓ Servo system configured\n");
  
  // ===== 9. CAN BUS INITIALIZATION (Communication layer) =====
  Serial.println("→ Initializing CAN (500 kbit/s)...");
  add_log("→ CAN init");
  if (!CAN_Init()) {
    add_log("✗ FATAL: CAN init failed");
    Serial.println("✗ FATAL: CAN initialization failed");
    while (1) delay(1000);
  }
  add_log("✓ CAN initialized");
  Serial.println("✓ CAN initialized (TX: GPIO 14, RX: GPIO 15)\n");
  
  // ===== 10. MOTOR CONFIGURATION (Depends on CAN) =====
  // Clear any pending CAN messages before motor config
  SimpleCANFrame dummy_frame;
  while (CAN_Receive(&dummy_frame, 1) == CAN_OK) {
    // Flush receive buffer
  }
  
  if (!MotorConfig()) {
    add_log("✗ FATAL: Motor config failed");
    Serial.println("\n✗ FATAL: Motor configuration failed");
    Serial.println("   System halted. Fix errors and reset.\n");
    while (1) delay(1000);
  }
  
  // Verify motor is actually enabled by reading statusword
  uint32_t statusword = 0;
  if (SDO_Read(0x6041, 0x00, &statusword) == CAN_OK) {
    Serial.print("   Motor Statusword: 0x");
    Serial.println(statusword, HEX);
    if ((statusword & 0x004F) == 0x0027) {  // Operation enabled state
      Serial.println("   ✓ Motor verified in ENABLED state\n");
    } else {
      Serial.println("   ⚠ Motor not in expected state - retrying enable...");
      delay(100);
      if (enable() == CAN_OK) {
        Serial.println("   ✓ Motor enabled on retry\n");
      }
    }
  }

  // ===== 11. SET INITIAL STATE VARIABLES =====
  drive_motor_set_speed = S0_5;  // Initial speed set to 0.5 m/s
  
  // ===== 12. USER FEEDBACK =====
  Beep();
  add_log("Bot:" + String(BOT_ID) + " Code:" + CODE_ID);
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  printFileName();
  add_log("Starting the bot!");
  
  // ===== 13. CREATE RTOS TASKS (Last - after all hardware ready) =====
  Serial.println("→ Creating RTOS tasks...");
  
  // Create HTTP debug logger task on Core 0
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    8192,  // 8KB stack for HTTP operations
    NULL,
    2,
    &httpDebugLog,
    0);
  
  // Create sensor reading task on Core 1
  xTaskCreatePinnedToCore(
    READING_SENSOR_AND_UPDATE_STATE_TASK,
    "reading_sensor_and_update_state_task",
    6144,  // 6KB stack for state machine and column detection
    NULL,
    5,
    &readingSensorAndUpdateStateTask,
    1);
  
  vTaskDelay(pdMS_TO_TICKS(20));
  
  // Create actuation task on Core 1
  xTaskCreatePinnedToCore(
    ACTUATION_TASK,
    "actuation_task",
    4096,  // 4KB stack for motor control and CAN
    NULL,
    4,
    &actuationTask,
    1);
  
  Serial.println("✓ All tasks created\n");
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   System Ready - State Machine Active  ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  vTaskDelay(pdMS_TO_TICKS(20));
  
  xTaskCreatePinnedToCore(
    API_TASK,
    "api_task",
    4096,
    NULL,
    3,
    &apiTask,
    0);
  
  last_toggle_time = millis();
}

void loop() {
  // Empty loop as tasks handle functionality
}

// Calculate health status based on majority frame count
HealthStatus CalculateMajorityHealth(int majorityCount, int threshold) {
  if (threshold == 0) return SAFE;
  
  float percentage = (float)majorityCount / threshold * 100.0;
  
  if (percentage > MAJORITY_SAFE_THRESHOLD) {
    return SAFE;
  } else if (percentage >= MAJORITY_WARNING_THRESHOLD) {
    return WARNING;
  } else {
    return CRITICAL;
  }
}

// Calculate health status based on minority frame counts
HealthStatus CalculateMinorityHealth(int minorityTotal, int totalFrames) {
  if (totalFrames == 0) return SAFE;
  
  float percentage = (float)minorityTotal / totalFrames * 100.0;
  
  if (percentage < MINORITY_SAFE_THRESHOLD) {
    return SAFE;
  } else if (percentage <= MINORITY_WARNING_THRESHOLD) {
    return WARNING;
  } else {
    return CRITICAL;
  }
}

// Calculate empty space health using majority (00 frames) and minority (noise frames) approach
HealthStatus CalculateEmptySpaceHealth(int majorityFrames, int noiseFrames, int totalFrames) {
  if (totalFrames == 0) return SAFE;
  
  // Calculate majority health (00 frames should be > 80% of total)
  HealthStatus majorityHealth = CalculateMajorityHealth(majorityFrames, DEFAULT_EMPTY_SPACE_THRESHOLD);
  
  // Calculate minority health (noise frames should be < 20% of total)
  HealthStatus minorityHealth = CalculateMinorityHealth(noiseFrames, totalFrames);
  
  // Overall health is worst of both (AND operation)
  return GetWorstHealth(majorityHealth, minorityHealth);
}

// Get worst health status between two statuses
HealthStatus GetWorstHealth(HealthStatus h1, HealthStatus h2) {
  if (h1 == CRITICAL || h2 == CRITICAL) return CRITICAL;
  if (h1 == WARNING || h2 == WARNING) return WARNING;
  return SAFE;
}

// Convert health status to string
String HealthToString(HealthStatus health) {
  switch(health) {
    case SAFE: return "SAFE";
    case WARNING: return "WARNING";
    case CRITICAL: return "CRITICAL";
    default: return "UNKNOWN";
  }
}

// Analyze column detection and report health
HealthStatus AnalyzeWTMColumnHealth() {
  if (detectedColumnCode == "") return SAFE;
  
  // Total frames during column reading (excluding 00 exit confirmation frames)
  int totalColumnFrames = frameCount01 + frameCount10 + frameCount11;
  int totalFrames = frameCount00 + totalColumnFrames;  // All frames including noise
  
  // Determine majority and minority counts
  int majorityCount = 0;
  int minorityTotal = 0;
  String majorityCode = "";
  
  // Find the majority (should match detected column code)
  if (detectedColumnCode == "01") {
    majorityCount = frameCount01;
    majorityCode = "01";
    minorityTotal = frameCount10 + frameCount11;  // Other column readings (not 00)
  } else if (detectedColumnCode == "10") {
    majorityCount = frameCount10;
    majorityCode = "10";
    minorityTotal = frameCount01 + frameCount11;
  } else if (detectedColumnCode == "11") {
    majorityCount = frameCount11;
    majorityCode = "11";
    minorityTotal = frameCount01 + frameCount10;
  }
  
  // Calculate health
  // Majority: Use static threshold (DEFAULT_COLUMN_FRAME_THRESHOLD or maxCount)
  majorityHealthWTM = CalculateMajorityHealth(majorityCount, DEFAULT_COLUMN_FRAME_THRESHOLD);
  
  // Minority: Use total frames captured from previous empty space to current empty space
  minorityHealthWTM = CalculateMinorityHealth(minorityTotal, totalFramesBetweenSpaces);
  overallHealthWTM = GetWorstHealth(majorityHealthWTM, minorityHealthWTM);
  
  // Compact log for memory efficiency
  // add_log("C" + String(totalColumnsDetected) + "[" + detectedColumnCode + "]:" + String(totalFramesBetweenSpaces) + "f(" + String(frameCount01) + "," + String(frameCount10) + "," + String(frameCount11) + ") " + HealthToString(overallHealthWTM));
  return overallHealthWTM;
}

void printFileName() {
  Serial.println(__FILE__);
}

// Function to load values from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);
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

void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("MCP Error.");
  }
}

void Beep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (debugLoggingString != BOT_ID + " " + CODE_ID + ": ") {
      // Serial.println("Sending logs!");
      // Serial.println(String(WiFi.status()));
      int httpCode = -1;
      if (loggerFlag) {
        String loggerUrl = HTTP_DEBUG_SERVER_URL + "?entity=" + ENTITY_TYPE + "&entity_id=" + BOT_ID;
        httpDebugger.begin(loggerUrl);
        httpCode = httpDebugger.POST(debugLoggingString);
        // Serial.println("Response: " + httpDebugger.getString());
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = BOT_ID + " " + CODE_ID + ": ";
      }
    }
  }
}

// Function to add logs into the log string (thread-safe)
void add_log(String log) {
//   Serial.println(log);  // Also print to serial for real-time debugging
  if (xDebugLogMutex != NULL) {
    if (xSemaphoreTake(xDebugLogMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      debugLoggingString += " | " + log;
      xSemaphoreGive(xDebugLogMutex);
    }
  } else {
    debugLoggingString += " | " + log;
  }
}

// Function to connect controller to network
void WiFiConfig() {
  add_log("WiFi connecting...");
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  add_log("WiFi connected");
  Serial.println("Connected to WiFi!");
}

void PinConfig() {
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT_PULLUP);       // Traffic indicator pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT_PULLUP);      // Column indicator pin config

  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);

  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // GPIO command outputs to Secondary
  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);

  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, LOW);
}

void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
}

void GetDestinationAndJourneyPath() {
  String url = DMS_URL_PREFIX + "GetDestinationStationIdForBot?botId=" + BOT_ID + "&infeedStationId=I1&strategyType=MyntraSingleScan&retries=0&sectionId=S1";
  int httpCode = 0;
  _http.begin(url);
  add_log("API call!");
  httpCode = _http.GET();
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
  } else {
    add_log(_http.getString());
  }
  _http.end();
  
  if (response == "") {
    dropoffStation = "D18";
  } else {
    int destinationIndex = response.indexOf('-');
    dropoffStation = response.substring(0, destinationIndex);
  }
}

void UpdateParcelDropped() {
  String url = DMS_URL_PREFIX + "UpdateParcelDroppedForBot?botId=" + BOT_ID + "&sectionIds=S1&infeedStationId=I1";
  int httpCode = 0;
  _http.begin(url);
  add_log("Update API call!");
  httpCode = _http.PUT(url);
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
  } else {
    add_log(_http.getString());
  }
  _http.end();
}

void UpdateParcelDroppedInTheDump() {
  String url = DMS_URL_PREFIX + "UpdateParcelDroppedInDumpForBot?botId=" + BOT_ID + "&sectionIds=S1&infeedStationId=I1&reason=FailedDetection";
  int httpCode = 0;
  _http.begin(url);
  add_log("API call!");
  httpCode = _http.PUT(url);
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
  } else {
    add_log(_http.getString());
  }
  _http.end();
}

void ClearInfeedStation() {
  String url = DMS_URL_PREFIX + "ClearInfeedStations?infeedStationId=I1";
  int httpCode = 0;
  _http.begin(url);
  while (httpCode != HTTP_CODE_OK) {
    httpCode = _http.PUT(url);
    delay(100);
  }
  _http.end();
}

// Logger Function which will run as RTOS task
void API_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (getDestinationAPIFlag) {
      GetDestinationAndJourneyPath();
      clearInfeedAPIFlag = true;
      getDestinationAPIFlag = false;
    }

    if (clearInfeedAPIFlag) {
      ClearInfeedStation();
      clearInfeedAPIFlag = false;
    }

    if (updateParcelDroppedInTheDumpAPIFlag) {
      UpdateParcelDroppedInTheDump();
      updateParcelDroppedInTheDumpAPIFlag = false;
      dropoffStation = "X";
    }

    if (updateParcelDroppedAPIFlag) {
      UpdateParcelDropped();
      updateParcelDroppedAPIFlag = false;
      dropoffStation = "X";
    }
  }
}