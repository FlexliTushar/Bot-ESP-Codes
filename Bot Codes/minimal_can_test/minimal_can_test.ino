/*
  Minimal CAN Communication Test with Maxon EPOS
  
  Hardware:
  - ESP32 CAN TX: GPIO 14
  - ESP32 CAN RX: GPIO 15
  - CAN transceiver connected to these pins
  - Maxon EPOS motor controller on CAN bus (Node ID = 1)
  
  What this does:
  1. Initialize TWAI (CAN) at 500 kbit/s
  2. Read actual velocity from EPOS (SDO upload request to 0x606C)
  3. Set target velocity to 500 rpm (SDO download to 0x60FF)
  4. Wait for and validate SDO responses
  
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
#define VELOCITY_ACTUAL_VALUE_AVG   0x30D3]

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
  // TWAI general config
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN, 
    CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  
  // 500 kbit/s timing
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  
  // Accept all frames (no filtering)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // Install driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  
  // Start driver
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
  
  // Build SDO download request
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x23;  // Expedited download, 4 bytes
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
    Serial.print("TX failed: ");
    Serial.println(status);
    return status;
  }
  
  Serial.print("Sent SDO write to 0x");
  Serial.print(index, HEX);
  Serial.print(" = ");
  Serial.println(value);
  
  // Wait for response
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < 10) {  // 10 ms timeout
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      // Check if it's our SDO response
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
          Serial.println("SDO write confirmed");
          return CAN_OK;
        }
      }
    }
  }
  
  Serial.println("SDO response timeout");
  return CAN_RX_TIMEOUT;
}

// Send SDO Upload (read) and wait for response
CANStatus SDO_Read(uint16_t index, uint8_t subindex, uint32_t* value) {
  SimpleCANFrame tx_frame;
  
  // Build SDO upload request
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
    Serial.print("TX failed: ");
    Serial.println(status);
    return status;
  }
  
  Serial.print("Sent SDO read from 0x");
  Serial.println(index, HEX);
  
  // Wait for response
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < 10) {  // 10 ms timeout
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      // Check if it's our SDO response
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
        
        // Upload response (0x43 or 0x4B typically)
        if ((cmd & 0xE0) == 0x40) {
          *value = rx_frame.data[4] 
                 | (rx_frame.data[5] << 8)
                 | (rx_frame.data[6] << 16)
                 | (rx_frame.data[7] << 24);
          Serial.print("SDO read value: ");
          Serial.println(*value);
          return CAN_OK;
        }
      }
    }
  }
  
  Serial.println("SDO response timeout");
  return CAN_RX_TIMEOUT;
}

//=============================================================================
// SIMPLE MOTOR CONTROL FUNCTIONS
//=============================================================================

CANStatus Motor_ReadActualVelocity(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(VELOCITY_ACTUAL_VALUE_AVG, 0x01, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;  // Treat as signed
  }
  return status;
}

//=============================================================================
// ARDUINO SETUP & LOOP
//=============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Minimal CAN Test ===");
  Serial.println("Initializing TWAI (CAN)...");
  
  if (!CAN_Init()) {
    Serial.println("FAILED to initialize CAN!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("CAN initialized successfully at 500 kbit/s");
  Serial.println("TX: GPIO 14, RX: GPIO 15");
  Serial.println();
  
  delay(1000);
}

void loop() {
  // Read and print velocity every 2 seconds
  delay(2000);
  
  int32_t velocity;
  CANStatus status = Motor_ReadActualVelocity(&velocity);
  
  if (status == CAN_OK) {
    Serial.print("Velocity: ");
    Serial.println(velocity);
  } else {
    Serial.print("Read failed: ");
    Serial.println(status);
  }
}
