#include <CAN.h>
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>

// 12, 14 we can use for PI GPIO

// ESCON Driver Pins
#define PKT_DET_PI_PIN 25
#define ACK_PI_PIN 33
#define BLDC_PWM 26    // v3  Digital Input 1
#define BLDC_EN 27     // v3  Digital Input 2
#define BLDC_BRAKE 14  // v3  Digital Input 3
#define BLDC_DIR 12    // v3  Digital Input 4
#define CLIT_DETECTION_SENSOR 16
#define ACK_BUTTON_PIN 13

const String CODE_ID = "d282-InfeedController";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const int PACKET_TRANSFER_CURTAIN_SENSOR[] = { 5, 17, 18, 19, 23 };  // Array of packet transfer curtain sensors
const int NUMBER_OF_CURTAIN_SENSOR = sizeof(PACKET_TRANSFER_CURTAIN_SENSOR) / sizeof(PACKET_TRANSFER_CURTAIN_SENSOR[0]);

#define CONVEYOR_ON 1      // Active Low Config | For Active HIGH config set CONVEYOR_ON = 1
#define CONVEYOR_OFF 0     //                                              & CONVEYOR_OFF = 0
#define ENABLE_DRIVER 0    // Active Low Config | For Active HIGH config set ENABLE_DRIVER = 1
#define DISABLE_DRIVER 1   //                                              & DISABLE_DRIVER = 0
#define FORWARD 0          // LOW - FORWARD
#define REVERSE 1          // HIGH - REVERSE
#define INFEED_SPEED 80    // Speed Level for infeeding.
#define ROLLBACK_SPEED 35  // Speed Level for rollback.

const int BLDC_PWM_CHANNEL = 7;
const int BLDC_PWM_FREQ = 2000;
const int BLDC_PWM_RESOLUTION = 8;

#define CAN_RX_PIN 21
#define CAN_TX_PIN 22
#define CAN_SPEED 1000E3

// Structs
struct RequestToYourZoneModel {
  uint8_t requestId = -1;
  uint8_t receiverId = 0;
  uint8_t originalSenderId = 0;
  uint8_t requestType = 0;
  uint8_t data1 = 0.0;
  uint8_t data2 = 0.0;
  uint8_t data3 = 0.0;
  uint8_t data4 = 0.0;
  uint8_t data5 = 0.0;
};

struct RequestFromYourZoneModel {
  uint8_t requestId = -1;
  uint8_t senderId = 0;
  uint8_t originalSenderId = 0;
  uint8_t requestType = 0;
  uint8_t data1 = 0.0;
  uint8_t data2 = 0.0;
  uint8_t data3 = 0.0;
  uint8_t data4 = 0.0;
  uint8_t data5 = 0.0;
};

Preferences preferences;
String zoneType;
uint8_t currentZoneID;
uint8_t infeedZoneID;
int requestId = 0;
bool botDetected = false;
bool ack_active = false;
int clit_count = 0;
volatile bool clitDetected = false;
bool startConveyor = false;
bool startRollup = false;
bool stopRollup = false;
bool stuckErrorMode = false;
bool packetTransferCANSuccess = false;

// Logger variables
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

void IRAM_ATTR clitDetectedISR() {
  clitDetected = true;
}

// CANCommunicator: Sending CAN request
void SendRequest(RequestToYourZoneModel request) {
  // Begin the CAN frame
  CAN.beginPacket(currentZoneID);
  // Write relevant information - Consumes 4 bytes right now
  CAN.write(request.receiverId);
  CAN.write(request.originalSenderId);
  CAN.write(request.requestType);
  CAN.write(request.requestId);
  // Case: If condition true then write 1st part of payload as crude value else write mapped value (1 byte)
  CAN.write(request.data1);
  // End the CAN frame
  CAN.endPacket();
}

// Function to create a PacketTransferred CAN request
RequestToYourZoneModel IntimationPacketTransferredRequest() {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = infeedZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0x6;
  requestToYourZone.data1 = 1;
  return requestToYourZone;
}

// CANCommunicator: Receive the income CAN request
void RecieveRequest() {
  // Serial.println("Trying to receive!");
  unsigned long latency = micros();
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    RequestFromYourZoneModel recievedRequest;
    recievedRequest.senderId = CAN.packetId();
    if (!CAN.packetRtr()) {
      uint8_t receiverId = CAN.read();
      if (recievedRequest.senderId == 41) {
        Serial.println(receiverId);
      }
      if (receiverId == currentZoneID) {
        while (CAN.available()) {
          recievedRequest.originalSenderId = CAN.read();
          recievedRequest.requestType = CAN.read();
          recievedRequest.requestId = CAN.read();
          if (recievedRequest.requestType == 0x0 || recievedRequest.requestType == 0x3 || recievedRequest.requestType == 0x5 || recievedRequest.requestType == 0x7) {
            // Serial.println("Ack. Received!");
            recievedRequest.data1 = CAN.read();
            if (recievedRequest.requestType == 0x7) {
              botDetected = true;
              RequestToYourZoneModel ackRequest;
              ackRequest.requestId = recievedRequest.requestId;
              ackRequest.receiverId = recievedRequest.senderId;
              ackRequest.originalSenderId = currentZoneID;
              ackRequest.requestType = 0x0;

              SendRequest(ackRequest);
              add_log("Received bot presence!");
            }
          } else {
            recievedRequest.data1 = ((uint8_t)CAN.read() * 512.0 / 255.0) / 100.0;
          }
          if (recievedRequest.requestType == 0x1 || recievedRequest.requestType == 0x2 || recievedRequest.requestType == 0x5) {
            recievedRequest.data2 = ((uint8_t)CAN.read() * 512.0 / 255.0) / 100.0;
            recievedRequest.data3 = ((uint8_t)CAN.read() * 512.0 / 255.0) / 100.0;
          }
          if (recievedRequest.requestType == 0x2) {
            recievedRequest.data4 = ((uint8_t)CAN.read() * 512.0 / 255.0) / 100.0;
          }
          if (recievedRequest.requestType == 0x0) {
            packetTransferCANSuccess = true;
          }
        }
      }
    }
  }
}

void StartConveyor() {
  add_log("Starting Conveyor!");
  ledcWrite(BLDC_PWM, INFEED_SPEED);
  digitalWrite(BLDC_BRAKE, CONVEYOR_ON);
  startConveyor = true;
  add_log("Conveyor Started. Waiting for stop...");
}

void StopConveyor() {
  while(!digitalRead(CLIT_DETECTION_SENSOR)) {}
  digitalWrite(BLDC_BRAKE, CONVEYOR_OFF);
  startConveyor = false;
  add_log("Conveyor Stopped!");
}

void StartRollup() {
  add_log("Starting Roll Up!");
  ledcWrite(BLDC_PWM, ROLLBACK_SPEED);
  digitalWrite(BLDC_BRAKE, CONVEYOR_ON);
  add_log("Roll Up Started. Waiting for stop...");
}

void StopRollup() {
  while(!digitalRead(CLIT_DETECTION_SENSOR)) {}
  digitalWrite(BLDC_BRAKE, CONVEYOR_OFF);
  stopRollup = false;
  add_log("Roll Up Stopped!");
}

bool IsPacketStuck() {
  for (int i = 0; i < NUMBER_OF_CURTAIN_SENSOR; i++) {
    if (digitalRead(PACKET_TRANSFER_CURTAIN_SENSOR[i]) == HIGH) {
      return true;  // If any sensor is triggered, packet is stuck
    }
  }
  return false;
}

// Function to connect controller to network
void WiFiConfig() {
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  Serial.println("Connected to WiFi!");
}

void setup() {
  Serial.begin(115200);
  WiFiConfig();
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    10000,
    NULL,
    1,
    &httpDebugLog,
    0);
  PinConfig();
  // attachInterrupt(digitalPinToInterrupt(CLIT_DETECTION_SENSOR), clitDetectedISR, RISING);
  ConveyorConfig();
  BoardConfig();
  CANConfig();
  Serial.println("Infeed Controller zone : Z" + String(currentZoneID) + " Code ID: " + CODE_ID);
  Serial.println(__FILE__);
  debugLoggingString = "Z" + String(currentZoneID) + " " + CODE_ID + ": ";
  add_log("Infeed Controller Started!");
}

void loop() {
  // bool packetPresent = digitalRead(PKT_DET_PI_PIN);
  // Serial.println("Packet Sensor : " + String(IsPacketStuck()));
  // Serial.println(String(packetPresent) + "-" + String(botDetected));
  bool buttonPressed = !digitalRead(ACK_BUTTON_PIN);
  // Serial.println(String(buttonPressed));
  if (botDetected) {
    // delay(2000);
    add_log("Button Pressed!");
    clitDetected = false;
    StartConveyor();
    botDetected = false;
  }

  if (startConveyor) {
    StopConveyor();
    if (!startConveyor) {
      delay(500);
      if (IsPacketStuck()) {
        // Serial.println("Packet Sensor : " + String(IsPacketStuck()));
        stuckErrorMode = true;
        bool ackButtonPressed = !digitalRead(ACK_BUTTON_PIN);
        while (!ackButtonPressed || stuckErrorMode) {
          ackButtonPressed = !digitalRead(ACK_BUTTON_PIN);
          if (ackButtonPressed && !IsPacketStuck()) {
            // Serial.println("Packet Sensor 2 : " + String(IsPacketStuck()));
            stuckErrorMode = false;
            // DMS Handling
          }
        }
      }
      RequestToYourZoneModel request = IntimationPacketTransferredRequest();
      SendRequest(request);
      startRollup = true;
      while(!packetTransferCANSuccess){RecieveRequest();}
      packetTransferCANSuccess = false;
      // ack_active = true;
      // digitalWrite(ACK_PI_PIN, HIGH);
    }
  }

  if (startRollup) {
    clitDetected = false;
    StartRollup();
    startRollup = false;
    stopRollup = true;
  }

  if (stopRollup) {
    StopRollup();
  }

  // if (!packetPresent && ack_active) {
  //   digitalWrite(ACK_PI_PIN, LOW);
  //   ack_active = false;
  // }
  // delay(2000);

  RecieveRequest();
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    if (debugLoggingString != "Z" + String(currentZoneID) + " " + CODE_ID + ": ") {
      httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
      int httpCode = -1;
      if (loggerFlag) {
        httpCode = httpDebugger.POST(debugLoggingString);
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = "Z" + String(currentZoneID) + " " + CODE_ID + ": ";
      }
    }
  }
}

// Function to add logs into the log string
void add_log(String log) {
  debugLoggingString += " | " + log;
}

void ConveyorConfig() {
  ledcAttach(BLDC_PWM, BLDC_PWM_FREQ, BLDC_PWM_RESOLUTION);
  ledcWrite(BLDC_PWM, 225);
  disable_driver();
  delay(10);
  enable_driver();
  delay(10);
  digitalWrite(BLDC_DIR, FORWARD);
  digitalWrite(BLDC_BRAKE, CONVEYOR_OFF);
}

void PinConfig() {
  pinMode(BLDC_EN, OUTPUT);
  pinMode(BLDC_DIR, OUTPUT);
  pinMode(BLDC_BRAKE, OUTPUT);
  pinMode(ACK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PKT_DET_PI_PIN, INPUT_PULLDOWN);
  pinMode(ACK_PI_PIN, OUTPUT);
  pinMode(CLIT_DETECTION_SENSOR, INPUT);
  for (int i = 0; i < NUMBER_OF_CURTAIN_SENSOR; i++) {
    pinMode(PACKET_TRANSFER_CURTAIN_SENSOR[i], INPUT);
  }
}

// Configuration of the CAN communication
void CANConfig() {
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  // Start the CAN bus
  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  Serial.println("CAN Started!");
}

// Configuration function to read non-volatile memory of EEPROM
void BoardConfig() {
  preferences.begin("board-config", false);
  zoneType = preferences.getString("ZoneType", "regular");  // Default to "regular" if not set
  currentZoneID = preferences.getInt("CurrentZoneID", -1);  // Default to -1 if not set
  infeedZoneID = preferences.getInt("InfZoneID", -1);       // Default to -1 if not set

  if (currentZoneID == -1) {
    while (true)
      ;
  }

  Serial.println("Current: Z" + String(currentZoneID) + " and Infeed: Z" + String(infeedZoneID));
}

void enable_driver() {
  digitalWrite(BLDC_EN, ENABLE_DRIVER);
}

void disable_driver() {
  digitalWrite(BLDC_EN, DISABLE_DRIVER);
}