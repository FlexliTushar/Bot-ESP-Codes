/*
  Table 1
  ------------------------------------------
  | Bot speed | Ramp Rate | Brake Col Count|
  |  1.5m/s   | 2000rpm/s |      15        |
  |  2.0m/s   | 2000rpm/s |      23        |
  ------------------------------------------

  Table 2
  -------------------------
  | Bot speed | Motor RPM |
  |  0.05m/s  |    26     |
  |  0.10m/s  |    53     |
  |  0.50m/s  |    267    |
  |  1.00m/s  |    535    |
  |  1.50m/s  |    802    |
  |  2.00m/s  |    1070   |
  |  2.50m/s  |    1337   |
  |  3.00m/s  |    1604   |
  -------------------------
*/

#include <esp32_can.h>
#include <Adafruit_MCP23X17.h>
#include <can_common.h>
#include <object_dictionary.h>
#include <maxon.h>
#include <WiFi.h>
#include <HTTPClient.h>
using namespace std;

// Motor Speed Levels
#define S0_05 26   // 0.05 m/s
#define S0_1 53    // 0.1 m/s
#define S0_5 267   // 0.5 m/s
#define S1 535     // 1 m/s
#define S1_5 802   // 1.5 m/s
#define S2 1070    // 2 m/s
#define S2_5 1337  // 2.5 m/s
#define S3 1604    // 3 m/s

// GPIO Pin Assignments
#define PANEL_LED_PIN 8
#define PANEL_LED_BUTTON 7 

// Constant variables
const String CODE_ID = "d359-DriveMotorStandaloneTest";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";

// Logger variables
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Bot Variables
Adafruit_MCP23X17 mcp;
bool notLogged = true;
unsigned long startTime;

void setup() {
  delay(1000);
  Serial.begin(115200);
  // WiFiConfig();
  // xTaskCreatePinnedToCore(
  //   HTTP_DEBUG_LOGGER,
  //   "debug_logging",
  //   10000,
  //   NULL,
  //   1,
  //   &httpDebugLog,
  //   0);
  MCPConfig();
  CANConfig();
  PinConfig();
  MotorConfig();
  Serial.println("Bot: BB CodeID: " + CODE_ID);
  Serial.println(__FILE__);
  while (mcp.digitalRead(PANEL_LED_BUTTON)) {}
  setTagetVelocity(S2);
  startTime = millis();
}

void loop() {
  int speed = getVelocityActualValueAveraged();
  Serial.println("Speed: " + String(speed));
  delay(5000);
  if (millis() - startTime >= 120000 && notLogged) {
    Serial.println("Stopping!");
    haltMovement();
    notLogged = false;
  }
}

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (debugLoggingString != "B1 " + CODE_ID + ": ") {
      int httpCode = -1;
      if (loggerFlag) {
        httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
        httpCode = httpDebugger.POST(debugLoggingString);
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = "B1 " + CODE_ID + ": ";
      }
    }
  }
}

// Function to add logs into the log string
void add_log(String log) {
  debugLoggingString += " | " + log;
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

// Setup the pin configuration
void PinConfig() {
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
}

// Function to configure the CAN protocol
void CANConfig() {
  CAN0.setCANPins(GPIO_NUM_15, GPIO_NUM_14);  // rx tx
  CAN0.begin(500000);
  Serial.println("CAN Ready!");
}

// Function to define the configuration of driving motor
void MotorConfig() {
  disable();
  enable();
  setOperationMode(PROFILE_VELOCITY_MODE);
  WriteCAN(FOLLOWING_ERROR_WINDOW, 0x00, 2000);  // Following error window -   2000inc
  delay(CAN_DELAY);
  setMaxProfileVelocity(2080);
  setProfileVelocity(800); 
  setProfileAcceleration(1000);
  setProfileDeceleration(1000);
  setQuickStopDeceleration(1000);
  setMotionProfileType(1);
  disable();
  enable();
}