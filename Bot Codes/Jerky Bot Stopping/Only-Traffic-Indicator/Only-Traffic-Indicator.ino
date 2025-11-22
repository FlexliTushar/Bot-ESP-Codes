#include <esp32_can.h>
#include <Adafruit_MCP23X17.h>
#include <can_common.h>
#include <object_dictionary.h>
#include <maxon.h>
#include <WiFi.h>
#include <HTTPClient.h>

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
#define BUZZER 13
#define EMG_PIN 12
#define PANEL_LED_PIN 8
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35

// Constant variables
const String CODE_ID = "Only-Traffic-Indicator";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const int LOWER_THRESHOLD_INTENSITY = 200;
const int UPPER_THRESHOLD_INTENSITY = 400;

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Bot Variables
Adafruit_MCP23X17 mcp;
bool slowDownFlag = false;
bool stopBotFlag = false;
int setSpeed;
int currentSpeed;
SemaphoreHandle_t xSpeedFeedback;
int currentSpeedCore0 = 0;
bool onTraffic = false;
bool onColumn = true;
TaskHandle_t speedFeedback;
int zeroSpeedCounter = 0;

// Handler for the case when TI is OFF
void HandleTrafficIndicatorDetectionOFF() {
  if (slowDownFlag) {
    setTagetVelocity(setSpeed);
    stopBotFlag = false;
    if (setSpeed != S0_05) slowDownFlag = false;
  }
}

// Handler for the case when TI is ON
void HandleTrafficIndicatorDetectionON() {
  if (!slowDownFlag) {
    add_log("Setting min. speed");
    setTagetVelocity(S0_05);
    slowDownFlag = true;
  } else {
    if (!stopBotFlag) {
      if (currentSpeed <= 27) {
        if (!stopBotFlag) {
          add_log("Stopping");
          haltMovement();
          stopBotFlag = true;
        }
      }
    }
  }
}

// Function which tells if traffic indicator is detected or not
bool TrafficIndicatorDetection() {
  int TI_rawValue = map(analogRead(TRAFFIC_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  if (TI_rawValue < LOWER_THRESHOLD_INTENSITY) return true;
  else if (TI_rawValue > LOWER_THRESHOLD_INTENSITY && TI_rawValue < UPPER_THRESHOLD_INTENSITY) return onTraffic;
  else if (TI_rawValue > UPPER_THRESHOLD_INTENSITY) return false;
  return false;
}

// Function which tells if column indicator is detected or not
bool ColumnIndicatorDetection() {
  int CI_rawValue = map(analogRead(COLUMN_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  if (CI_rawValue < LOWER_THRESHOLD_INTENSITY) return true;
  else if (CI_rawValue > LOWER_THRESHOLD_INTENSITY && CI_rawValue < UPPER_THRESHOLD_INTENSITY) return onColumn;
  else if (CI_rawValue > UPPER_THRESHOLD_INTENSITY) return false;
  return false;
}

void setup() {
  Serial.begin(115200);
  xSpeedFeedback = xSemaphoreCreateBinary();
  xSemaphoreGive(xSpeedFeedback);
  WiFiConfig();
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    10000,
    NULL,
    1,
    &httpDebugLog,
    0);
  MCPConfig();
  CANConfig();
  PinConfig();
  delay(2000);
  MotorConfig();
  Beep();
  delay(1000);
  xTaskCreatePinnedToCore(
    SPEED_FEEDBACK,
    "speed_feedback",
    4096,
    NULL,
    2,
    &speedFeedback,
    0);
  delay(1000);
  setSpeed = S2;
  setTagetVelocity(setSpeed);
  Serial.println("Code: " + CODE_ID);
  Serial.println(__FILE__);
  add_log("Starting the bot!");
}

void loop() {
  // ------------------------------------------------------------------------- WTM -------------------------------------------------------------
  if (xSemaphoreTake(xSpeedFeedback, portMAX_DELAY) == pdTRUE) {
    currentSpeed = currentSpeedCore0;
    xSemaphoreGive(xSpeedFeedback);
  }

  bool trafficDetected = TrafficIndicatorDetection();
  if (trafficDetected) {
    mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    if (onTraffic) {

    } else {
      onTraffic = true;
      HandleTrafficIndicatorDetectionON();
    }
  } else {
    if (onTraffic) {
      onTraffic = false;
    }
    if (stopBotFlag && currentSpeed == 0) {
      zeroSpeedCounter++;
      if (zeroSpeedCounter == 50) {
        add_log("Correction");
        setTagetVelocity(S0_05);
        stopBotFlag = false;
      }
    } else {
      zeroSpeedCounter = 0;
    }
    mcp.digitalWrite(PANEL_LED_PIN, LOW);
  }
}

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
  }
}

// Speed feedback function which will run as RTOS task
void SPEED_FEEDBACK(void* pvParameters) {
  while (true) {
    int curSpeed = getVelocityActualValueAveraged();
    if (xSemaphoreTake(xSpeedFeedback, portMAX_DELAY) == pdTRUE) {
      currentSpeedCore0 = curSpeed;
      xSemaphoreGive(xSpeedFeedback);
    }
    vTaskDelay(10/ portTICK_PERIOD_MS);
  }
}

// Function to make ahardware beep for 250 ms
void Beep() {
  digitalWrite(BUZZER, HIGH);
  mcp.digitalWrite(PANEL_LED_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
  mcp.digitalWrite(PANEL_LED_PIN, LOW);
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (debugLoggingString != CODE_ID + ": ") {
      int httpCode = -1;
      if (loggerFlag) {
        httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
        httpCode = httpDebugger.POST(debugLoggingString);
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = CODE_ID + ": ";
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
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT_PULLUP);
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT_PULLUP);
  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  digitalWrite(EMG_PIN, HIGH);
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
  setProfileAcceleration(800);
  setProfileDeceleration(800);
  setQuickStopDeceleration(800);
  setMotionProfileType(1);
  disable();
  enable();
}