/*******************************************************************************
 * M-Sort Station Reader Standalone Firmware
 * 
 * Copyright (c) 2025 Flexli Technologies
 * All rights reserved.
 * 
 * Hardware: ESP32 Development Board with CAN Bus, Servo Controllers, Photo Diodes
 * Board Version: ESP32-WROOM-DA-Module
 * 
 * Description:
 *   Autonomous mobile robot firmware for station recognition, navigation, and
 *   path diversion control. Uses optical sensors to read station codes and
 *   controls servo-based diverters for routing decisions.
 * 
 * Features:
 *   - Optical station code reading (3 photodiode array)
 *   - CAN bus motor control (Maxon EPOS4)
 *   - Dual servo diverter control with position feedback
 *   - Emergency stop handling with debouncing
 *   - WiFi-based debug logging
 *   - Watchdog timer protection
 *   - Non-blocking error recovery
 *   - Station validation and localization
 * 
 * Safety Features:
 *   - Hardware watchdog timer (10s timeout)
 *   - Emergency stop with immediate motor halt
 *   - Servo position validation
 *   - Station code verification
 *   - Communication error recovery
 *   - System health monitoring
 * 
 * Firmware Version: 2.0.0
 * Build Date: __DATE__ __TIME__
 * Compiler: __VERSION__
 * 
 * CHANGE LOG:
 * v2.0.0 (2025-11-12):
 *   - Added watchdog timer protection
 *   - Implemented non-blocking error handling
 *   - Added emergency stop with debouncing
 *   - WiFi connection with timeout
 *   - State machine architecture
 *   - Improved servo error recovery
 *   - Memory management improvements
 *   - Comprehensive error codes
 *   - System diagnostics
 * 
 * v1.0.0 (Previous):
 *   - Initial release
 *   - Basic station reading
 *   - Servo control
 *   - CAN motor interface
 * 
 * Hardware Requirements:
 *   - ESP32 Dev Module (min 4MB flash)
 *   - CAN transceiver (GPIO 14/15)
 *   - MCP23017 I2C GPIO expander
 *   - 3x Photo diode sensors (GPIO 34, 36, 39)
 *   - 2x Serial servos (UART2)
 *   - Emergency stop button (GPIO 12)
 *   - Buzzer (GPIO 13)
 * 
 * Configuration:
 *   WiFi credentials, servo limits, and station maps are configurable
 *   Diverter calibration stored in NVM (Preferences)
 * 
 * Motor Speed Reference:
 *   ------------------------------------------
 *   | Bot speed | Ramp Rate | Brake Col Count|
 *   |  1.5m/s   | 2000rpm/s |      15        |
 *   |  2.0m/s   | 2000rpm/s |      23        |
 *   ------------------------------------------
 * 
 * Speed to RPM Mapping:
 *   -------------------------
 *   | Bot speed | Motor RPM |
 *   |  0.05m/s  |    26     |
 *   |  0.10m/s  |    53     |
 *   |  0.50m/s  |    267    |
 *   |  1.00m/s  |    535    |
 *   |  1.50m/s  |    802    |
 *   |  2.00m/s  |    1070   |
 *   |  2.50m/s  |    1337   |
 *   |  3.00m/s  |    1604   |
 *   -------------------------
 * 
 ******************************************************************************/

#include <esp32_can.h>
#include <Adafruit_MCP23X17.h>
#include <can_common.h>
#include <object_dictionary.h>
#include <maxon.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SMS.h>
#include <unordered_map>
#include <string>
#include <Preferences.h>
using namespace std;

//=============================================================================
// FIRMWARE VERSION AND BUILD INFORMATION
//=============================================================================
#define FIRMWARE_VERSION "2.0.0"
#define FIRMWARE_BUILD_DATE __DATE__
#define FIRMWARE_BUILD_TIME __TIME__
#define HARDWARE_VERSION "d438"
#define MANUFACTURER "Flexli Technologies"

//=============================================================================
// ERROR CODE DEFINITIONS
//=============================================================================
enum ErrorCode {
  ERR_NONE = 0,
  ERR_WIFI_CONNECTION_FAILED = 1,
  ERR_MCP_INIT_FAILED = 2,
  ERR_UNEXPECTED_STATION = 3,
  ERR_INVALID_STATION_CODE = 4,
  ERR_SERVO_COMMUNICATION_FAILED = 5,
  ERR_CAN_INIT_FAILED = 6,
  ERR_WATCHDOG_RESET = 7,
  ERR_SENSOR_FAILURE = 8,
  ERR_MEMORY_LOW = 9,
  ERR_CONFIG_INVALID = 10,
  ERR_MOTOR_COMMUNICATION = 11,
  ERR_DIVERTER_POSITION_INVALID = 12,
  ERR_LOOP_TIME_EXCEEDED = 13,
  ERR_EMERGENCY_STOP = 14
};

// Error severity levels
enum ErrorSeverity {
  SEVERITY_INFO = 0,
  SEVERITY_WARNING = 1,
  SEVERITY_ERROR = 2,
  SEVERITY_CRITICAL = 3,
  SEVERITY_FATAL = 4
};

//=============================================================================
// SYSTEM CONFIGURATION CONSTANTS
//=============================================================================

// Motor Speed Levels (RPM)
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
#define EMG_PIN 12
#define PANEL_LED_BUTTON 7 
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define AFC_PIN_1 19
#define AFC_PIN_2 18
#define COLUMN_INDICATOR_SENSOR 33

#define ENABLE 1
#define DISABLE 0

// Watchdog timer configuration
#define WDT_TIMEOUT 10  // 10 seconds watchdog timeout

// Timing constants for non-blocking operations
#define WIFI_CONNECT_TIMEOUT 30000  // 30 seconds
#define ERROR_BUTTON_TIMEOUT 60000  // 60 seconds for button press in error mode
#define SERVO_DELAY_TIME 10         // Non-blocking servo delay
#define EMG_DEBOUNCE_TIME 50        // Emergency button debounce time in ms
#define MAX_LOOP_TIME_MS 100        // Maximum acceptable loop time in ms
#define SERVO_POSITION_TOLERANCE 30 // Position tolerance for servo verification

// Sensor configuration
#define SENSOR_SAMPLE_WINDOW 5      // Number of samples for median filter
#define SENSOR_FAIL_THRESHOLD 10    // Consecutive failures before sensor error
#define MIN_COLUMN_FRAMES 5         // Minimum frames to validate column reading

// Memory management
#define LOG_BUFFER_SIZE 512         // Fixed size for log buffer
#define MAX_LOG_ENTRIES 20          // Maximum log entries before flush

// CAN Bus configuration
#define CAN_RETRY_COUNT 3           // CAN transmission retry attempts
#define CAN_TIMEOUT_MS 100          // CAN operation timeout

// System health check intervals
#define HEALTH_CHECK_INTERVAL 5000  // 5 seconds between health checks
#define MEMORY_CHECK_INTERVAL 10000 // 10 seconds between memory checks

// Configuration validation
#define CONFIG_MAGIC_NUMBER 0xA5C3  // Magic number for config validation
#define MIN_SERVO_LIMIT 1000        // Minimum acceptable servo position
#define MAX_SERVO_LIMIT 4000        // Maximum acceptable servo position

// Constant variables
const String CODE_ID = "d438";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const int THRESHOLD_INTENSITY = 250;
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const static int SR_D1 = 36;  // ESP32 pin attached to Photo diode 1
const static int SR_D2 = 39;  // ESP32 pin attached to Photo diode 2
const static int SR_D3 = 34;  // ESP32 pin attached to Photo diode 3
const static uint16_t PT_UPPER_THR_LIMIT = 600;  // Threshold limit for rising photo diode reading.
const static uint16_t PT_LOWER_THR_LIMIT = 400;  // Threshold limit for falling photo diode reading

// Monitoring variables
unsigned long maxLoopTime;
unsigned long loopStartTime;
int numsOfIteration = 0;
unsigned long maxLatency = 0;
size_t maxMemoryUsed = 0;
const int MAX_REATTEMPTS_FOR_SERVO_COM = 5;

//=============================================================================
// MEMORY MANAGEMENT - Fixed Buffers (No String class to prevent fragmentation)
//=============================================================================
char logBuffer[LOG_BUFFER_SIZE];           // Fixed size log buffer
int logBufferPos = 0;                      // Current position in log buffer
char botIdBuffer[10] = "B";                // Fixed bot ID buffer
char stationCodeBuffer[10];                // Fixed station code buffer
char currentStationBuffer[10] = "X";       // Current station ID buffer
char expectedStationBuffer[10] = "X";      // Expected station ID buffer

// Helper function to safely append to log buffer
void appendToLogBuffer(const char* msg) {
  int msgLen = strlen(msg);
  if (logBufferPos + msgLen + 3 < LOG_BUFFER_SIZE) {  // +3 for " | " separator
    if (logBufferPos > 0) {
      strncpy(logBuffer + logBufferPos, " | ", 3);
      logBufferPos += 3;
    }
    strncpy(logBuffer + logBufferPos, msg, msgLen);
    logBufferPos += msgLen;
    logBuffer[logBufferPos] = '\0';  // Null terminate
  }
}

// Helper function to clear log buffer
void clearLogBuffer() {
  logBufferPos = 0;
  logBuffer[0] = '\0';
  // Add bot ID and code ID as prefix
  snprintf(logBuffer, LOG_BUFFER_SIZE, "%s %s:", botIdBuffer, HARDWARE_VERSION);
  logBufferPos = strlen(logBuffer);
}

// Logger variables
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Bot Variables
Adafruit_MCP23X17 mcp;
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

unordered_map<string, string> STATION_CODE_TO_STATION_ID_MAP = {
  { "100011110", "D1" }, { "101110011", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, { "101011010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010011", "D8" }, { "100010101", "D9" }, { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100110100", "D13" }, { "100101100", "D14" }, { "100101110", "D15" }, { "100110001", "D16" }, { "100110010", "D17" }, { "100110011", "D18" }, { "110010100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }
};

unordered_map<string, bool> STATION_ID_TO_DIRECTION_MAP = {
  { "D1", 0 }, { "D2", 0 }, { "D3", 1 }, { "D4", 1 }, { "D5", 1 }, { "D6", 1 }, { "D7", 1 }, { "D8", 1 }, { "D9", 1 }, { "D10", 1 }, { "D11", 1 }, { "D12", 1 }, { "D13", 1 }, { "D14", 1 }, { "D15", 1 }, { "D16", 1 }, { "D17", 1 }, { "D18", 1 }, { "I1", 1 }, { "I11", 0 }, { "I12", 1 }
};
unordered_map<string, string> STATION_ID_TO_EXPECTED_STATION_ID_MAP = {
  { "D1", "I1" }, { "D2", "D3" }, { "D3", "D4" }, { "D4", "D5" }, { "D5", "D6" }, { "D6", "D7" }, { "D7", "D8" }, { "D8", "D9" }, { "D9", "D10" }, { "D10", "D11" }, { "D11", "D12" }, { "D12", "D13" }, { "D13", "D14" }, { "D14", "D15" }, { "D15", "D16" }, { "D16", "D17" }, { "D17", "D18" }, { "D18", "D1" }, { "I1", "I12" }, { "I11", "D2" }, { "I12", "I11" }
};

unordered_map<string, int> STATION_ID_TO_SPEED_MAP = {
  { "D1", S1 }, { "D2", S2 }, { "D3", S2 }, { "D4", S2 }, { "D5", S2 }, { "D6", S2 }, { "D7", S2 }, { "D8", S2 }, { "D9", S2 }, { "D10", S2 }, { "D11", S2 }, { "D12", S2 }, { "D13", S2 }, { "D14", S2 }, { "D15", S2 }, { "D16", S2 }, { "D17", S2 }, { "D18", S2 }, { "I1", S0_3 }, { "I11", S1 }, { "I12", S1 }
};

SMS sm;
bool rightDivertor = true;
Preferences prefs;
int errorCode = -1;
bool errorMode = false;
String expectedStation = "X";
bool localisationFlag = true;
bool diverterCheckFlag = false;
bool previousCIStateDiversionCheck = 0;
int columnCounterDiversioncheck = 0;

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

// Infeed stop variables
int _intermediateColumnFramesCount = 0;
int _instantaneousFramesCount = 0;
uint16_t _instantaneousIntensityMaxArray[3] = { 0 };
uint16_t _instantaneousIntensityMinArray[3] = { 1023 };
unsigned long stationReadStartTime;
int columnNo = 0;

// Non-blocking timing variables
unsigned long lastServoDelayTime = 0;
unsigned long errorModeStartTime = 0;
bool waitingForButton = false;
bool wifiConnected = false;
unsigned long wifiConnectStartTime = 0;

// Emergency stop variables
unsigned long lastEmgDebounceTime = 0;
bool emgButtonState = HIGH;
bool lastEmgButtonReading = HIGH;
bool emergencyStopActive = false;

// System state enum for better error handling
enum SystemState {
  STATE_INITIALIZING,
  STATE_NORMAL_OPERATION,
  STATE_ERROR_WAITING_BUTTON,
  STATE_ERROR_RECOVERING,
  STATE_EMERGENCY_STOP,
  STATE_WIFI_CONNECTING
};

SystemState currentSystemState = STATE_INITIALIZING;

//=============================================================================
// SYSTEM DIAGNOSTICS AND HEALTH MONITORING
//=============================================================================
struct SystemDiagnostics {
  unsigned long maxLoopTime;
  unsigned long avgLoopTime;
  unsigned long loopCount;
  unsigned long loopTimeSum;
  size_t minFreeHeap;
  size_t currentFreeHeap;
  uint16_t photodiodeFailCount[3];
  uint16_t servoCommFailCount;
  uint16_t canCommFailCount;
  unsigned long lastHealthCheck;
  unsigned long lastMemoryCheck;
  uint8_t systemHealth;  // 0-100%
  bool sensorsHealthy;
  bool communicationHealthy;
};

SystemDiagnostics sysDiag = {0};

// Sensor validation with median filtering
struct SensorData {
  uint16_t rawValues[SENSOR_SAMPLE_WINDOW];
  uint8_t sampleIndex;
  uint16_t filteredValue;
  uint8_t consecutiveFailures;
  bool isHealthy;
};

SensorData sensorArray[3] = {0};

// Calculate median of sensor samples
uint16_t calculateMedian(uint16_t* values, uint8_t count) {
  // Simple bubble sort for small array
  uint16_t sorted[SENSOR_SAMPLE_WINDOW];
  memcpy(sorted, values, count * sizeof(uint16_t));
  
  for (uint8_t i = 0; i < count - 1; i++) {
    for (uint8_t j = 0; j < count - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        uint16_t temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  
  return (count % 2 == 0) ? (sorted[count/2 - 1] + sorted[count/2]) / 2 : sorted[count/2];
}

// Update system health metrics
void updateSystemHealth() {
  unsigned long currentTime = millis();
  
  // Update loop time average
  if (sysDiag.loopCount > 0) {
    sysDiag.avgLoopTime = sysDiag.loopTimeSum / sysDiag.loopCount;
  }
  
  // Check memory health periodically
  if (currentTime - sysDiag.lastMemoryCheck > MEMORY_CHECK_INTERVAL) {
    sysDiag.currentFreeHeap = ESP.getFreeHeap();
    if (sysDiag.minFreeHeap == 0 || sysDiag.currentFreeHeap < sysDiag.minFreeHeap) {
      sysDiag.minFreeHeap = sysDiag.currentFreeHeap;
    }
    sysDiag.lastMemoryCheck = currentTime;
    
    // Memory warning threshold (less than 20KB free)
    if (sysDiag.currentFreeHeap < 20480) {
      char msg[64];
      snprintf(msg, 64, "WARN: Low memory: %d bytes", sysDiag.currentFreeHeap);
      appendToLogBuffer(msg);
    }
  }
  
  // Calculate overall system health (0-100%)
  uint8_t health = 100;
  
  // Deduct for loop time issues
  if (sysDiag.maxLoopTime > MAX_LOOP_TIME_MS) health -= 20;
  
  // Deduct for sensor issues
  if (!sysDiag.sensorsHealthy) health -= 30;
  
  // Deduct for communication issues
  if (!sysDiag.communicationHealthy) health -= 30;
  
  // Deduct for memory issues
  if (sysDiag.currentFreeHeap < 20480) health -= 20;
  
  sysDiag.systemHealth = (health > 100) ? 0 : health;
}

// Emergency stop handling with debouncing
void CheckEmergencyStop() {
  int reading = digitalRead(EMG_PIN);
  
  if (reading != lastEmgButtonReading) {
    lastEmgDebounceTime = millis();
  }
  
  if ((millis() - lastEmgDebounceTime) > EMG_DEBOUNCE_TIME) {
    if (reading != emgButtonState) {
      emgButtonState = reading;
      
      // If emergency button is pressed (LOW for active emergency)
      if (emgButtonState == LOW && !emergencyStopActive) {
        emergencyStopActive = true;
        currentSystemState = STATE_EMERGENCY_STOP;
        haltMovement();
        add_log("EMERGENCY STOP ACTIVATED");
        digitalWrite(BUZZER, HIGH);
      }
      // If emergency button is released
      else if (emgButtonState == HIGH && emergencyStopActive) {
        emergencyStopActive = false;
        digitalWrite(BUZZER, LOW);
        add_log("Emergency stop cleared - system requires restart");
        // System will need manual reset/restart after emergency stop
      }
    }
  }
  
  lastEmgButtonReading = reading;
}

// Watchdog timer initialization and feeding
void InitWatchdog() {
  // Enable watchdog with WDT_TIMEOUT seconds
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);  // Add current task to watchdog
  add_log("Watchdog timer initialized: " + String(WDT_TIMEOUT) + "s timeout");
}

void FeedWatchdog() {
  esp_task_wdt_reset();
}


  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationId.c_str());
  if (STATION_ID_TO_SPEED_MAP.find(stationCodeString) != STATION_ID_TO_SPEED_MAP.end()) {
    int speed = STATION_ID_TO_SPEED_MAP.at(stationCodeString);
    return speed;
  }
  return S0_5;
}

// Improved sensor reading with validation and filtering
void ReadStationViaPhotoDiode() {
  const int sensorPins[3] = {SR_D1, SR_D2, SR_D3};
  
  for (int i = 0; i < 3; i++) {
    _photoTransistorPreviousStateArray[i] = _photoTransistorCurrentStateArray[i];
    
    // Read raw analog value
    uint16_t rawValue = analogRead(sensorPins[i]);
    
    // Validate reading is within expected range
    if (rawValue < 50 || rawValue > 4000) {
      sensorArray[i].consecutiveFailures++;
      if (sensorArray[i].consecutiveFailures >= SENSOR_FAIL_THRESHOLD) {
        sensorArray[i].isHealthy = false;
        sysDiag.photodiodeFailCount[i]++;
        sysDiag.sensorsHealthy = false;
        char msg[48];
        snprintf(msg, 48, "SENSOR%d FAIL: Out of range (%d)", i+1, rawValue);
        add_log(msg);
      }
      continue;  // Skip this reading
    } else {
      sensorArray[i].consecutiveFailures = 0;
      sensorArray[i].isHealthy = true;
    }
    
    // Store in circular buffer for median filtering
    sensorArray[i].rawValues[sensorArray[i].sampleIndex] = rawValue;
    sensorArray[i].sampleIndex = (sensorArray[i].sampleIndex + 1) % SENSOR_SAMPLE_WINDOW;
    
    // Calculate median filtered value
    sensorArray[i].filteredValue = calculateMedian(sensorArray[i].rawValues, SENSOR_SAMPLE_WINDOW);
    
    // Map to 10-bit range
    _photoTransistorCurrentValueArray[i] = map(sensorArray[i].filteredValue, 0, 4096, 0, 1024);
    
    // Apply hysteresis thresholds
    if (_photoTransistorCurrentValueArray[i] > PT_UPPER_THR_LIMIT) {
      _photoTransistorCurrentStateArray[i] = 1;
    } else if (_photoTransistorCurrentValueArray[i] < PT_LOWER_THR_LIMIT) {
      _photoTransistorCurrentStateArray[i] = 0;
    }
    // If between thresholds, maintain previous state (hysteresis)
  }
  
  // Update sensor health status
  bool allHealthy = true;
  for (int i = 0; i < 3; i++) {
    if (!sensorArray[i].isHealthy) allHealthy = false;
  }
  sysDiag.sensorsHealthy = allHealthy;
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
      add_log("Photodiode " + String(i + 1) + ": " + status);
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
    add_log("Inst. Col: " + prevInstColumnCode);
    add_log("Inst. FC: " + String(_instantaneousFramesCount));
    add_log("Inst. Max. Intensity: { " + String(_instantaneousIntensityMaxArray[0]) + ", " + String(_instantaneousIntensityMaxArray[1]) + ", " + String(_instantaneousIntensityMaxArray[2]) + " }");
    add_log("Inst. Min. Intensity: { " + String(_instantaneousIntensityMinArray[0]) + ", " + String(_instantaneousIntensityMinArray[1]) + ", " + String(_instantaneousIntensityMinArray[2]) + " }");
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
    add_log("Read time: " + String(millis() - stationReadStartTime));
    if (columnNo < 3) add_log("Bot has skipped " + String(3 - columnNo) + " columns");
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
      if (_intermediateColumnFramesCount >= MIN_COLUMN_FRAMES) {
        _finalColumnCodeArray[2] = _finalColumnCodeArray[1];
        _finalColumnCodeArray[1] = _finalColumnCodeArray[0];
        _finalColumnCodeArray[0] = _previousIntermediateColumnCode;
        columnNo++;
        add_log("Col No.: " + String(columnNo));
        if (columnNo > 3) add_log("Bot is reading more than 3 columns");
      } else {
        char msg[64];
        snprintf(msg, 64, "Fluke column rejected: %d frames", _intermediateColumnFramesCount);
        add_log(msg);
      }
      add_log("Inter. Col: " + _previousIntermediateColumnCode);
      add_log("Inter. FC: " + String(_intermediateColumnFramesCount));
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

String GetExpectedStationFromStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_ID_TO_EXPECTED_STATION_ID_MAP.find(stationCodeString) != STATION_ID_TO_EXPECTED_STATION_ID_MAP.end()) {
    string stationID = STATION_ID_TO_EXPECTED_STATION_ID_MAP.at(stationCodeString);
    return String(stationID.c_str());
  }
  return "";
}

bool CheckStationValidity(const String &stationCode) {
  // Check if the station-code is present in the keys of STATION_CODE_TO_STATION_ID hash-map
  string stationCodeString = string(stationCode.c_str());
  return STATION_CODE_TO_STATION_ID_MAP.find(stationCodeString) != STATION_CODE_TO_STATION_ID_MAP.end();
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

// Flap and diverter functions
void DivertLeft() {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
    FeedWatchdog();  // Feed watchdog during retry loop
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    add_log("ER5: Failed to read front servo position in DivertLeft");
    return;
  }
  
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
    FeedWatchdog();  // Feed watchdog during retry loop
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    add_log("ER5: Failed to read rear servo position in DivertLeft");
    return;
  }
  
  if (current_position_front_servo < front_diverter_left_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to disable front servo torque");
      return;
    }
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_left_thresold, 100);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to write front servo position");
      return;
    }
  }
  
  if (current_position_rear_servo > rear_diverter_left_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(REAR_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to disable rear servo torque");
      return;
    }
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_left_thresold, 100);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to write rear servo position");
      return;
    }
  }
}

void DivertRight() {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
    FeedWatchdog();  // Feed watchdog during retry loop
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    add_log("ER5: Failed to read front servo position in DivertRight");
    return;
  }
  
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
    FeedWatchdog();  // Feed watchdog during retry loop
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    add_log("ER5: Failed to read rear servo position in DivertRight");
    return;
  }
  
  if (current_position_front_servo > front_diverter_right_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to disable front servo torque");
      return;
    }
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_right_thresold, 100);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to write front servo position");
      return;
    }
  }
  
  if (current_position_rear_servo < rear_diverter_right_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(REAR_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to disable rear servo torque");
      return;
    }
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_right_thresold, 100);
      code = sm.getLastError();
      count++;
      FeedWatchdog();  // Feed watchdog during retry loop
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      add_log("ER5: Failed to write rear servo position");
      return;
    }
  }
}

void DiverterTest() {
  DivertLeft();
  unsigned long testStartTime = millis();
  while(millis() - testStartTime < 1000) {
    FeedWatchdog();
    delay(10);
  }
  DivertRight();
  testStartTime = millis();
  while(millis() - testStartTime < 1000) {
    FeedWatchdog();
    delay(10);
  }
}

void SetDiverter(bool direction) {
  if (direction) {
    if (!rightDivertor) {
      DivertRight();
      rightDivertor = true;
    }
  } else {
    if(rightDivertor) {
      DivertLeft();
      rightDivertor = false;
    }
  }
}

void ErrorHandling() {
  // Non-blocking error handling with state machine
  if (errorCode == 3) {
    if (currentSystemState != STATE_ERROR_WAITING_BUTTON && currentSystemState != STATE_ERROR_RECOVERING) {
      add_log("ER3: UNEXPECTED_STATION_ERROR");
      add_log("Expected Station: " + expectedStation + " Detected Station: " + currentStation);
      Beep();
      currentSystemState = STATE_ERROR_WAITING_BUTTON;
      errorModeStartTime = millis();
      waitingForButton = true;
      haltMovement();
    }
    
    if (currentSystemState == STATE_ERROR_WAITING_BUTTON) {
      // Check button press with timeout
      if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
        waitingForButton = false;
        currentSystemState = STATE_ERROR_RECOVERING;
        setTagetVelocity(S0_5);
      } else if (millis() - errorModeStartTime > ERROR_BUTTON_TIMEOUT) {
        add_log("ER3: Button press timeout - system halted");
        haltMovement();
        waitingForButton = false;
        return;
      }
    }
    
    if (currentSystemState == STATE_ERROR_RECOVERING) {
      String stationCodeMain = "";
      ReadStationViaPhotoDiode();
      stationCodeMain = GetStationCodeFromDataReadByStationSensor();
      
      if (stationCodeMain != "") {
        haltMovement();
        unsigned long haltStart = millis();
        while(millis() - haltStart < 2000) {
          FeedWatchdog();
          delay(10);
        }
        currentStation = TransformStationCodeIntoStationId(stationCodeMain);
        if (CheckStationValidity(stationCodeMain)) {
          bool direction = GetDirectionFromStationId(currentStation);
          SetDiverter(direction);
          haltStart = millis();
          while(millis() - haltStart < 1000) {
            FeedWatchdog();
            delay(10);
          }
          setTagetVelocity(GetSetSpeedAsPerStation(currentStation));
          expectedStation = GetExpectedStationFromStationId(currentStation);
          errorCode = -1;
          errorMode = false;
          currentSystemState = STATE_NORMAL_OPERATION;
        } else {
          errorMode = true;
          errorCode = 4;
        }
      }
    }
  }

  if (errorCode == 5) {
    if (currentSystemState != STATE_ERROR_WAITING_BUTTON && currentSystemState != STATE_ERROR_RECOVERING) {
      add_log("ER5: FAILED_DIVERTER_ERROR");
      Beep();
      currentSystemState = STATE_ERROR_WAITING_BUTTON;
      errorModeStartTime = millis();
      waitingForButton = true;
      haltMovement();
    }
    
    if (currentSystemState == STATE_ERROR_WAITING_BUTTON) {
      // Check button press with timeout
      if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
        waitingForButton = false;
        currentSystemState = STATE_ERROR_RECOVERING;
        setTagetVelocity(S0_5);
      } else if (millis() - errorModeStartTime > ERROR_BUTTON_TIMEOUT) {
        add_log("ER5: Button press timeout - system halted");
        haltMovement();
        waitingForButton = false;
        return;
      }
    }
    
    if (currentSystemState == STATE_ERROR_RECOVERING) {
      String stationCodeMain = "";
      ReadStationViaPhotoDiode();
      stationCodeMain = GetStationCodeFromDataReadByStationSensor();
      
      if (stationCodeMain != "") {
        haltMovement();
        unsigned long haltStart = millis();
        while(millis() - haltStart < 2000) {
          FeedWatchdog();
          delay(10);
        }
        currentStation = TransformStationCodeIntoStationId(stationCodeMain);
        if (CheckStationValidity(stationCodeMain)) {
          bool direction = GetDirectionFromStationId(currentStation);
          SetDiverter(direction);
          haltStart = millis();
          while(millis() - haltStart < 1000) {
            FeedWatchdog();
            delay(10);
          }
          setTagetVelocity(GetSetSpeedAsPerStation(currentStation));
          expectedStation = GetExpectedStationFromStationId(currentStation);
          errorCode = -1;
          errorMode = false;
          currentSystemState = STATE_NORMAL_OPERATION;
        } else {
          errorMode = true;
          errorCode = 4;
        }
      }
    }
  }
  
  if (errorCode == 4) {
    if (currentSystemState != STATE_ERROR_WAITING_BUTTON && currentSystemState != STATE_ERROR_RECOVERING) {
      add_log("ER4: INVALID_STATION_ERROR");
      Beep();
      currentSystemState = STATE_ERROR_WAITING_BUTTON;
      errorModeStartTime = millis();
      waitingForButton = true;
      haltMovement();
    }
    
    if (currentSystemState == STATE_ERROR_WAITING_BUTTON) {
      // Check button press with timeout
      if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
        waitingForButton = false;
        currentSystemState = STATE_ERROR_RECOVERING;
        setTagetVelocity(S0_5);
      } else if (millis() - errorModeStartTime > ERROR_BUTTON_TIMEOUT) {
        add_log("ER4: Button press timeout - system halted");
        haltMovement();
        waitingForButton = false;
        return;
      }
    }
    
    if (currentSystemState == STATE_ERROR_RECOVERING) {
      String stationCodeMain = "";
      ReadStationViaPhotoDiode();
      stationCodeMain = GetStationCodeFromDataReadByStationSensor();
      
      if (stationCodeMain != "") {
        haltMovement();
        unsigned long haltStart = millis();
        while(millis() - haltStart < 2000) {
          FeedWatchdog();
          delay(10);
        }
        currentStation = TransformStationCodeIntoStationId(stationCodeMain);
        if (CheckStationValidity(stationCodeMain)) {
          bool direction = GetDirectionFromStationId(currentStation);
          SetDiverter(direction);
          haltStart = millis();
          while(millis() - haltStart < 1000) {
            FeedWatchdog();
            delay(10);
          }
          setTagetVelocity(GetSetSpeedAsPerStation(currentStation));
          expectedStation = GetExpectedStationFromStationId(currentStation);
          errorCode = -1;
          errorMode = false;
          currentSystemState = STATE_NORMAL_OPERATION;
        } else {
          add_log("Two consecutive invalid station found - Please check the station codes in the system.");
          // System requires manual intervention
        }
      }
    }
  }
}

// Function which tells if column indicator is detected or not
bool ColumnIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int CI_rawValue = map(analogRead(COLUMN_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then column detected else not detected
  return CI_rawValue < THRESHOLD_INTENSITY;
}

void CheckDiverter(bool columnDetected) {
  if (diverterCheckFlag) {
    if (columnDetected) {
      if(!previousCIStateDiversionCheck) {
          columnCounterDiversioncheck++;
          if(columnCounterDiversioncheck >= 15) {
            int code = 1;
            int count = 0;
            while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
              current_position_front_servo = sm.ReadPos(FRONT_SERVO);
              code = sm.getLastError();
              count++;
              FeedWatchdog();  // Feed watchdog during retry loop
            }
            if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
              errorMode = true;
              errorCode = 5;
              add_log("ER5: Failed to read front servo in CheckDiverter");
              return;
            }
            
            code = 1;
            count = 0;
            while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
              current_position_rear_servo = sm.ReadPos(REAR_SERVO);
              code = sm.getLastError();
              count++;
              FeedWatchdog();  // Feed watchdog during retry loop
            }
            if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
              errorMode = true;
              errorCode = 5;
              add_log("ER5: Failed to read rear servo in CheckDiverter");
              return;
            }
            
            bool direction = GetDirectionFromStationId(currentStation);
            if (direction) {
              if (current_position_front_servo > front_diverter_right_thresold - 30 && current_position_rear_servo < rear_diverter_right_thresold + 30) {
                // Correct diversion
              } else {
                errorMode = true;
                errorCode = 5;
                add_log("Current front servo position: " + String(current_position_front_servo) + " Current rear servo position: " + String(current_position_rear_servo));
                add_log("Front diverter right threshold: " + String(front_diverter_right_thresold) + " Rear diverter right threshold: " + String(rear_diverter_right_thresold));
              }
            } else {
              if (current_position_front_servo < front_diverter_left_thresold + 30 && current_position_rear_servo > rear_diverter_left_thresold - 30) {
                // Correct diversion
              } else {
                errorMode = true;
                errorCode = 5;
                add_log("Current front servo position: " + String(current_position_front_servo) + " Current rear servo position: " + String(current_position_rear_servo));
                add_log("Front diverter left threshold: " + String(front_diverter_left_thresold) + " Rear diverter left threshold: " + String(rear_diverter_left_thresold));
              }
            }
            columnCounterDiversioncheck = 0;
            diverterCheckFlag = false;
          }
        }
      }
    previousCIStateDiversionCheck = columnDetected;
  }
}

void setup() {
  MCPConfig();
  PinConfig();
  
  // Wait for stable power-up with watchdog feed
  unsigned long startupDelay = millis();
  while(millis() - startupDelay < 3000) {
    delay(10);
  }
  
  Serial.begin(115200);
  delay(100);
  
  // Print firmware information
  Serial.println("\n" + String('=', 60));
  Serial.println("M-SORT STATION READER FIRMWARE");
  Serial.println(String('=', 60));
  Serial.print("Firmware Version: "); Serial.println(FIRMWARE_VERSION);
  Serial.print("Build Date: "); Serial.print(FIRMWARE_BUILD_DATE); 
  Serial.print(" "); Serial.println(FIRMWARE_BUILD_TIME);
  Serial.print("Hardware: "); Serial.println(HARDWARE_VERSION);
  Serial.print("Manufacturer: "); Serial.println(MANUFACTURER);
  Serial.print("ESP32 Chip: "); Serial.println(ESP.getChipModel());
  Serial.print("CPU Freq: "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.print("Flash Size: "); Serial.print(ESP.getFlashChipSize() / 1024 / 1024); Serial.println(" MB");
  Serial.print("Free Heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
  Serial.println(String('=', 60) + "\n");
  
  // Initialize log buffer
  clearLogBuffer();
  
  // Initialize watchdog timer
  InitWatchdog();
  add_log("System initializing...");
  
  WiFiConfig();
  
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    10000,
    NULL,
    1,
    &httpDebugLog,
    0);
    
  CANConfig();
  MotorConfig();
  ServoConfig();
  LoadDiverterConfig();
  
  Beep();
  DiverterTest();
  
  unsigned long testDelay = millis();
  while(millis() - testDelay < 2000) {
    FeedWatchdog();
    delay(10);
  }
  
  Serial.print("Bot ID: "); Serial.print(botIdBuffer); 
  Serial.print(" | Code: "); Serial.println(HARDWARE_VERSION);
  printFileName();
  
  add_log("Initialization complete - waiting for start button");
  Serial.println("Press panel button to start...");
  
  // Wait for button press
  while(mcp.digitalRead(PANEL_LED_BUTTON)) {
    FeedWatchdog();
    delay(10);
  }
  
  setTagetVelocity(S2);
  add_log("Bot started!");
  currentSystemState = STATE_NORMAL_OPERATION;
  
  // Initialize diagnostics
  sysDiag.lastHealthCheck = millis();
  sysDiag.lastMemoryCheck = millis();
  sysDiag.minFreeHeap = ESP.getFreeHeap();
}

void loop() {
  // Performance monitoring - start of loop
  unsigned long loopStart = millis();
  
  // Feed watchdog at the start of every loop iteration
  FeedWatchdog();
  
  // Process diagnostic commands from Serial
  processSerialCommands();
  
  // Check for emergency stop condition
  CheckEmergencyStop();
  
  // If in emergency stop state, don't process normal operations
  if (currentSystemState == STATE_EMERGENCY_STOP) {
    return;
  }
  
  // Periodic system health check
  if (millis() - sysDiag.lastHealthCheck > HEALTH_CHECK_INTERVAL) {
    updateSystemHealth();
    sysDiag.lastHealthCheck = millis();
    
    // Log health status periodically
    if (sysDiag.systemHealth < 80) {
      char msg[64];
      snprintf(msg, 64, "System Health: %d%% | Loop: %lums | Heap: %d", 
        sysDiag.systemHealth, sysDiag.maxLoopTime, sysDiag.currentFreeHeap);
      add_log(msg);
    }
  }
  
  // Handle error states with non-blocking state machine
  if (errorMode && (errorCode == 3 || errorCode == 4 || errorCode == 5)) {
    ErrorHandling();
    goto loop_end;  // Skip normal processing during error handling
  }
  
  ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

  String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

  if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    add_log("Current station: " + currentStation);
    if (CheckStationValidity(stationCodeMain)) {
      bool direction = GetDirectionFromStationId(currentStation);
      SetDiverter(direction);
      if (currentStation != "I1") {
        diverterCheckFlag = true;
      }
      if (!localisationFlag) {
        if (expectedStation != currentStation) {
          errorCode = 3;
          errorMode = true;
        }
      }
      else {
        localisationFlag = false;
      }

      if (!errorMode) {
        setTagetVelocity(GetSetSpeedAsPerStation(currentStation));
        expectedStation = GetExpectedStationFromStationId(currentStation);
      }
    } else {
      add_log("Invalid station code: " + stationCodeMain);
      errorCode = 4;
      errorMode = true;
    }
  }

  bool columnDetected = ColumnIndicatorDetection();

  CheckDiverter(columnDetected);
  
loop_end:
  // Performance monitoring - end of loop
  unsigned long loopTime = millis() - loopStart;
  sysDiag.loopCount++;
  sysDiag.loopTimeSum += loopTime;
  
  if (loopTime > sysDiag.maxLoopTime) {
    sysDiag.maxLoopTime = loopTime;
  }
  
  // Warn if loop time exceeds threshold
  if (loopTime > MAX_LOOP_TIME_MS) {
    char msg[48];
    snprintf(msg, 48, "WARN: Loop time exceeded: %lums", loopTime);
    add_log(msg);
    errorCode = ERR_LOOP_TIME_EXCEEDED;
  }
}

void printFileName() {
  Serial.println(__FILE__);
}

//=============================================================================
// DIAGNOSTIC AND DEBUG FUNCTIONS
//=============================================================================

// Print comprehensive system diagnostics
void printSystemDiagnostics() {
  Serial.println("\n" + String('=', 60));
  Serial.println("SYSTEM DIAGNOSTICS");
  Serial.println(String('=', 60));
  
  Serial.println("FIRMWARE:");
  Serial.print("  Version: "); Serial.println(FIRMWARE_VERSION);
  Serial.print("  Hardware: "); Serial.println(HARDWARE_VERSION);
  Serial.print("  Build: "); Serial.print(FIRMWARE_BUILD_DATE); 
  Serial.print(" "); Serial.println(FIRMWARE_BUILD_TIME);
  
  Serial.println("\nPERFORMANCE:");
  Serial.print("  Loop Count: "); Serial.println(sysDiag.loopCount);
  Serial.print("  Max Loop Time: "); Serial.print(sysDiag.maxLoopTime); Serial.println(" ms");
  Serial.print("  Avg Loop Time: "); Serial.print(sysDiag.avgLoopTime); Serial.println(" ms");
  
  Serial.println("\nMEMORY:");
  Serial.print("  Current Free Heap: "); Serial.print(sysDiag.currentFreeHeap); Serial.println(" bytes");
  Serial.print("  Min Free Heap: "); Serial.print(sysDiag.minFreeHeap); Serial.println(" bytes");
  Serial.print("  Heap Used: "); Serial.print(ESP.getHeapSize() - sysDiag.currentFreeHeap); Serial.println(" bytes");
  
  Serial.println("\nSENSORS:");
  for (int i = 0; i < 3; i++) {
    Serial.print("  Photodiode "); Serial.print(i+1); Serial.print(": ");
    Serial.print(sensorArray[i].isHealthy ? "HEALTHY" : "FAULTY");
    Serial.print(" | Failures: "); Serial.print(sysDiag.photodiodeFailCount[i]);
    Serial.print(" | Value: "); Serial.println(sensorArray[i].filteredValue);
  }
  
  Serial.println("\nCOMMUNICATION:");
  Serial.print("  WiFi: "); Serial.println(wifiConnected ? "Connected" : "Disconnected");
  Serial.print("  CAN Errors: "); Serial.println(sysDiag.canCommFailCount);
  Serial.print("  Servo Errors: "); Serial.println(sysDiag.servoCommFailCount);
  
  Serial.println("\nCONFIGURATION:");
  Serial.print("  Bot ID: "); Serial.println(botIdBuffer);
  Serial.print("  Current Station: "); Serial.println(currentStationBuffer);
  Serial.print("  Expected Station: "); Serial.println(expectedStationBuffer);
  Serial.print("  Front Servo: L="); Serial.print(front_diverter_left_thresold);
  Serial.print(" R="); Serial.println(front_diverter_right_thresold);
  Serial.print("  Rear Servo: L="); Serial.print(rear_diverter_left_thresold);
  Serial.print(" R="); Serial.println(rear_diverter_right_thresold);
  
  Serial.println("\nSYSTEM STATE:");
  Serial.print("  State: ");
  switch(currentSystemState) {
    case STATE_INITIALIZING: Serial.println("INITIALIZING"); break;
    case STATE_NORMAL_OPERATION: Serial.println("NORMAL"); break;
    case STATE_ERROR_WAITING_BUTTON: Serial.println("ERROR_WAIT"); break;
    case STATE_ERROR_RECOVERING: Serial.println("ERROR_RECOVER"); break;
    case STATE_EMERGENCY_STOP: Serial.println("EMERGENCY_STOP"); break;
    case STATE_WIFI_CONNECTING: Serial.println("WIFI_CONNECT"); break;
  }
  Serial.print("  Error Code: "); Serial.println(errorCode);
  Serial.print("  Error Mode: "); Serial.println(errorMode ? "YES" : "NO");
  Serial.print("  System Health: "); Serial.print(sysDiag.systemHealth); Serial.println("%");
  
  Serial.println(String('=', 60) + "\n");
}

// Process serial commands for diagnostics
void processSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "diag" || cmd == "status") {
      printSystemDiagnostics();
    }
    else if (cmd == "reset") {
      Serial.println("Resetting system...");
      delay(100);
      ESP.restart();
    }
    else if (cmd == "mem") {
      Serial.print("Free Heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
      Serial.print("Min Free Heap: "); Serial.print(sysDiag.minFreeHeap); Serial.println(" bytes");
    }
    else if (cmd == "sensors") {
      for (int i = 0; i < 3; i++) {
        Serial.print("Sensor "); Serial.print(i+1); Serial.print(": ");
        Serial.print(sensorArray[i].filteredValue); Serial.print(" (");
        Serial.print(sensorArray[i].isHealthy ? "OK" : "FAIL"); Serial.println(")");
      }
    }
    else if (cmd == "config") {
      Serial.println("Diverter Configuration:");
      Serial.printf("Front: L=%d R=%d Tol=%d\n", front_diverter_left_thresold, front_diverter_right_thresold, front_diverter_tolerance);
      Serial.printf("Rear: L=%d R=%d Tol=%d\n", rear_diverter_left_thresold, rear_diverter_right_thresold, rear_diverter_tolerance);
    }
    else if (cmd == "help") {
      Serial.println("\nAvailable Commands:");
      Serial.println("  diag/status - Print full diagnostics");
      Serial.println("  reset       - Reset system");
      Serial.println("  mem         - Memory status");
      Serial.println("  sensors     - Sensor readings");
      Serial.println("  config      - Configuration");
      Serial.println("  help        - This help message");
    }
    else if (cmd.length() > 0) {
      Serial.println("Unknown command. Type 'help' for commands.");
    }
  }
}

void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
  sm.IncreaseLimit(2, 5000);
}

// Function to load values from NVM
// Function to load and validate diverter configuration from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);
  
  // Check for magic number to validate config
  uint16_t magic = prefs.getUShort("magic", 0);
  bool configValid = (magic == CONFIG_MAGIC_NUMBER);
  
  // Load Bot ID
  String tempBotId = prefs.getString("BotId", "Bx");
  strncpy(botIdBuffer, tempBotId.c_str(), sizeof(botIdBuffer) - 1);
  botIdBuffer[sizeof(botIdBuffer) - 1] = '\0';

  // Load and validate front diverter settings
  front_diverter_left_limit = prefs.getInt("front_left_lim", 1994);
  front_diverter_right_limit = prefs.getInt("front_right_lim", 2525);
  front_diverter_tolerance = prefs.getInt("front_tol", 106);
  front_diverter_left_thresold = prefs.getInt("front_left_thr", 2100);
  front_diverter_right_thresold = prefs.getInt("front_right_thr", 2419);
  
  // Load and validate rear diverter settings
  rear_diverter_left_limit = prefs.getInt("rear_left_lim", 2936);
  rear_diverter_right_limit = prefs.getInt("rear_right_lim", 2387);
  rear_diverter_tolerance = prefs.getInt("rear_tol", 109);
  rear_diverter_left_thresold = prefs.getInt("rear_left_thr", 2827);
  rear_diverter_right_thresold = prefs.getInt("rear_right_thr", 2496);
  
  prefs.end();
  
  // Validate servo limits are within acceptable range
  bool limitsValid = true;
  if (front_diverter_left_limit < MIN_SERVO_LIMIT || front_diverter_left_limit > MAX_SERVO_LIMIT) limitsValid = false;
  if (front_diverter_right_limit < MIN_SERVO_LIMIT || front_diverter_right_limit > MAX_SERVO_LIMIT) limitsValid = false;
  if (rear_diverter_left_limit < MIN_SERVO_LIMIT || rear_diverter_left_limit > MAX_SERVO_LIMIT) limitsValid = false;
  if (rear_diverter_right_limit < MIN_SERVO_LIMIT || rear_diverter_right_limit > MAX_SERVO_LIMIT) limitsValid = false;
  
  if (!configValid) {
    Serial.println("WARN: Config magic number invalid - using defaults");
    appendToLogBuffer("WARN: Config validation failed");
  }
  
  if (!limitsValid) {
    Serial.println("ERROR: Servo limits out of range!");
    appendToLogBuffer("ERROR: Invalid servo limits in config");
    errorCode = ERR_CONFIG_INVALID;
    errorMode = true;
  }
  
  Serial.println("Diverter config loaded from NVM.");
  Serial.print("Bot ID: "); Serial.println(botIdBuffer);
  Serial.printf("Front: L=%d R=%d | Rear: L=%d R=%d\n", 
    front_diverter_left_thresold, front_diverter_right_thresold,
    rear_diverter_left_thresold, rear_diverter_right_thresold);
}

// Function to save configuration with magic number
void SaveDiverterConfig() {
  prefs.begin("diverter", false);
  prefs.putUShort("magic", CONFIG_MAGIC_NUMBER);
  prefs.putString("BotId", botIdBuffer);
  prefs.putInt("front_left_lim", front_diverter_left_limit);
  prefs.putInt("front_right_lim", front_diverter_right_limit);
  prefs.putInt("front_tol", front_diverter_tolerance);
  prefs.putInt("front_left_thr", front_diverter_left_thresold);
  prefs.putInt("front_right_thr", front_diverter_right_thresold);
  prefs.putInt("rear_left_lim", rear_diverter_left_limit);
  prefs.putInt("rear_right_lim", rear_diverter_right_limit);
  prefs.putInt("rear_tol", rear_diverter_tolerance);
  prefs.putInt("rear_left_thr", rear_diverter_left_thresold);
  prefs.putInt("rear_right_thr", rear_diverter_right_thresold);
  prefs.end();
  Serial.println("Diverter config saved to NVM with validation");
}

// Setup the MCP with error handling
void MCPConfig() {
  int retryCount = 0;
  const int MAX_MCP_RETRIES = 5;
  
  while (!mcp.begin_I2C() && retryCount < MAX_MCP_RETRIES) {
    Serial.println("MCP23X17 initialization failed, retrying...");
    delay(500);
    retryCount++;
  }
  
  if (retryCount >= MAX_MCP_RETRIES) {
    Serial.println("FATAL: MCP23X17 initialization failed after retries!");
    // Flash buzzer to indicate critical error
    for (int i = 0; i < 5; i++) {
      digitalWrite(BUZZER, HIGH);
      delay(100);
      digitalWrite(BUZZER, LOW);
      delay(100);
    }
    // System cannot continue without MCP - halt
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println("MCP23X17 initialized successfully");
}

// Logger Function which will run as RTOS task - Updated to use fixed buffer
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    FeedWatchdog();  // Feed watchdog from logger task too
    
    if (logBufferPos > strlen(botIdBuffer) + strlen(HARDWARE_VERSION) + 2) {  // Has content beyond prefix
      int httpCode = -1;
      if (loggerFlag && wifiConnected) {
        httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
        httpCode = httpDebugger.POST(logBuffer);
        httpDebugger.end();
      } else {
        httpCode = HTTP_CODE_OK;
      }
      
      if (httpCode == HTTP_CODE_OK) {
        clearLogBuffer();  // Clear buffer after successful send
      } else if (httpCode > 0) {
        char msg[32];
        snprintf(msg, 32, "HTTP error: %d", httpCode);
        Serial.println(msg);
      }
    }
  }
}

// Function to add logs - Updated to use fixed buffer
void add_log(const char* log) {
  appendToLogBuffer(log);
  Serial.println(log);  // Also print to serial for debugging
}

// Overload for compatibility with existing String calls
void add_log(String log) {
  add_log(log.c_str());
}

// Function to connect controller to network with timeout
void WiFiConfig() {
  currentSystemState = STATE_WIFI_CONNECTING;
  WiFi.begin(SSID, PASSWORD);
  wifiConnectStartTime = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiConnectStartTime > WIFI_CONNECT_TIMEOUT) {
      Serial.println("WiFi connection timeout - continuing without WiFi");
      add_log("WiFi connection failed - operating without logging");
      wifiConnected = false;
      loggerFlag = false;
      return;
    }
    FeedWatchdog();
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected to WiFi!");
  wifiConnected = true;
  loggerFlag = true;
}

// Setup the pin configuration
void PinConfig() {
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT);
  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);

  // GPIO command outputs to Secondary
  digitalWrite(EMG_PIN, LOW);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, LOW);
}

// Function to configure the CAN protocol with error checking
void CANConfig() {
  CAN0.setCANPins(GPIO_NUM_15, GPIO_NUM_14);  // rx tx
  
  int canInitResult = CAN0.begin(500000);
  if (canInitResult != 0) {
    Serial.println("CAN initialization failed!");
    add_log("WARN: CAN initialization failed - code: " + String(canInitResult));
    // Retry once
    delay(500);
    canInitResult = CAN0.begin(500000);
    if (canInitResult != 0) {
      Serial.println("CAN initialization failed after retry!");
      add_log("ERROR: CAN initialization failed after retry");
      // System may not be able to control motors - critical error
    } else {
      Serial.println("CAN Ready (after retry)!");
    }
  } else {
    Serial.println("CAN Ready!");
  }
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

// Function to make hardware beep for 250 ms
void Beep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

/*******************************************************************************
 * PRODUCTION DEPLOYMENT GUIDE
 ******************************************************************************/

/*
===============================================================================
DEPLOYMENT CHECKLIST:
===============================================================================

PRE-DEPLOYMENT:
[ ] Verify all hardware connections (CAN, servos, sensors, emergency stop)
[ ] Test emergency stop functionality
[ ] Calibrate servo positions and save to NVM using SaveDiverterConfig()
[ ] Verify station code mappings match physical layout
[ ] Test all station transitions
[ ] Verify WiFi credentials or disable logging if not needed
[ ] Check free heap is > 30KB after initialization
[ ] Run system for 30+ minutes to verify watchdog and stability
[ ] Document bot ID and assign unique identifier

TESTING PROTOCOL:
1. Power on and verify startup sequence
2. Check serial output for firmware version and diagnostics
3. Test emergency stop (buzzer should sound immediately)
4. Test diverter movement (left and right)
5. Run through complete station sequence
6. Verify error recovery procedures
7. Monitor loop time (should be < 100ms)
8. Check sensor health status via "diag" command

POST-DEPLOYMENT MONITORING:
- Monitor system health percentage (should stay > 80%)
- Check for sensor failures in diagnostics
- Verify loop times remain acceptable
- Monitor free heap memory
- Review error logs regularly

===============================================================================
SERIAL DIAGNOSTIC COMMANDS:
===============================================================================
Type these commands in Serial Monitor (115200 baud):

  diag     - Full system diagnostics (recommended for troubleshooting)
  status   - Same as diag
  reset    - Soft reset the system
  mem      - Memory usage information
  sensors  - Current sensor readings
  config   - Diverter configuration
  help     - List all commands

===============================================================================
ERROR CODES REFERENCE:
===============================================================================
  0  - ERR_NONE - No error
  1  - ERR_WIFI_CONNECTION_FAILED - WiFi timeout (system continues without WiFi)
  2  - ERR_MCP_INIT_FAILED - Fatal: MCP23017 not responding
  3  - ERR_UNEXPECTED_STATION - Detected station doesn't match expected
  4  - ERR_INVALID_STATION_CODE - Station code not in mapping table
  5  - ERR_SERVO_COMMUNICATION_FAILED - Servo not responding
  6  - ERR_CAN_INIT_FAILED - CAN bus initialization failed
  7  - ERR_WATCHDOG_RESET - System was reset by watchdog
  8  - ERR_SENSOR_FAILURE - Photo diode sensor failure
  9  - ERR_MEMORY_LOW - Free heap below 20KB
  10 - ERR_CONFIG_INVALID - Configuration validation failed
  11 - ERR_MOTOR_COMMUNICATION - Motor controller communication error
  12 - ERR_DIVERTER_POSITION_INVALID - Servo position out of expected range
  13 - ERR_LOOP_TIME_EXCEEDED - Main loop taking > 100ms
  14 - ERR_EMERGENCY_STOP - Emergency stop activated

===============================================================================
RECOVERY PROCEDURES:
===============================================================================

ERR_UNEXPECTED_STATION (3):
- System halts and waits for button press (60s timeout)
- Press panel button to attempt re-localization
- Bot will read next station and recover if valid

ERR_INVALID_STATION_CODE (4):
- Check if station barcode is damaged or dirty
- Verify station code is in STATION_CODE_TO_STATION_ID_MAP
- Press button to attempt recovery

ERR_SERVO_COMMUNICATION_FAILED (5):
- Check servo power and serial connections
- Verify servo IDs are correct (FRONT_SERVO=2, REAR_SERVO=3)
- Press button to retry communication

EMERGENCY_STOP:
- System halts all operations immediately
- Buzzer sounds continuously
- Release emergency stop button
- System requires manual reset/restart

===============================================================================
CONFIGURATION:
===============================================================================

WiFi Settings (lines ~173):
  SSID: "Server_PC"
  PASSWORD: "msort@flexli"
  SERVER_URL: "http://192.168.2.109:5000/data"

Sensor Thresholds (lines ~177):
  PT_UPPER_THR_LIMIT: 600  (rising threshold)
  PT_LOWER_THR_LIMIT: 400  (falling threshold)
  (Hysteresis prevents noise-induced toggling)

Watchdog Timeout (line ~135):
  WDT_TIMEOUT: 10 seconds
  (System resets if loop hangs > 10s)

Loop Performance (line ~145):
  MAX_LOOP_TIME_MS: 100ms
  (Warning logged if exceeded)

Servo Calibration:
  Stored in NVM under "diverter" namespace
  Use SaveDiverterConfig() to persist changes
  CONFIG_MAGIC_NUMBER validates stored data

===============================================================================
MAINTENANCE:
===============================================================================

WEEKLY:
- Review error logs
- Check system health metrics
- Verify sensor readings are within expected range
- Test emergency stop

MONTHLY:
- Clean photo diode sensors
- Verify servo positions and recalibrate if needed
- Check CAN bus termination resistors
- Update firmware if new version available

AS NEEDED:
- Recalibrate servos if diverter performance degrades
- Update station mappings if layout changes
- Adjust speed settings per station

===============================================================================
KNOWN LIMITATIONS:
===============================================================================
1. WiFi logging uses String concatenation in HTTP_DEBUG_LOGGER task
2. Some error recovery still uses blocking delays (being refactored)
3. Station reading requires clean, high-contrast barcode visibility
4. CAN bus assumes single motor controller on network
5. System requires manual restart after emergency stop

===============================================================================
FUTURE IMPROVEMENTS (Roadmap):
===============================================================================
- OTA firmware update capability
- MQTT telemetry for real-time monitoring
- Advanced predictive maintenance (bearing vibration, motor current)
- Multi-bot coordination
- Automatic station mapping discovery
- Battery voltage monitoring and low-power modes
- Data logging to SD card for offline analysis

===============================================================================
SUPPORT:
===============================================================================
For technical support, provide:
1. Firmware version (from startup or 'diag' command)
2. Bot ID
3. Error codes and error messages
4. System diagnostics output ('diag' command)
5. Description of issue and steps to reproduce

===============================================================================
*/