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
#include <SMS.h>
#include <unordered_map>
#include <string>
#include <Preferences.h>
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
#define BUZZER 13
#define EMG_PIN 12
#define PANEL_LED_PIN 8
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35
#define BOT_INDICATOR_SENSOR 19
#define PANEL_LED_BUTTON 7 
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define ENABLE 1
#define DISABLE 0
#define SERVO_UART_DELAY 50

// Constant variables
const String CODE_ID = "d171";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const int THRESHOLD_INTENSITY = 250;
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

// Logger variables
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Bot Variables
Adafruit_MCP23X17 mcp;
bool slowDownFlag = false;
bool stopBotFlag = false;
bool _photoTransistorCurrentStateArray[3] = { 0, 0, 0 };       // Array to store current state of sensors, either 0 or 1 based on threshold during station reading
bool _photoTransistorPreviousStateArray[3] = { 0, 0, 0 };      // Array to store previous state of sensors, mainily to check if state has changed or not.

uint16_t _photoTransistorCurrentValueArray[3] = { 0, 0, 0 };   // Array to store the analog value sent by the sensor currently.
uint16_t _photoTransistorPreviousValueArray[3] = { 0, 0, 0 };  // Array to store the analog value sent by the sensor previously.

String _intermediateColumnCode = "000";                        // String to store intermediate column codes in String format, mainly to detect when bot is changing columns.
bool _readingInProcess = false;                                // Flag variable to keep track wether station reading has started and is in progress.
String _finalColumnCodeArray[3] = { "000", "000", "000" };     // Array used to store the 3 columns of the station in string format.
bool _rightStationDetected = false;                            // Flag variable to check if a station has been detected so that it can perform its next set of operations on the obtained data.
String _previousIntermediateColumnCode = "000";
String stationCode = "";      // String variable to store entire stationCode in string format
String currentStation = "X";  // String variable to store the station Id, after converting the station code read into stationId

String StationsArray[25] = { "000000000" };
unordered_map<string, string> STATION_CODE_TO_STATION_ID_MAP = {
  { "100011110", "D1" }, { "011110110", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, { "110110010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010011", "D8" }, { "100100001", "D9" }, { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100100110", "D13" }, { "100101100", "D14" }, { "100100101", "D15" }, { "100100011", "D16" }, { "100100100", "D17" }, { "100100010", "D18" }, { "011011100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }
};

unordered_map<string, bool> STATION_ID_TO_DIRECTION_MAP = {
  { "D1", 0 }, { "D2", 0 }, { "D3", 1 }, { "D4", 1 }, { "D5", 1 }, { "D6", 1 }, { "D7", 1 }, { "D8", 1 }, { "D9", 1 }, { "D10", 1 }, { "D11", 1 }, { "D12", 1 }, { "D13", 1 }, { "D14", 1 }, { "D15", 1 }, { "D16", 1 }, { "D17", 1 }, { "D18", 1 }, { "I1", 1 }, { "I11", 0 }, { "I12", 1 }
};

SMS sm;
bool rightDivertor = true;
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

// Handler for the case when TI is OFF
void HandleTrafficIndicatorDetectionOFF() {
  // Case: If condition true then the bot is in decelerated mode
  if (slowDownFlag) {
    // Set the bot to the SET_SPEED
    setTagetVelocity(S0_5);
    stopBotFlag = false;
    slowDownFlag = false;
  }
}

// Handler for the case when TI is ON
void HandleTrafficIndicatorDetectionON() {
  // Case: If condition true then the bot is in accelerated mode else bot is in decelerated mode
  if (!slowDownFlag) {
    // Set the bot to the MINIMAL_SPEED
    setTagetVelocity(S0_05);
    slowDownFlag = true;
  } else {
    // Case: If condition true then bot has achieved MINIMAL_SPED
    if (getVelocityActualValueAveraged() < 30) {
      // Case: If condition true then bot has not already stopped
      if (!stopBotFlag) {
        // Stop the bot
        haltMovement();
        stopBotFlag = true;
      }
    }
  }
}

// Function which tells if traffic indicator is detected or not
bool TrafficIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int TI_rawValue = map(analogRead(TRAFFIC_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then traffic detected else not detected
  if (TI_rawValue < THRESHOLD_INTENSITY) {
    return true;
  }
  return false;
}

// Function which tells if column indicator is detected or not
bool ColumnIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int CI_rawValue = map(analogRead(COLUMN_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then column detected else not detected
  if (CI_rawValue < THRESHOLD_INTENSITY) {
    return true;
  }
  return false;
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

String GetStationCodeFromDataReadByStationSensor() {
  String instColumnCode = String(_photoTransistorCurrentStateArray[0]) + String(_photoTransistorCurrentStateArray[1]) + String(_photoTransistorCurrentStateArray[2]);
  if (instColumnCode == "111") {
    _intermediateColumnCode = instColumnCode;
  }

  // START STATION READING
  if (!_readingInProcess && instColumnCode == "111") {
    _readingInProcess = true;
  }

  // STATION READING COMPLETED
  String stationCode;
  if (instColumnCode == "000" && _readingInProcess) {
    if (_finalColumnCodeArray[2] != "000" && _finalColumnCodeArray[1] != "000" && _finalColumnCodeArray[0] != "000") {
      stationCode = _finalColumnCodeArray[2] + _finalColumnCodeArray[1] + _finalColumnCodeArray[0];
      _rightStationDetected = true;
      _readingInProcess = false;
    }
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
    } else if (currentStateSum == prevStateSum) {
      // Nothing
    }

    if (_previousIntermediateColumnCode != "000" && _previousIntermediateColumnCode != "111" && _intermediateColumnCode == "111" && _intermediateColumnCode != "000") {
      _finalColumnCodeArray[2] = _finalColumnCodeArray[1];
      _finalColumnCodeArray[1] = _finalColumnCodeArray[0];
      _finalColumnCodeArray[0] = _previousIntermediateColumnCode;
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
  current_position_front_servo = sm.ReadPos(FRONT_SERVO);
  delay(1);
  current_position_rear_servo = sm.ReadPos(REAR_SERVO);
  delay(1);
  if (current_position_front_servo < front_diverter_left_thresold) {
    sm.EnableTorque(FRONT_SERVO, DISABLE);
    delay(1);
  } else {
    sm.WritePosEx(FRONT_SERVO, front_diverter_left_thresold, 100);
    delay(1);
  }
  
  if (current_position_rear_servo > rear_diverter_left_thresold) {
    sm.EnableTorque(REAR_SERVO, DISABLE);
    delay(1);
  } else {
    sm.WritePosEx(REAR_SERVO, rear_diverter_left_thresold, 100);
    delay(1);
  }
}

void DivertRight() {
  current_position_front_servo = sm.ReadPos(FRONT_SERVO);
  delay(1);
  current_position_rear_servo = sm.ReadPos(REAR_SERVO);
  delay(1);
  if (current_position_front_servo > front_diverter_right_thresold) {
    sm.EnableTorque(FRONT_SERVO, DISABLE);
     delay(1);
  } else {
    sm.WritePosEx(FRONT_SERVO, front_diverter_right_thresold, 100);
     delay(1);
  }
  
  if (current_position_rear_servo < rear_diverter_right_thresold) {
    sm.EnableTorque(REAR_SERVO, DISABLE);
    delay(1);
  } else {
    sm.WritePosEx(REAR_SERVO, rear_diverter_right_thresold, 100);
    delay(1);
  }
}

void DiverterTest() {
  DivertLeft();
  delay(1000);
  DivertRight();
  delay(1000);
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

void setup() {
  delay(1000);
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
  MCPConfig();
  CANConfig();
  PinConfig();
  MotorConfig();
  ServoConfig();
  LoadDiverterConfig();
  Beep();
  DiverterTest();
  delay(1000);
  setTagetVelocity(S2);
  delay(500);
}

void loop() {
  unsigned long loopStartTime = micros();
  // unsigned long latency = micros();

  bool columnDetected = ColumnIndicatorDetection();
  if (columnDetected) {
    // Serial.println("CI: " + String(columnDetected));
    mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    bool trafficDetected = TrafficIndicatorDetection();

    if (trafficDetected) {
      // add_log("Traffic Detected!");
      HandleTrafficIndicatorDetectionON();
    } else {
      // add_log("Traffic NOT Detected!");
      HandleTrafficIndicatorDetectionOFF();
    }
  } else {
    mcp.digitalWrite(PANEL_LED_PIN, LOW);
    if (stopBotFlag && getVelocityActualValueAveraged() == 0) {
      setTagetVelocity(S0_05);
      stopBotFlag = false;
    }
  }

  // if (maxLatency < micros() - latency) {
  //   maxLatency = micros() - latency;
  //   add_log("New. WTM max. time: " + String(maxLatency));
  // }

  // unsigned long latency = micros();

  ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

  String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

  // if (maxLatency < micros() - latency) {
  //   maxLatency = micros() - latency;
  //   add_log("New. station reading max. time: " + String(maxLatency));
  // }
  
  if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    if (CheckStationValidity(stationCodeMain)) {
      bool direction = GetDirectionFromStationId(currentStation);
      // unsigned long latency = micros();
      SetDiverter(direction);
      // if (maxLatency < micros() - latency) {
      //   maxLatency = micros() - latency;
      //   add_log("New. diversion max. time: " + String(maxLatency));
      // }
      if (currentStation == "D1") {
        setTagetVelocity(S1);
      }

      if (currentStation == "D2") {
        setTagetVelocity(S2);
      }

      if (currentStation == "I11") {
        setTagetVelocity(S1);
      }

      if (currentStation == "I1") {
        setTagetVelocity(S0_5);
      }
    }
  }

  PerformanceMonitor();
}

void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
  sm.IncreaseLimit(2, 5000);
}

// Function to load values from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);
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

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (debugLoggingString != "B1 " + CODE_ID + ": ") {
      httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
      int httpCode = -1;
      if (loggerFlag) {
        // Serial.println("Sending log!");
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
  debugLoggingString += "\n" + log;
}

// Function to montior the loop time and memory usage of the controller
void PerformanceMonitor() {
  // RAM monitoring
  size_t freeHeap = ESP.getFreeHeap();   // Free heap memory
  size_t totalHeap = ESP.getHeapSize();  // Total heap memory
  float ramUsage = 100.0 - ((float)freeHeap / totalHeap * 100.0);
  if (ramUsage > 70) {
    add_log("Ram usage has exceeded 70%!");
    add_log("Total ram available (b): " + String(totalHeap));
    add_log("RAM used (b): " + String(totalHeap - freeHeap));
    add_log("Free space left (b): " + String(freeHeap));
    add_log("RAM used (%): " + String(ramUsage) + " %");
  }

  // Loop time monitoring
  unsigned long loopTime = micros() - loopStartTime;
  if (numsOfIteration == 10000) {
    // add_log("Core 1 Loop Time (micros): " + String(loopTime));
    // add_log("RAM used (b): " + String(totalHeap - freeHeap));
    numsOfIteration = 0;
  }
  if (loopTime > maxLoopTime) {
    maxLoopTime = loopTime;
    add_log("Core 1 New Max. Loop Time (micros): " + String(maxLoopTime));
  }
  if (totalHeap - freeHeap > maxMemoryUsed) {
    maxMemoryUsed = totalHeap - freeHeap;
    // add_log("New Max. Memory Usage (b): " + String(totalHeap - freeHeap));
  }
  numsOfIteration++;
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
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT);       // Traffic indicator pin config
  pinMode(BOT_INDICATOR_SENSOR, OUTPUT);         // Bot detection sensor pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT);      // Column indicator pin config
  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);

  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(BOT_INDICATOR_SENSOR, LOW);
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
  setProfileAcceleration(2000);
  setProfileDeceleration(2000);
  setQuickStopDeceleration(2000);
  setMotionProfileType(1);
  disable();
  enable();
}