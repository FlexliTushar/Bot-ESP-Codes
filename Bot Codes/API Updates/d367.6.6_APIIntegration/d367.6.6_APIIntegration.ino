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
#include <can_common.h>
#include <object_dictionary.h>
#include <maxon.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SMS.h>
#include <unordered_map>
#include <string>
#include <Preferences.h>
#include <Adafruit_MCP23X17.h>
using namespace std;

// Motor Speed 
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
#define EMG_PIN 12
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35
#define PANEL_LED_BUTTON 7
#define PANEL_LED_PIN 8
#define AFC_PIN_1 18
#define AFC_PIN_2 19
// #define BUMPER_SENSOR 34

#define ENABLE 1
#define DISABLE 0
#define SERVO_UART_DELAY 50
#define UART_BAUD 115200
#define RX_PIN 16
#define TX_PIN 17

// Constant variables
const String CODE_ID = "d367.6.6-APIIntegration";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const static int SR_D1 = 36;  // ESP32 pin attached to Photo diode 1
const static int SR_D2 = 39;  // ESP32 pin attached to Photo diode 2
const static int SR_D3 = 34;  // ESP32 pin attached to Photo diode 3
const static uint16_t PT_UPPER_THR_LIMIT = 600;  // Threshold limit for rising photo diode reading.
const static uint16_t PT_LOWER_THR_LIMIT = 400;  // Threshold limit for falling photo diode reading
const static int MAX_REATTEMPTS_FOR_SERVO_COM = 3;
const int LOWER_THRESHOLD_INTENSITY = 200;
const int UPPER_THRESHOLD_INTENSITY = 400;
const unsigned long THRESHOLD_TIME_TO_CLOSE_FLAPS = 500;
unsigned long THRESHOLD_TIME_TO_DEFAULT_STATE_OF_AFC = 1000;
const int LED_INDEX = 14;
const int DIVERTER_POSITION_ERROR_LIMIT = 30;
const String DMS_URL_PREFIX = "http://192.168.2.109:8443/m-sort-distribution-server/";
const float ACTUAL_ACCELERATION = 1.515;
const unsigned int MIN_THRESHOLD_TIME_FOR_ON_COLUMN = 3;
HTTPClient _http;

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Bot Variables
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
String currentStation = "X";

unordered_map<string, string> STATION_CODE_TO_STATION_ID_MAP = {
  { "100011110", "D1" }, { "101110011", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, { "101011010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010011", "D8" }, { "100010101", "D9" }, { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100110100", "D13" }, { "100101100", "D14" }, { "100101110", "D15" }, { "100110001", "D16" }, { "100110010", "D17" }, { "100110011", "D18" }, { "110010100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }
};

unordered_map<string, bool> STATION_ID_TO_DIRECTION_MAP = {
  { "D1", 0 }, { "D2", 0 }, { "D3", 1 }, { "D4", 1 }, { "D5", 1 }, { "D6", 1 }, { "D7", 1 }, { "D8", 1 }, { "D9", 1 }, { "D10", 1 }, { "D11", 1 }, { "D12", 1 }, { "D13", 1 }, { "D14", 1 }, { "D15", 1 }, { "D16", 1 }, { "D17", 1 }, { "D18", 1 }, { "I1", 1 }, { "I11", 0 }, { "I12", 1 }
};

unordered_map<string, string> STATION_ID_TO_EXPECTED_STATION_ID_MAP = {
  { "D1", "I1" }, { "D2", "D3" }, { "D3", "D4" }, { "D4", "D5" }, { "D5", "D6" }, { "D6", "D7" }, { "D7", "D8" }, { "D8", "D9" }, { "D9", "D10" }, { "D10", "D11" }, { "D11", "D12" }, { "D12", "D13" }, { "D13", "D14" }, { "D14", "D15" }, { "D15", "D16" }, { "D16", "D17" }, { "D17", "D18" }, { "D18", "D1" }, { "I1", "I12" }, { "I11", "D2" }, { "I12", "I11" }
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

// Infeed stop variables
volatile bool errorMode = false;
int errorCode = -1;
int _intensitySumArray[3][3] = { 0 };
int _frameCount[3] = { 0 };
int _intermediateColumnFramesCount = 0;
int _intermediateIntensitySumArray[3] = { 0 };
uint16_t _warningMatrixData[3][3][3] = { 0 };
uint16_t _intermColumnPos1Count[3] = { 0 };
uint16_t _intermColumn0Count[3] = { 0 };
uint16_t _intermColumnNeg1Count[3] = { 0 };
unsigned long stationReadStartTime;
int setSpeed;
bool columnDetected = false;
bool trafficDetected = false;
SemaphoreHandle_t xStationCodeSemaphore;
TaskHandle_t stationDetectionOps;
String detectedStationCode = "";
String detectedStationCodeCore0 = "";
String prevDetectedStationCodeCore0 = "";
SemaphoreHandle_t xWTMDetectionSemaphore;
TaskHandle_t WTMDetectionOps;
Adafruit_MCP23X17 mcp;
String expectedStation = "X";
bool localisationFlag = true;
bool afcPinState = false;
unsigned long afcStartTime;
bool closingFlap = false;
unsigned long startClosingTime;
bool newDropOff = false;
bool columnDetectedForCore0 = false;
bool trafficDetectedForCore0 = false;
portMUX_TYPE xWTMMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xStartColumnCounterMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xColumnCountCompleteMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xErrorMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xStartColumnCounterAPIMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xColumnCountCompleteAPIMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool WTMInAction = false;
bool slowDownFlag = false;
bool stopBotFlag = false;
volatile bool startColumnCounter = false;
volatile int columnCounter = 0;
bool previousCIState = 0;
bool firstTimeTrafficDetected = false;
bool diverterCheckFlag = false;
bool previousCIStateDiversionCheck = 0;
int columnCounterDiversioncheck = 0;
volatile bool startInfeedAPIColumnCounter = false;
volatile int infeedAPIcolumnCounter = 0;
String dropoffStation = "X";
volatile bool getDestinationAPIFlag = false;
portMUX_TYPE xGetDestinationAPIMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t xDropoffStationMutex;
bool clearInfeedAPIFlag = false;
String previousDroppingStation = "X";
String droppingStationAPITask = "X";
bool updateParcelDroppedInTheDumpAPIFlag = false;
bool updateParcelDroppedAPIFlag = false;
bool previousCIStateForInfeedAPI = false;
bool onColumn = false;
int columnCounterForDetection = 0;
int columnNo = 0;
bool onTraffic = false;
unsigned long previousOnColumnTime;
float setSpeedInMPS;
unsigned long slowDownCommandStartTime = millis();
unsigned long stopBotFlagStartTime = millis();
int onTrafficCounter = 0;
bool onColumnForCounting = false;
unsigned long previousOnColumnTimeForCounting;
bool onTrafficForCounting = false;
unsigned long previousOnTrafficTimeForCounting;
int trafficNo = 0;
int trafficCounterForDetection = 0;
int prevErrorCode = -1;
TaskHandle_t apiTask;
unsigned long beepStartTime;
bool beepOn = false;

// Utility function to initialise station reader variables
void InitialiseAllVariables() {
  _intermediateColumnCode = "000";
  _previousIntermediateColumnCode = "000";
  for (int i = 0; i < 3; i++) {
    _finalColumnCodeArray[i] = "000";
    _frameCount[i] = 0;
  }
}

// Function to read the value across all photo diodes
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

// Function to convert photodiode data into a complete station code
String GetStationCodeFromDataReadByStationSensor() {
  String instColumnCode = String(_photoTransistorCurrentStateArray[0]) + String(_photoTransistorCurrentStateArray[1]) + String(_photoTransistorCurrentStateArray[2]);
  if (instColumnCode == "111") {
    _intermediateColumnCode = instColumnCode;
  }

  // START STATION READING
  if (!_readingInProcess && instColumnCode == "111") {
    _readingInProcess = true;
    stationReadStartTime = millis();
  }

  // STATION READING COMPLETED
  String stationCode;
  if (instColumnCode == "000" && _readingInProcess) {
    if (_finalColumnCodeArray[2] != "000" && _finalColumnCodeArray[1] != "000" && _finalColumnCodeArray[0] != "000") {
      stationCode = _finalColumnCodeArray[2] + _finalColumnCodeArray[1] + _finalColumnCodeArray[0];
      _rightStationDetected = true;
      _readingInProcess = false;
      add_log("Time between start and stop: " + String(millis() - stationReadStartTime));
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
      _intermediateColumnFramesCount = 1;
      for (int i = 0; i < 3; i++) {
        _intermediateIntensitySumArray[i] = _photoTransistorCurrentValueArray[i];
      }
    } else if (currentStateSum == prevStateSum) {
      if (currentStateSum > 0 && currentStateSum < 3) {
        _intermediateColumnFramesCount++;
        for (int i = 0; i < 3; i++) {
          _intermediateIntensitySumArray[i] += _photoTransistorCurrentValueArray[i];
        }
      }
    }

    if (_previousIntermediateColumnCode != "000" && _previousIntermediateColumnCode != "111" && _intermediateColumnCode == "111" && _intermediateColumnCode != "000") {
      _finalColumnCodeArray[2] = _finalColumnCodeArray[1];
      _finalColumnCodeArray[1] = _finalColumnCodeArray[0];
      _finalColumnCodeArray[0] = _previousIntermediateColumnCode;

      _frameCount[2] = _frameCount[1];
      _frameCount[1] = _frameCount[0];
      _frameCount[0] = _intermediateColumnFramesCount;

      for (int i = 2; i > 0; i--) {
        for (int j = 0; j < 3; j++) {
          _intensitySumArray[i][j] = _intensitySumArray[i - 1][j];
        }
      }

      for(int i = 0; i < 3; i++) {
        _intensitySumArray[0][i] = _intermediateIntensitySumArray[i];
      }
    }
    _previousIntermediateColumnCode = _intermediateColumnCode;
  }

  String stationCodeMain = "";
  if (_rightStationDetected) {
    _rightStationDetected = false;
    stationCodeMain = stationCode;
    add_log("Frame Count: {" + String(_frameCount[2]) + ", " + String(_frameCount[1]) + ", " + String(_frameCount[0]) + "}");
    add_log("Average intensity column 1: {" + String(_intensitySumArray[2][0]/_frameCount[2]) + ", " + String(_intensitySumArray[2][1]/_frameCount[2]) + ", " + String(_intensitySumArray[2][2]/_frameCount[2]) + "}");
    add_log("Average intensity column 2: {" + String(_intensitySumArray[1][0]/_frameCount[1]) + ", " + String(_intensitySumArray[1][1]/_frameCount[1]) + ", " + String(_intensitySumArray[1][2]/_frameCount[1]) + "}");
    add_log("Average intensity column 3: {" + String(_intensitySumArray[0][0]/_frameCount[0]) + ", " + String(_intensitySumArray[0][1]/_frameCount[0]) + ", " + String(_intensitySumArray[0][2]/_frameCount[0]) + "}");

    InitialiseAllVariables();
  }
  return stationCodeMain;
}

// Function to transform sttaion code into interpretable station Id
String TransformStationCodeIntoStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_CODE_TO_STATION_ID_MAP.find(stationCodeString) != STATION_CODE_TO_STATION_ID_MAP.end()) {
    string stationID = STATION_CODE_TO_STATION_ID_MAP.at(stationCodeString);
    return String(stationID.c_str());
  }
  return "";
}

// Function to check whether the station code is valid or not
bool CheckStationValidity(const String &stationCode) {
  // Check if the station-code is present in the keys of STATION_CODE_TO_STATION_ID hash-map
  string stationCodeString = string(stationCode.c_str());
  return STATION_CODE_TO_STATION_ID_MAP.find(stationCodeString) != STATION_CODE_TO_STATION_ID_MAP.end();
}

// Function to get the direction of diverter for the given stationId
bool GetDirectionFromStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_ID_TO_DIRECTION_MAP.find(stationCodeString) != STATION_ID_TO_DIRECTION_MAP.end()) {
    bool direction = STATION_ID_TO_DIRECTION_MAP.at(stationCodeString);
    return direction;
  }
  return 1;
}

// Diverter function - For Left
void DivertLeft() {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    taskENTER_CRITICAL(&xErrorMux);
    errorMode = true;
    taskEXIT_CRITICAL(&xErrorMux);
    errorCode = 5;
    return;
  }
  delay(1);
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    return;
  }
  delay(1);
  if (current_position_front_servo < front_diverter_left_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_left_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(10);
  }
  
  if (current_position_rear_servo > rear_diverter_left_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(REAR_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_left_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(1);
  }
}

// Diverter function - For Right
void DivertRight() {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    taskENTER_CRITICAL(&xErrorMux);
    errorMode = true;
    taskEXIT_CRITICAL(&xErrorMux);
    errorCode = 5;
    return;
  }
  delay(1);
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    taskENTER_CRITICAL(&xErrorMux);
    errorMode = true;
    taskEXIT_CRITICAL(&xErrorMux);
    errorCode = 5;
    return;
  }
  delay(1);
  if (current_position_front_servo > front_diverter_right_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_right_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(10);
  }
  
  if (current_position_rear_servo < rear_diverter_right_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(REAR_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_right_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      taskENTER_CRITICAL(&xErrorMux);
      errorMode = true;
      taskEXIT_CRITICAL(&xErrorMux);
      errorCode = 5;
      return;
    }
    delay(1);
  }
}

// Diverter function - Initialization test
void DiverterTest() {
  DivertLeft();
  delay(1000);
  DivertRight();
  delay(1000);
}

// Diverter function - Setting the direction
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

// Function to compute a tentative time after which we can check the bot's speed
unsigned long ComputeWaitTime() {
  return ((setSpeedInMPS - 0.05)/ACTUAL_ACCELERATION)*1500;
}

// // Handler for the case when TI is OFF
// void HandleTrafficIndicatorDetectionOFF() {
//   // Case: If condition true then the bot is in decelerated mode
//   if (slowDownFlag) {
//     // Set the bot to the SET_SPEED
//     setTagetVelocity(setSpeed);
//     stopBotFlag = false;
//     if (setSpeed != S0_05) slowDownFlag = false;
//   }
// }

// // Handler for the case when TI is ON
// void HandleTrafficIndicatorDetectionON() {
//   // Case: If condition true then the bot is in accelerated mode else bot is in decelerated mode
//   if (!slowDownFlag) {
//     // Set the bot to the MINIMAL_SPEED
//     setTagetVelocity(S0_05);
//     slowDownFlag = true;
//   } else {
//     // Case: If condition true then bot has achieved MINIMAL_SPEED
//     if (getVelocityActualValueAveraged() < 30) {
//       // Case: If condition true then bot has not already stopped
//       if (!stopBotFlag) {
//         // Stop the bot
//         haltMovement();
//         stopBotFlag = true;
//       }
//     }
//   }
// }

// Handler for the case when TI is OFF
void HandleTrafficIndicatorDetectionOFF() {
  // Case: If condition true then the bot is in decelerated mode
  if (slowDownFlag) {
    // Set the bot to the SET_SPEED
    setTagetVelocity(setSpeed);
    stopBotFlag = false;
    if (setSpeed != S0_03) slowDownFlag = false;
  }
}

// Handler for the case when TI is ON
void HandleTrafficIndicatorDetectionON() {
  // Case: If condition true then the bot is in accelerated mode else bot is in decelerated mode
  if (!slowDownFlag) {
    // Set the bot to the MINIMAL_SPEED
    add_log("Slowing doen to 0.03m/s");
    setTagetVelocity(S0_03);
    slowDownFlag = true;
    slowDownCommandStartTime = millis();
  } else {
    // if (millis() - slowDownCommandStartTime >= 1930 && !stopBotFlag) {
    if (!stopBotFlag) {
      // Case: If condition true then bot has achieved MINIMAL_SPEED
      int currentVelocity = getVelocityActualValueAveraged();
      add_log("Velocity: " + String(currentVelocity));
      if (currentVelocity < 30) {
        // Case: If condition true then bot has not already stopped
        if (!stopBotFlag) {
          // Stop the bot
          add_log("Stopping the bot!");
          haltMovement();
          stopBotFlag = true;
          stopBotFlagStartTime = millis();
        }
      }
    }
  }
}

// Function which tells if traffic indicator is detected or not
bool TrafficIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int TI_rawValue = map(analogRead(TRAFFIC_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then traffic detected else not detected
  if (TI_rawValue < LOWER_THRESHOLD_INTENSITY) return true;
  else if (TI_rawValue > LOWER_THRESHOLD_INTENSITY && TI_rawValue < UPPER_THRESHOLD_INTENSITY) return onTrafficForCounting;
  else if (TI_rawValue > UPPER_THRESHOLD_INTENSITY) return false;
  return false;
}

// Function which tells if column indicator is detected or not
bool ColumnIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int CI_rawValue = map(analogRead(COLUMN_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then column detected else not detected
  if (CI_rawValue < LOWER_THRESHOLD_INTENSITY) return true;
  else if (CI_rawValue > LOWER_THRESHOLD_INTENSITY && CI_rawValue < UPPER_THRESHOLD_INTENSITY) return onColumnForCounting;
  else if (CI_rawValue > UPPER_THRESHOLD_INTENSITY) return false;
  return false;
}

// Function which will execute when the bot goes in error mode
void ErrorHandling(int errorCode) {
  if (dropoffStation != "X") {
    dropoffStation = "D18";
  }
  if (errorCode == 3) {
    if (prevErrorCode == -1) {
      add_log("ER3: UNEXPECTED_STATION_ERROR");
      add_log("Expected Station: " + expectedStation + " Detected Station: " + currentStation);
    }
    if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
      currentStation = "X";
      expectedStation = "X";
      localisationFlag = true;
      setTagetVelocity(S0_5);
      errorMode = false;
      errorCode = -1;
      prevErrorCode = -1;
      // return;
    }
  }

  if (errorCode == 5) {
    if (prevErrorCode == -1) {
      add_log("ER5: FAILED_DIVERTER_ERROR");
    }
    if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
      currentStation = "X";
      expectedStation = "X";
      localisationFlag = true;
      setTagetVelocity(S0_5);
      errorMode = false;
      errorCode = -1;
      prevErrorCode = -1;
      // return;
    }
  }
  
  if (errorCode == 4) {
    if (prevErrorCode == -1) {
      add_log("ER4: INVALID_STATION_ERROR");
    }
    if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
      currentStation = "X";
      expectedStation = "X";
      localisationFlag = true;
      setTagetVelocity(S0_5);
      errorMode = false;
      errorCode = -1;
      prevErrorCode = -1;
      return;
    }
  }

  // if (errorCode == 6) {
  //   if (prevErrorCode == -1) {
  //     add_log("ER6: BUMPER_HIT_ERROR");
  //   }
  //   if (!mcp.digitalRead(PANEL_LED_BUTTON) && digitalRead(BUMPER_SENSOR)) {
  //     currentStation = "X";
  //     expectedStation = "X";
  //     localisationFlag = true;
  //     setTagetVelocity(S0_5);
  //     errorMode = false;
  //     errorCode = -1;
  //     prevErrorCode = -1;
  //     return;
  //   }
  // }
}

// Function to get the expected station after the given station
String GetExpectedStationFromStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_ID_TO_EXPECTED_STATION_ID_MAP.find(stationCodeString) != STATION_ID_TO_EXPECTED_STATION_ID_MAP.end()) {
    string stationID = STATION_ID_TO_EXPECTED_STATION_ID_MAP.at(stationCodeString);
    return String(stationID.c_str());
  }
  return "";
}

// Flap function - To turn ON the electromagnets and command secondary MCU to start closing the flaps
void StopFlap() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, HIGH);
}

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

// Function to check if diverter is in correct position or not
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
            }
            if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
              taskENTER_CRITICAL(&xErrorMux);
              errorMode = true;
              taskEXIT_CRITICAL(&xErrorMux);
              errorCode = 5;
              return;
            }
            delay(1);
            code = 1;
            count = 0;
            while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
              current_position_rear_servo = sm.ReadPos(REAR_SERVO);
              code = sm.getLastError();
              count++;
            }
            if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
              taskENTER_CRITICAL(&xErrorMux);
              errorMode = true;
              taskEXIT_CRITICAL(&xErrorMux);
              errorCode = 5;
              return;
            }
            delay(1);
            bool direction = GetDirectionFromStationId(currentStation);
            if (direction) {
              if (current_position_front_servo > front_diverter_right_thresold - DIVERTER_POSITION_ERROR_LIMIT && current_position_rear_servo < rear_diverter_right_thresold + DIVERTER_POSITION_ERROR_LIMIT) {
                // Correct diversion
              } else {
                taskENTER_CRITICAL(&xErrorMux);
                errorMode = true;
                taskEXIT_CRITICAL(&xErrorMux);
                errorCode = 5;
                add_log("Current front servo position: " + String(current_position_front_servo) + " Current rear servo position: " + String(current_position_rear_servo));
                add_log("Front diverter right threshold: " + String(front_diverter_right_thresold) + " Rear diverter right threshold: " + String(rear_diverter_right_thresold));
              }
            } else {
              if (current_position_front_servo < front_diverter_left_thresold + DIVERTER_POSITION_ERROR_LIMIT && current_position_rear_servo > rear_diverter_left_thresold - DIVERTER_POSITION_ERROR_LIMIT) {
                // Correct diversion
              } else {
                taskENTER_CRITICAL(&xErrorMux);
                errorMode = true;
                taskEXIT_CRITICAL(&xErrorMux);
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
    if (xSemaphoreTake(xDropoffStationMutex, portMAX_DELAY) == pdTRUE) {
      droppingStationAPITask = "D18";
      xSemaphoreGive(xDropoffStationMutex);
    }
  } else {
    int destinationIndex = response.indexOf('-');
    if (xSemaphoreTake(xDropoffStationMutex, portMAX_DELAY) == pdTRUE) {
      droppingStationAPITask = response.substring(0, destinationIndex);
      xSemaphoreGive(xDropoffStationMutex);
    }
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
  String url = DMS_URL_PREFIX + "UpdateParcelDroppedInDumpForBot?botId=" + BOT_ID;
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
      taskENTER_CRITICAL(&xGetDestinationAPIMux);
      getDestinationAPIFlag = false;
      taskEXIT_CRITICAL(&xGetDestinationAPIMux);
    }

    if (clearInfeedAPIFlag) {
      ClearInfeedStation();
      clearInfeedAPIFlag = false;
    }

    if (updateParcelDroppedInTheDumpAPIFlag) {
      UpdateParcelDroppedInTheDump();
      updateParcelDroppedInTheDumpAPIFlag = false;
      if (xSemaphoreTake(xDropoffStationMutex, portMAX_DELAY) == pdTRUE) {
        droppingStationAPITask = "X";
        xSemaphoreGive(xDropoffStationMutex);
      }
    }

    if (updateParcelDroppedAPIFlag) {
      UpdateParcelDropped();
      updateParcelDroppedAPIFlag = false;
      if (xSemaphoreTake(xDropoffStationMutex, portMAX_DELAY) == pdTRUE) {
        droppingStationAPITask = "X";
        xSemaphoreGive(xDropoffStationMutex);
      }
    }
  }
}

// RTOS task for the WTM
void WTM_OPS(void* pvParameters) {
  while(true) {
    vTaskDelay(2 / portTICK_PERIOD_MS);
    // UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("WTM_OPS min free stack: ");
    // Serial.print(watermark);
    // Serial.print(" words (");
    // Serial.print(watermark * 4);
    // Serial.println(" bytes)");
    if (errorMode) continue;
    if (xSemaphoreTake(xWTMDetectionSemaphore, portMAX_DELAY) == pdTRUE) {
      columnDetectedForCore0 = columnDetected;
      trafficDetectedForCore0 = trafficDetected;
      xSemaphoreGive(xWTMDetectionSemaphore);
    }

    if (columnDetectedForCore0) {
      mcp.digitalWrite(PANEL_LED_PIN, HIGH);
      if (trafficDetectedForCore0) {
        taskENTER_CRITICAL(&xWTMMux);
        WTMInAction = true;
        taskEXIT_CRITICAL(&xWTMMux);
        // add_log("TI - ON");
        // firstTimeTrafficDetected = true;
        HandleTrafficIndicatorDetectionON();
      } else {
        // if (firstTimeTrafficDetected) add_log("TI - OFF");
        HandleTrafficIndicatorDetectionOFF();
        taskENTER_CRITICAL(&xWTMMux);
        WTMInAction = false;
        taskEXIT_CRITICAL(&xWTMMux);
      }
    } else {
      mcp.digitalWrite(PANEL_LED_PIN, LOW);
      if (stopBotFlag && getVelocityActualValueAveraged() == 0) {
        taskENTER_CRITICAL(&xWTMMux);
        WTMInAction = true;
        taskEXIT_CRITICAL(&xWTMMux);
        setTagetVelocity(S0_03);
        add_log("CORRECTION");
        stopBotFlag = false;
        slowDownFlag = true;
      }
    }

    // ---------------------------------------------------------------- Infeed Controller --------------------------------------------------------
    if (columnDetectedForCore0) {
      if(!previousCIState) {
        if (startColumnCounter) {
          taskENTER_CRITICAL(&xColumnCountCompleteMux);
          columnCounter++;
          taskEXIT_CRITICAL(&xColumnCountCompleteMux);
        }
      }
    }
    previousCIState = columnDetectedForCore0;
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------- Destination Controller --------------------------------------------------------
    if (columnDetectedForCore0) {
      if(!previousCIStateForInfeedAPI) {
        if (startInfeedAPIColumnCounter) {
          taskENTER_CRITICAL(&xColumnCountCompleteAPIMux);
          infeedAPIcolumnCounter++;
          taskEXIT_CRITICAL(&xColumnCountCompleteAPIMux);
        }
      }
    }
    previousCIStateForInfeedAPI = columnDetectedForCore0;
    // -------------------------------------------------------------------------------------------------------------------------------------------
  }
}

// RTOS task for the bot operations
void STATION_DETECTION_OPS(void* pvParameters) {
  while(true) {
    vTaskDelay(2 / portTICK_PERIOD_MS);
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("STATION_DETECTION_OPS min free stack: ");
    // Serial.print(watermark);
    // Serial.print(" words (");
    // Serial.print(watermark * 4);
    // Serial.println(" bytes)");
    if (errorMode) {
      if (prevErrorCode == -1) {
        sendCommand("ERROR: ER-" + String(errorCode));
        BeepOn();
        haltMovement();
      }
      // taskENTER_CRITICAL(&xErrorMux);
      ErrorHandling(errorCode);
      prevErrorCode = errorCode;
      sendCommand("WARNING: ERROR-HANDLING");
      // taskEXIT_CRITICAL(&xErrorMux);
    }

    if (xSemaphoreTake(xStationCodeSemaphore, portMAX_DELAY) == pdTRUE) {
      if (detectedStationCodeCore0 != detectedStationCode) {
        detectedStationCodeCore0 = detectedStationCode;
      }
      xSemaphoreGive(xStationCodeSemaphore);
    }

    if (prevDetectedStationCodeCore0 != detectedStationCodeCore0) {
      add_log("Station code: " + detectedStationCodeCore0);
      currentStation = TransformStationCodeIntoStationId(detectedStationCodeCore0);
      add_log("Current Station: " + currentStation);
      if (CheckStationValidity(detectedStationCodeCore0)) {
        bool direction = GetDirectionFromStationId(currentStation);
        SetDiverter(direction);
        if (currentStation != "I1") {
          diverterCheckFlag = true;
        }
        if (!localisationFlag) {
          if (expectedStation != currentStation) {
            errorCode = 3;
            taskENTER_CRITICAL(&xErrorMux);
            errorMode = true;
            taskEXIT_CRITICAL(&xErrorMux);
          }
        }
        else {
          localisationFlag = false;
          sendCommand("SUCCESS: EMPTY");
        }
        if (!errorMode && !WTMInAction) {
          if (currentStation == "D1") {
            setSpeed = S1;
            setSpeedInMPS = 1;
            setTagetVelocity(setSpeed);
          }

          if (currentStation == "D18") {
            setSpeed = S2;
            setSpeedInMPS = 2;
            setTagetVelocity(setSpeed);
          }

          if (currentStation == "D1") {
            setSpeed = S1;
            setSpeedInMPS = 1;
            setTagetVelocity(setSpeed);
          }

          if (currentStation == "I1") {
            // setSpeed = S0_2;
            // setSpeedInMPS = 0.2;
            // setTagetVelocity(setSpeed);
            setSpeed = S0_3;
            setSpeedInMPS = 0.3;
            setTagetVelocity(setSpeed);
          }

          if (currentStation == "I12") {
            setSpeed = S0_03;
            setSpeedInMPS = 0.03;
            setTagetVelocity(setSpeed);
            taskENTER_CRITICAL(&xStartColumnCounterMux);
            startColumnCounter = true;
            taskEXIT_CRITICAL(&xStartColumnCounterMux);
            slowDownFlag = true;
          }

          if (currentStation == "I11") {
            setSpeed = S2;
            setSpeedInMPS = 2;
            setTagetVelocity(setSpeed);
            StageFlap();
          }

          if (currentStation == "D2") {
            setSpeed = S2_5;
            setSpeedInMPS = 2.5;
            setTagetVelocity(setSpeed);
          }

          if(currentStation == dropoffStation) {
            OpenFlap();
            closingFlap = true;
            startClosingTime = millis();
          }
        }
        expectedStation = GetExpectedStationFromStationId(currentStation);
      } else {
        add_log("Invalid station code: " + detectedStationCodeCore0);
        errorCode = 4;
        taskENTER_CRITICAL(&xErrorMux);
        errorMode = true;
        taskEXIT_CRITICAL(&xErrorMux);
      }
      prevDetectedStationCodeCore0 = detectedStationCodeCore0;
    }

    // Start closing the flaps after threshold time of detecting the drop-off
    if (closingFlap && millis() - startClosingTime >= THRESHOLD_TIME_TO_CLOSE_FLAPS) {
      CloseFlap();
      closingFlap = false;
    }

    // Infeed stopping profiler
    if (startColumnCounter) {
      if(columnCounter >= 2) {
        BeepOn();
        setSpeed = S1;
        taskENTER_CRITICAL(&xStartColumnCounterMux);
        startColumnCounter = false;
        taskEXIT_CRITICAL(&xStartColumnCounterMux);
        taskENTER_CRITICAL(&xColumnCountCompleteMux);
        columnCounter = 0;
        taskEXIT_CRITICAL(&xColumnCountCompleteMux);
        taskENTER_CRITICAL(&xStartColumnCounterAPIMux);
        startInfeedAPIColumnCounter = true;
        taskEXIT_CRITICAL(&xStartColumnCounterAPIMux);
      }
    }

    // API trigger
    if (startInfeedAPIColumnCounter) {
      if(infeedAPIcolumnCounter >= 10) {
        BeepOn();
        taskENTER_CRITICAL(&xGetDestinationAPIMux);
        getDestinationAPIFlag = true;
        taskEXIT_CRITICAL(&xGetDestinationAPIMux);
        taskENTER_CRITICAL(&xColumnCountCompleteAPIMux);
        infeedAPIcolumnCounter = 0;
        taskEXIT_CRITICAL(&xColumnCountCompleteAPIMux);
        taskENTER_CRITICAL(&xStartColumnCounterAPIMux);
        startInfeedAPIColumnCounter = false;
        taskEXIT_CRITICAL(&xStartColumnCounterAPIMux);
      }
    }

    // Indicator state based on dropping assignment
    if (xSemaphoreTake(xDropoffStationMutex, portMAX_DELAY) == pdTRUE) {
      dropoffStation = droppingStationAPITask;
      xSemaphoreGive(xDropoffStationMutex);
    }
    if (previousDroppingStation != dropoffStation) {
      if (dropoffStation == "X") sendCommand("SUCCESS: FULL-" + dropoffStation);
      else if (dropoffStation == "D18") sendCommand("WARNING: FULL-DUMP");
      else sendCommand("SUCCESS: EMPTY");
      previousDroppingStation = dropoffStation;
    }

    if(beepOn && millis() - beepStartTime >= 250) {
      BeepOff();
    }

    CheckDiverter(columnDetectedForCore0);

    // if (!errorMode && !digitalRead(BUMPER_SENSOR)) {
    //   errorMode = true;
    //   errorCode = 6;
    // }
  }
}

void setup() {
  MCPConfig();
  PinConfig();
  Serial.begin(115200);
  delay(3000);
  xStationCodeSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xStationCodeSemaphore);
  xWTMDetectionSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xWTMDetectionSemaphore);
  xDropoffStationMutex = xSemaphoreCreateBinary();
  xSemaphoreGive(xDropoffStationMutex);
  WiFiConfig();
  CANConfig();
  MotorConfig();
  ServoConfig();
  LoadDiverterConfig();
  ErrorIndicatorConfig();
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    8192,
    NULL,
    1,
    &httpDebugLog,
    0);
  CloseFlap();
  Beep();
  delay(100);
  DiverterTest();
  delay(1000);
  setSpeed = S2;
  setTagetVelocity(setSpeed);
  xTaskCreatePinnedToCore(
    STATION_DETECTION_OPS,
    "station_detection_ops",
    2048,
    NULL,
    2,
    &stationDetectionOps,
    0);
  vTaskDelay(pdMS_TO_TICKS(20));
  xTaskCreatePinnedToCore(
    WTM_OPS,
    "wtm_ops",
    2560,
    NULL,
    3,
    &WTMDetectionOps,
    0);
  vTaskDelay(pdMS_TO_TICKS(10));
  xTaskCreatePinnedToCore(
    API_TASK,
    "api_task",
    4096,
    NULL,
    4,
    &apiTask,
    0);
  delay(500);
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  Serial.println(__FILE__);
  sendCommand("SUCCESS: EMPTY");
  add_log("Starting the bot!");
}

void loop() {
  ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

  String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

  if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
    if (xSemaphoreTake(xStationCodeSemaphore, portMAX_DELAY) == pdTRUE) {
      detectedStationCode = stationCodeMain;
      xSemaphoreGive(xStationCodeSemaphore);
    }
  }

  if (xSemaphoreTake(xWTMDetectionSemaphore, portMAX_DELAY) == pdTRUE) {
    trafficDetected = TrafficIndicatorDetection();
    columnDetected = ColumnIndicatorDetection();
    if (columnDetected) {
      if (onColumnForCounting) {
        columnCounterForDetection++;
      } else {
        onColumnForCounting = true;
        columnCounterForDetection = 1;
        columnNo++;
      }
    } else {
      if (onColumnForCounting) {                   
        onColumnForCounting = false;
        // add_log("CNo.: " + String(columnNo) + " | FC: " + String(columnCounterForDetection));
      }
    }

    if (trafficDetected) {
      if (onTrafficForCounting) {
        trafficCounterForDetection++;
      } else {
        onTrafficForCounting = true;
        trafficCounterForDetection = 1;
        trafficNo++;
      }
    } else {
      if (onTrafficForCounting) {                   
        onTrafficForCounting = false;
        // add_log("TNo.: " + String(trafficNo) + " | FC: " + String(trafficCounterForDetection));
      }
    }
    xSemaphoreGive(xWTMDetectionSemaphore);
  }
}

// Define the configuration for the servo object
void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
  sm.IncreaseLimit(2, 5000);
}

// Function to load diverter position values from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);
  BOT_ID = prefs.getString("BotId", "x");

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

// Function to make ahardware beep for 250 ms
void Beep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

void BeepOn() {
  digitalWrite(BUZZER, HIGH);
  beepStartTime = millis();
  beepOn = true;
}

void BeepOff() {
  digitalWrite(BUZZER, LOW);
  beepOn = false;
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (debugLoggingString != BOT_ID + " " + CODE_ID + ": ") {
      int httpCode = -1;
      if (loggerFlag) {
        httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
        httpCode = httpDebugger.POST(debugLoggingString);
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = BOT_ID + " " + CODE_ID + ": ";
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
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT);       // Traffic indicator pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT);      // Column indicator pin config
  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);
  // pinMode(BUMPER_SENSOR, INPUT_PULLUP);

  mcp.digitalWrite(PANEL_LED_PIN, LOW);
  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, LOW);
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

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    // while (1);
  }
}

// Clear display
void clearDisplay() {
  sendCommand("CLEAR");
}

// Send command to secondary ESP32
void sendCommand(String command) {
  clearDisplay();
  Serial1.println(command);
  // Serial.println("Sent: " + command); // Debug output
}

void ErrorIndicatorConfig() {
  // Initialize UART communication with secondary ESP32 (using Serial1)
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);
  // Initialize LCD on secondary ESP32
  sendCommand("INIT");
  delay(100);
}