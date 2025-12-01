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
#define PANEL_LED_PIN 8
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35
#define PANEL_LED_BUTTON 7 
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define AFC_BOTTOM_LSW_PIN 3
#define AFC_TOP_LSW_PIN 1
#define AFC_PIN_1 19
#define AFC_PIN_2 18

#define ENABLE 1
#define DISABLE 0
#define SERVO_UART_DELAY 50

//------- BOT B5 -----------
#define CLOSE -100
#define STAGE -50
#define OFF 0
#define CLOSEPOS 0
#define STAGEPOS 15400
#define MAXLIM 24570
//--------------------------

// Constant variables
const String CODE_ID = "d534";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/log";
const int THRESHOLD_INTENSITY = 250;
const static int SR_D1 = 36;  // ESP32 pin attached to Photo diode 1
const static int SR_D2 = 39;  // ESP32 pin attached to Photo diode 2
const static int SR_D3 = 34;  // ESP32 pin attached to Photo diode 3
const static uint16_t PT_UPPER_THR_LIMIT = 600;  // Threshold limit for rising photo diode reading.
const static uint16_t PT_LOWER_THR_LIMIT = 400;  // Threshold limit for falling photo diode reading

const String DMS_URL_PREFIX = "http://192.168.2.109:8443/m-sort/m-sort-distribution-server/";
const String ENTITY_TYPE = "bot";
HTTPClient _http;

// Monitoring variables
unsigned long maxLoopTime;
unsigned long loopStartTime;
int numsOfIteration = 0;
unsigned long maxLatency = 0;
size_t maxMemoryUsed = 0;
const int MAX_REATTEMPTS_FOR_SERVO_COM = 5;

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
bool startColumnCounter = false;
int columnCounter = 0;
bool infeedDetected = false;
bool previousCIState = 0;
bool previousTIState = 0;
volatile int setSpeed;
bool beepFlag = false;
bool flapOpen = false;
int packetCount = -1;
String dropoffStation = "X";
bool getDestinationAPIFlag = false;
bool clearInfeedAPIFlag = false;
bool startInfeedAPIColumnCounter = false;
int infeedAPIcolumnCounter = 0;
bool previousCIStateForInfeedAPI = false;
TaskHandle_t apiTask;
String expectedStation = "X";
bool localisationFlag = true;
volatile bool errorMode = false;
volatile int errorCode = -1;
bool diverterCheckFlag = false;
bool previousCIStateDiversionCheck = 0;
int columnCounterDiversioncheck = 0;
bool flapMotorONFlag = false;
bool updateParcelDroppedInTheDumpAPIFlag = false;
bool updateParcelDroppedAPIFlag = false;
bool afcPinState = false;
unsigned long afcStartTime;
bool closingFlap = false;
unsigned long startClosingTime;

// Infeed stop variables
int _intermediateColumnFramesCount = 0;
int _instantaneousFramesCount = 0;
uint16_t _instantaneousIntensityMaxArray[3] = { 0 };
uint16_t _instantaneousIntensityMinArray[3] = { 1023 };
unsigned long stationReadStartTime;
int columnNo = 0;
volatile bool checkForCorrection = false;
volatile bool checkForStopCondition = false;
volatile int actualSetSpeed;
volatile bool diverterDirection;
bool previousDirection;
portMUX_TYPE diverterDirectionMux = portMUX_INITIALIZER_UNLOCKED;
int previousSetSpeed;
portMUX_TYPE currentStationMux = portMUX_INITIALIZER_UNLOCKED;
unsigned long diverterCheckStartTime;
portMUX_TYPE setSpeedMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE checkForCorrectionMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE checkForStopConditionMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t sensorReadingTask;
volatile bool changeDirection = false;
volatile bool firstTimeError = false;

// Handler for the case when TI is OFF
void HandleTrafficIndicatorDetectionOFF() {
  // Case: If condition true then the bot is in decelerated mode
  if (slowDownFlag) {
    add_log("STARTING BCZ NO TRAFFIC - HANDLE_TRAFFIC_OFF!");
    // Set the bot to the SET_SPEED
    portENTER_CRITICAL(&setSpeedMux);
    setSpeed = actualSetSpeed;
    add_log("SET SPEED UPDATE TO: " + String(setSpeed));
    portEXIT_CRITICAL(&setSpeedMux);
    add_log("BCZ TRAFFIC OFF STOP FLAG: FALSE");
    stopBotFlag = false;
    if (setSpeed != S0_05) {
      add_log("BCZ TRAFFIC OFF SLOW-DOWN FLAG: FALSE");
      slowDownFlag = false;
    }
  }
}

// Handler for the case when TI is ON
void HandleTrafficIndicatorDetectionON() {
  // Case: If condition true then the bot is in accelerated mode else bot is in decelerated mode
  if (!slowDownFlag) {
    // Set the bot to the MINIMAL_SPEED
    add_log("SLOWING BCZ TRAFFIC - HANDLE_TRAFFIC_ON!");
    portENTER_CRITICAL(&setSpeedMux);
    setSpeed = S0_05;
    add_log("SET SPEED UPDATE TO: " + String(setSpeed));
    portEXIT_CRITICAL(&setSpeedMux);
    add_log("BCZ TRAFFIC ON SLOW-DOWN FLAG: TRUE");
    slowDownFlag = true;
  } else {
    portENTER_CRITICAL(&checkForStopConditionMux);
    checkForStopCondition = true;
    portEXIT_CRITICAL(&checkForStopConditionMux);
  }
}

// Function which tells if traffic indicator is detected or not
bool TrafficIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int TI_rawValue = map(analogRead(TRAFFIC_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then traffic detected else not detected
  return TI_rawValue < THRESHOLD_INTENSITY;
}

// Function which tells if column indicator is detected or not
bool ColumnIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int CI_rawValue = map(analogRead(COLUMN_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then column detected else not detected
  return CI_rawValue < THRESHOLD_INTENSITY;
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

// Flap and diverter functions
void DivertLeft() {
  add_log("-- Diverting Left --");
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    firstTimeError = true;
    add_log("Front servo read failed!");
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
    firstTimeError = true;
    add_log("Rear servo read failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Front servo torque disable failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Front servo write failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Rear servo torque disable failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Rear servo write failed!");
      return;
    }
    delay(1);
  }
}

void DivertRight() {
  add_log("-- Diverting Right --");
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    errorMode = true;
    errorCode = 5;
    firstTimeError = true;
    add_log("Front servo read failed!");
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
    firstTimeError = true;
    add_log("Rear servo read failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Front servo torque disable failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Front servo write failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Rear servo torque disable failed!");
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
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Rear servo write failed!");
      return;
    }
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

void ErrorHandling() {
  if (errorCode == 3) {
    if (firstTimeError) {
      add_log("ER3: UNEXPECTED_STATION_ERROR");
      add_log("Expected Station: " + expectedStation + " Detected Station: " + currentStation);
      Beep();
      mcp.digitalWrite(PANEL_LED_PIN, HIGH);
      firstTimeError = false;
    }
    if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
      portENTER_CRITICAL(&setSpeedMux);
      setSpeed = S0_5;
      add_log("SET SPEED UPDATED TO: " + String(setSpeed) + " TO HANDLE ERROR");
      portEXIT_CRITICAL(&setSpeedMux);
      localisationFlag = true;
      dropoffStation = "D18";
      errorCode = -1;
      errorMode = false;
    }
  }

  if (errorCode == 5) {
    if (firstTimeError) {
      add_log("ER5: FAILED_DIVERTER_ERROR");
      Beep();
      mcp.digitalWrite(PANEL_LED_PIN, HIGH);
      firstTimeError = false;
    }
    if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
      portENTER_CRITICAL(&setSpeedMux);
      setSpeed = S0_5;
      add_log("SET SPEED UPDATED TO: " + String(setSpeed) + " TO HANDLE ERROR");
      portEXIT_CRITICAL(&setSpeedMux);
      localisationFlag = true;
      dropoffStation = "D18";
      errorCode = -1;
      errorMode = false;
    }
  }
  
  if (errorCode == 4) {
    if (firstTimeError) {
      add_log("ER4: INVALID_STATION_ERROR");
      Beep();
      mcp.digitalWrite(PANEL_LED_PIN, HIGH);
      firstTimeError = false;
    }
    if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
      portENTER_CRITICAL(&setSpeedMux);
      setSpeed = S0_5;
      add_log("SET SPEED UPDATED TO: " + String(setSpeed) + " TO HANDLE ERROR");
      portEXIT_CRITICAL(&setSpeedMux);
      localisationFlag = true;
      dropoffStation = "D18";
      errorCode = -1;
      errorMode = false;
    }
  }
}

void CheckDiverter() {
  if (diverterCheckFlag) {
    int code = 1;
    int count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      current_position_front_servo = sm.ReadPos(FRONT_SERVO);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      errorMode = true;
      errorCode = 5;
      firstTimeError = true;
      add_log("Front servo read failed during diverter check!");
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
      firstTimeError = true;
      add_log("Rear servo read failed during diverter check!");
      return;
    }
    delay(1);
    bool direction = GetDirectionFromStationId(currentStation);
    if (direction) {
      if (current_position_front_servo > front_diverter_right_thresold - 30 && current_position_rear_servo < rear_diverter_right_thresold + 30) {
        // Correct diversion
      } else {
        errorMode = true;
        errorCode = 5;
        firstTimeError = true;
        add_log("Servo failed to divert to right -->");
        add_log("Current front servo position: " + String(current_position_front_servo) + " Current rear servo position: " + String(current_position_rear_servo));
        add_log("Front diverter right threshold: " + String(front_diverter_right_thresold) + " Rear diverter right threshold: " + String(rear_diverter_right_thresold));
      }
    } else {
      if (current_position_front_servo < front_diverter_left_thresold + 30 && current_position_rear_servo > rear_diverter_left_thresold - 30) {
        // Correct diversion
      } else {
        errorMode = true;
        errorCode = 5;
        firstTimeError = true;
        add_log("Servo failed to divert to left <--");
        add_log("Current front servo position: " + String(current_position_front_servo) + " Current rear servo position: " + String(current_position_rear_servo));
        add_log("Front diverter left threshold: " + String(front_diverter_left_thresold) + " Rear diverter left threshold: " + String(rear_diverter_left_thresold));
      }
    }
  }
}

bool stopWhileStaging = false;
bool stopWhileClosing = false;

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

unsigned long offCounter = 0;
unsigned long onCounter = 0;
unsigned long loopLogTime = 0;
String prevCombo = "01";
unsigned long lastDemandVelocityCheckTime = 0;

void setup() {
  MCPConfig();
  PinConfig();
  delay(3000);
  Serial.begin(115200);
  WiFiConfig();
  CANConfig();
  MotorConfig();
  ServoConfig();
  LoadDiverterConfig();
  debugLoggingString = BOT_ID + " " + CODE_ID + ": ";
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    10000,
    NULL,
    1,
    &httpDebugLog,
    0);
  xTaskCreatePinnedToCore(
    API_TASK,
    "api_task",
    4096,
    NULL,
    1,
    &apiTask,
    0);
  CloseFlap();
  Beep();
  DiverterTest();
  previousDirection = 1;
  diverterDirection = 1;
  delay(1000);
  actualSetSpeed = S0_5;
  setSpeed = actualSetSpeed;
  add_log("SET SPEED UPDATED TO: " + String(setSpeed) + " FOR INITIALIZATION");
  previousSetSpeed = S0_5;
  setTagetVelocity(setSpeed);
  delay(500);
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  printFileName();
  xTaskCreatePinnedToCore(
    SENSOR_READING_TASK,
    "sensor_reading_task",
    10000,
    NULL,
    2,
    &sensorReadingTask,
    1);
  add_log("Starting the bot!");
}

// Logger Function which will run as RTOS task
void SENSOR_READING_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

    String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

    if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
      portENTER_CRITICAL(&currentStationMux);
      currentStation = TransformStationCodeIntoStationId(stationCodeMain);
      portEXIT_CRITICAL(&currentStationMux);
      add_log("Current station: " + currentStation);
      if (CheckStationValidity(stationCodeMain)) {
        portENTER_CRITICAL(&diverterDirectionMux);
        changeDirection = true;
        diverterDirection = GetDirectionFromStationId(currentStation);
        portEXIT_CRITICAL(&diverterDirectionMux);
        // add_log("Diverter direction changed to " + String(diverterDirection));
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
          actualSetSpeed = GetSetSpeedAsPerStation(currentStation);
          portENTER_CRITICAL(&setSpeedMux);
          setSpeed = actualSetSpeed;
          add_log("SET SPEED UPDATE TO: " + String(setSpeed) + " BCZ OF EDGE");
          portEXIT_CRITICAL(&setSpeedMux);
          if (currentStation == "I12") {
            startColumnCounter = true;
            add_log("BCZ OF I12 SLOW-DOWN SET: TRUE");
            slowDownFlag = true;
          }

          if (currentStation == "I11") {
            add_log("Asking secondary to stage flaps @" + String(millis()));
            StageFlap();
            stopWhileStaging = true;
          }

          if(currentStation == dropoffStation) {
            add_log("EMG - OFF Open flaps @" + String(millis()));
            OpenFlap();
            closingFlap = true;
            startClosingTime = millis();
            // dropoffStation = "X";
            if (dropoffStation == "D18") {
              updateParcelDroppedInTheDumpAPIFlag = true;
            } else {
              updateParcelDroppedAPIFlag = true;
            }
          }
          expectedStation = GetExpectedStationFromStationId(currentStation);
        }
      } else {
        add_log("Invalid station code: " + stationCodeMain);
        errorCode = 4;
        errorMode = true;
        firstTimeError = true;
      }
    }

    // ------------------------------------------------------------------------- WTM -------------------------------------------------------------
    bool columnDetected = ColumnIndicatorDetection();
    if (columnDetected) {
      // mcp.digitalWrite(PANEL_LED_PIN, HIGH);
      bool trafficDetected = TrafficIndicatorDetection();
      String currentCombo = String(trafficDetected) + String(columnDetected);
      if (prevCombo != currentCombo) {
        if (trafficDetected) add_log("TRAFFIC DETECTED!");
        else add_log("TRAFFIC NOT DETECTED!");
      }
      prevCombo = currentCombo;
      if (trafficDetected) {
        HandleTrafficIndicatorDetectionON();
      } else {
        HandleTrafficIndicatorDetectionOFF();
      }
    } else {
      // mcp.digitalWrite(PANEL_LED_PIN, LOW);
      portENTER_CRITICAL(&checkForCorrectionMux);
      checkForCorrection = true;
      portEXIT_CRITICAL(&checkForCorrectionMux);
    }
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------- Infeed Controller --------------------------------------------------------
    if (columnDetected) {
      if(!previousCIState) {
        if (startColumnCounter) {
          columnCounter++;
          if(columnCounter >= 2) {
            beepFlag = true;
            actualSetSpeed = S1;
            add_log("Actual speed changed to " + String(actualSetSpeed) + "after 2 columns to start fast from infeed");
            startColumnCounter = false;
            columnCounter = 0;
            infeedDetected = false;
            startInfeedAPIColumnCounter = true;
          }
        }
      }
    }
    previousCIState = columnDetected;
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------- Destination Controller --------------------------------------------------------
    if (columnDetected) {
      if(!previousCIStateForInfeedAPI) {
        if (startInfeedAPIColumnCounter) {
          infeedAPIcolumnCounter++;
          if(infeedAPIcolumnCounter >= 10) {
            getDestinationAPIFlag = true;
            infeedAPIcolumnCounter = 0;
            startInfeedAPIColumnCounter = false;
            beepFlag = true;
            // doBeep = true;
          }
        }
      }
    }
    previousCIStateForInfeedAPI = columnDetected;
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // Start closing the flaps after threshold time of detecting the drop-off
    if (closingFlap && millis() - startClosingTime >= 2000) {
      add_log("Asking secondary to close flaps @" + String(millis()));
      CloseFlap();
      closingFlap = false;
      stopWhileClosing = true;
    }
  }
}

void loop() {
  // unsigned long loopStartTime = micros();
  // unsigned long latency = micros();

  // if (millis() - loopLogTime >= 5000) {
  //   add_log("LOOP EXECUTES!");
  //   add_log("PREV. SET SPEED: " + String(previousSetSpeed) + " SET SPEED: " + String(setSpeed) + " ERROR MODE: " + String(errorMode));
  //   loopLogTime = millis();
  // }

  if (errorMode) {
    haltMovement();
    ErrorHandling();
  }

  if (changeDirection) {
    // add_log("Changing diverter to " + String(diverterDirection));
    SetDiverter(diverterDirection);
    changeDirection = false;
    diverterCheckStartTime = millis();
    diverterCheckFlag = true;
  }

  if (millis() - lastDemandVelocityCheckTime >= 200) {
    int32_t demandVelocityEPOS = getVelocityDemandValue();
    if (setSpeed != demandVelocityEPOS) {
      // add_log("UPDATING THE PREV SPEED. USING DEMAND VELOCITY : " + String(previousSetSpeed) + " TO " + String(demandVelocityEPOS));
      previousSetSpeed = demandVelocityEPOS;
    }
    lastDemandVelocityCheckTime = millis();
  }

  if (previousSetSpeed != setSpeed && !errorMode) {
    // add_log("CHANGING SPEED TO " + String(setSpeed) + " FROM " + String(previousSetSpeed));
    if (setSpeed != 0) {
      setTagetVelocity(setSpeed);
    } else {
      haltMovement();
    }
    previousSetSpeed = setSpeed;
  }

  if (checkForCorrection && !errorMode) {
    if (stopBotFlag) {
      int speed = getVelocityActualValueAveraged();
      // add_log("ASKING SPEED FEEDBACK FROM MAXON - HANDLE_STOPPED_BETWEEEN_COLUMNS_CORRECTION: " + String(speed));
      if (speed == 0) {
        // add_log("CORRECTION TO STOP AT THE NEXT TRAFFIC COLUMN");
        portENTER_CRITICAL(&setSpeedMux);
        setSpeed = S0_05;
        add_log("SET SPEED UPDATE TO: " + String(setSpeed));
        portEXIT_CRITICAL(&setSpeedMux);
        add_log("STOP FLAG SET: FALSE");
        stopBotFlag = false;
      }
    }
    portENTER_CRITICAL(&checkForCorrectionMux);
    checkForCorrection = false;
    portEXIT_CRITICAL(&checkForCorrectionMux);
  }

  if (checkForStopCondition && !errorMode) {
    // Case: If condition true then bot has achieved MINIMAL_SPEED
    if (!stopBotFlag) {
      // Case: If condition true then bot has not already stopped
      int speed = getVelocityActualValueAveraged();
      // add_log("ASKING SPEED FEEDBACK FROM MAXON - HANDLE_TRAFFIC_ON_LOOP: " + String(speed));
      if (speed < 30) {
        // add_log("STOPPING BCZ TRAFFIC ON - HANDLE_TRAFFIC_ON_LOOP");
        // Stop the bot
        portENTER_CRITICAL(&setSpeedMux);
        setSpeed = 0;
        add_log("SET SPEED UPDATE TO: " + String(setSpeed));
        portEXIT_CRITICAL(&setSpeedMux);
        add_log("BOTH STOP AND SLOW-DOWN FLAGS SET: TRUE");
        stopBotFlag = true;
        slowDownFlag = true;
      }
    }
    portENTER_CRITICAL(&checkForStopConditionMux);
    checkForStopCondition = false;
    portEXIT_CRITICAL(&checkForStopConditionMux);
  }

  if (beepFlag) {
    Beep();
    beepFlag = false;
  }

  if (millis() - diverterCheckStartTime > 300 && diverterCheckFlag && currentStation != "I1") {
    CheckDiverter();
    diverterCheckFlag = false;
  }
}

void printFileName() {
  Serial.println(__FILE__);
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

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    // while (1);
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
    if (debugLoggingString != BOT_ID + " " + CODE_ID + ": ") {
      // Serial.println("Sending logs!");
      // Serial.println(String(WiFi.status()));
      int httpCode = -1;
      String logUrl = HTTP_DEBUG_SERVER_URL + "?entity=" + ENTITY_TYPE + "&entity_id=" + BOT_ID;
      if (loggerFlag) {
        httpDebugger.begin(logUrl);
        httpCode = httpDebugger.POST(debugLoggingString);
        // Serial.println("Response: " + httpDebugger.getString());
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = BOT_ID + " " + CODE_ID + ": ";
      } else {
        add_log("Logging failed in previous try!");
      }
    }
  }
}

// Function to add logs into the log string
void add_log(String log) {
  debugLoggingString += " | " + log;
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
    add_log("New Max. Memory Usage (b): " + String(totalHeap - freeHeap));
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
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT);      // Column indicator pin config
  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
  mcp.pinMode(AFC_TOP_LSW_PIN, INPUT_PULLUP);
  mcp.pinMode(AFC_BOTTOM_LSW_PIN, INPUT_PULLUP);

  // GPIO command outputs to Secondary
  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);

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

void GetDestinationAndJourneyPath() {
  String url = DMS_URL_PREFIX + "GetDestinationStationIdForBot?botId=" + BOT_ID + "&infeedStationId=I1&strategyType=MyntraSingleScan&retries=0&sectionId=S1";
  add_log("Get destination API call: " + url);
  int httpCode = 0;
  _http.begin(url);
  httpCode = _http.GET();
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
    add_log("Success: " + response);
  } else {
    add_log("Error: " + _http.getString());
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
  String url = DMS_URL_PREFIX + "UpdateSuccessfulDropForBot?botId=" + BOT_ID;
  add_log("Update API call: " + url);
  int httpCode = 0;
  _http.begin(url);
  httpCode = _http.PUT(url);
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
    add_log("Success: " + response);
  } else {
    add_log("Error: " + _http.getString());
  }
  _http.end();
}

void UpdateParcelDroppedInTheDump() {
  String url = DMS_URL_PREFIX + "UpdateDumpDropForBot?botId=" + BOT_ID;
  add_log("Update in dump API call: " + url);
  int httpCode = 0;
  _http.begin(url);
  httpCode = _http.PUT(url);
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
    add_log("Success: " + response);
  } else {
    add_log("Error: " + _http.getString());
  }
  _http.end();
}

void ClearInfeedStation() {
  String url = DMS_URL_PREFIX + "ClearInfeedStations?infeedStationId=I1";
  add_log("Clear infeed API call: " + url);
  int httpCode = 0;
  _http.begin(url);
  while (httpCode != HTTP_CODE_OK) {
    httpCode = _http.PUT(url);
    if (httpCode == HTTP_CODE_OK) {
      add_log("Success: " + _http.getString());
    } else {
      add_log("Error: " + _http.getString());
    }
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