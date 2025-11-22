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
const String CODE_ID = "d426";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const int THRESHOLD_INTENSITY = 250;
const static int SR_D1 = 36;  // ESP32 pin attached to Photo diode 1
const static int SR_D2 = 39;  // ESP32 pin attached to Photo diode 2
const static int SR_D3 = 34;  // ESP32 pin attached to Photo diode 3
const static uint16_t PT_UPPER_THR_LIMIT = 600;  // Threshold limit for rising photo diode reading.
const static uint16_t PT_LOWER_THR_LIMIT = 400;  // Threshold limit for falling photo diode reading

const String DMS_URL_PREFIX = "http://192.168.2.109:8443/m-sort-distribution-server/";
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
bool startColumnCounter = false;
int columnCounter = 0;
bool infeedDetected = false;
bool previousCIState = 0;
bool previousTIState = 0;
int setSpeed;
bool beepFlag = true;
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
bool errorMode = false;
int errorCode = -1;
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
uint8_t dataInstantaneousColumn[800] = { 0 };
uint8_t dataIntermediateColumn[800] = { 0 };
uint16_t dataFinalCode[800] = { 0 };
uint8_t dataPreviousInstantaneousColumn[800] = { 0 };
bool dataCapture = false;
int indexOfArray = 0;

// Handler for the case when TI is OFF
void HandleTrafficIndicatorDetectionOFF() {
  // Case: If condition true then the bot is in decelerated mode
  if (slowDownFlag) {
    // Set the bot to the SET_SPEED
    setTagetVelocity(setSpeed);
    stopBotFlag = false;
    if (setSpeed != S0_05) slowDownFlag = false;
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
    // Case: If condition true then bot has achieved MINIMAL_SPEED
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

String GetStationCodeFromDataReadByStationSensor() {
  String instColumnCode = String(_photoTransistorCurrentStateArray[0]) + String(_photoTransistorCurrentStateArray[1]) + String(_photoTransistorCurrentStateArray[2]);
  if (instColumnCode == "111") {
    _intermediateColumnCode = instColumnCode;
  }

  // START STATION READING
  if (!_readingInProcess && instColumnCode == "111") {
    _readingInProcess = true;
    if (currentStation == "D14") {
      dataCapture = true;
    }
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

    if (_previousIntermediateColumnCode != "000" && _previousIntermediateColumnCode != "111" && _intermediateColumnCode == "111" && _intermediateColumnCode != "000" && _finalColumnCodeArray[0] != _previousIntermediateColumnCode) {
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
  if (dataCapture && indexOfArray < 800) {
    int decimalValue = 0;
    for (int i = 0; i < 3; i++) {
      decimalValue = (decimalValue << 1) | (instColumnCode[i] - '0');
    }
    dataInstantaneousColumn[indexOfArray] = decimalValue;

    decimalValue = 0;
    for (int i = 0; i < 3; i++) {
      decimalValue = (decimalValue << 1) | (_intermediateColumnCode[i] - '0');
    }
    dataIntermediateColumn[indexOfArray] = decimalValue;

    decimalValue = 0;
    String _code = String(_photoTransistorPreviousStateArray[0]) + String(_photoTransistorPreviousStateArray[1]) + String(_photoTransistorPreviousStateArray[2]);
    for (int i = 0; i < 3; i++) {
      decimalValue = (decimalValue << 1) | (_code[i] - '0');
    }
    dataPreviousInstantaneousColumn[indexOfArray] = decimalValue;

    decimalValue = 0;
    _code = _finalColumnCodeArray[2] + _finalColumnCodeArray[1] + _finalColumnCodeArray[0];
    for (int i = 0; i < 9; i++) {
      decimalValue = (decimalValue << 1) | (_code[i] - '0');
    }
    dataFinalCode[indexOfArray] = decimalValue;
    indexOfArray++;
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

// Flap and diverter functions
void DivertLeft() {
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
      errorMode = true;
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
      errorMode = true;
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
      errorMode = true;
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
      errorMode = true;
      errorCode = 5;
      return;
    }
    delay(1);
  }
}

void DivertRight() {
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
    add_log("ER3: UNEXPECTED_STATION_ERROR");
    add_log("Expected Station: " + expectedStation + " Detected Station: " + currentStation);
    Beep();
    mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    while(mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
    }
    if (dropoffStation != "X") {
      dropoffStation = "D18";
    }
    setTagetVelocity(S0_5);
    String stationCodeMain = "";
    while (stationCodeMain == "") {
      ReadStationViaPhotoDiode();
      stationCodeMain = GetStationCodeFromDataReadByStationSensor();
    }
    haltMovement();
    delay(2000);
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    if (CheckStationValidity(stationCodeMain)) {
      bool direction = GetDirectionFromStationId(currentStation);
      SetDiverter(direction);
      delay(1000);
      setTagetVelocity(setSpeed);
      expectedStation = GetExpectedStationFromStationId(currentStation);
    } else {
      errorMode = true;
      errorCode = 4;
    }
  }

  if (errorCode == 5) {
    add_log("ER5: FAILED_DIVERTER_ERROR");
    Beep();
    mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    while(mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
    }
    if (dropoffStation != "X") {
      dropoffStation = "D18";
    }
    setTagetVelocity(S0_5);
    String stationCodeMain = "";
    while (stationCodeMain == "") {
      ReadStationViaPhotoDiode();
      stationCodeMain = GetStationCodeFromDataReadByStationSensor();
    }
    haltMovement();
    delay(2000);
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    if (CheckStationValidity(stationCodeMain)) {
      bool direction = GetDirectionFromStationId(currentStation);
      SetDiverter(direction);
      delay(1000);
      setTagetVelocity(setSpeed);
      expectedStation = GetExpectedStationFromStationId(currentStation);
    } else {
      errorMode = true;
      errorCode = 4;
    }
  }
  
  if (errorCode == 4) {
    add_log("ER4: INVALID_STATION_ERROR");
    Beep();
    mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    while(mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
    }
    if (dropoffStation != "X") {
      dropoffStation = "D18";
    }
    setTagetVelocity(S0_5);
    String stationCodeMain = "";
    while (stationCodeMain == "") {
      ReadStationViaPhotoDiode();
      stationCodeMain = GetStationCodeFromDataReadByStationSensor();
    }
    haltMovement();
    delay(2000);
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    if (CheckStationValidity(stationCodeMain)) {
      bool direction = GetDirectionFromStationId(currentStation);
      SetDiverter(direction);
      delay(1000);
      setTagetVelocity(setSpeed);
      expectedStation = GetExpectedStationFromStationId(currentStation);
    } else {
      add_log("Two consecutive invalid station found - Please check the station codes in the system.");
    }
  }
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
            }
            if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
              errorMode = true;
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

void setup() {
  MCPConfig();
  PinConfig();
  delay(3000);
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
  xTaskCreatePinnedToCore(
    API_TASK,
    "api_task",
    4096,
    NULL,
    1,
    &apiTask,
    0);
  CANConfig();
  MotorConfig();
  ServoConfig();
  LoadDiverterConfig();
  CloseFlap();
  Beep();
  DiverterTest();
  delay(1000);
  setSpeed = S2;
  setTagetVelocity(setSpeed);
  delay(500);
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  printFileName();
  add_log("Starting the bot!");
}

void loop() {
  unsigned long loopStartTime = micros();
  unsigned long latency = micros();

  if (errorMode) {
    haltMovement();
    ErrorHandling();
    errorMode = false;
    errorCode = -1;
  }

  // StopFlapClosingMotorIfRuning();

  ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

  String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

  if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
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
        if (currentStation == "D1") {
          setSpeed = S1;
          setTagetVelocity(setSpeed);
        }

        if (currentStation == "D15") {
          dataCapture = false;
        }

        if (currentStation == "D18") {
          haltMovement();
        }

        if (currentStation == "I1") {
          setSpeed = S0_3;
          setTagetVelocity(setSpeed);
          // setSpeed = S0_05;
          // setTagetVelocity(setSpeed);
          // slowDownFlag = true;
          packetCount++;
        }

        if (currentStation == "I12") {
          setSpeed = S0_05;
          setTagetVelocity(setSpeed);
          infeedDetected = true;
          startColumnCounter = true;
          slowDownFlag = true;
          // setSpeed = S0_05;
        }

        if (currentStation == "I11") {
          setSpeed = S1;
          setTagetVelocity(setSpeed);
          StageFlap();
          stopWhileStaging = true;
        }

        if (packetCount != -1) {
          if(currentStation == dropoffStation) {
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
        }
      }
      expectedStation = GetExpectedStationFromStationId(currentStation);
    } else {
      add_log("Invalid station code: " + stationCodeMain);
      errorCode = 4;
      errorMode = true;
    }
  }

  // ------------------------------------------------------------------------- WTM -------------------------------------------------------------
  bool columnDetected = ColumnIndicatorDetection();
  if (columnDetected) {
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
  // -------------------------------------------------------------------------------------------------------------------------------------------

  // ---------------------------------------------------------------- Infeed Controller --------------------------------------------------------
  if (columnDetected) {
    if(!previousCIState) {
      if (startColumnCounter) {
        columnCounter++;
        if(columnCounter >= 2) {
          Beep();
          setSpeed = S1;
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
          Beep();
          for (int i = 0 ; i < 100 ; i++) {
            add_log(String(i) + ": " + String(dataInstantaneousColumn[i]) + ", " + String(dataIntermediateColumn[i]) + ", " + String(dataPreviousInstantaneousColumn[i]) + ", " + String(dataFinalCode[i]));
            dataInstantaneousColumn[i] = 0;
            dataIntermediateColumn[i] = 0;
            dataPreviousInstantaneousColumn[i] = 0;
            dataFinalCode[i] = 0;
          }
          indexOfArray = 0;
        }
      }
    }
  }
  previousCIStateForInfeedAPI = columnDetected;
  // -------------------------------------------------------------------------------------------------------------------------------------------

  // Start closing the flaps after threshold time of detecting the drop-off
  if (closingFlap && millis() - startClosingTime >= 2000) {
    CloseFlap();
    closingFlap = false;
    stopWhileClosing = true;
  }
  
  CheckDiverter(columnDetected);
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
      if (loggerFlag) {
        httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
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
  String url = DMS_URL_PREFIX + "UpdateSuccessfulDropForBot?botId=" + BOT_ID;
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
  String url = DMS_URL_PREFIX + "UpdateDumpDropForBot?botId=" + BOT_ID;
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