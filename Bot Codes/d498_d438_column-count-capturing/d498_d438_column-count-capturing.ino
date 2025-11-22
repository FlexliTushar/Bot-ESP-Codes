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
#define PANEL_LED_BUTTON 7 
#define AFC_PIN_1 19
#define AFC_PIN_2 18
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35

// Constant variables
const String CODE_ID = "d498";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
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

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
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
  { "100011110", "D1" }, { "101110011", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, { "101011010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010011", "D8" }, { "100010101", "D9" }, { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100110100", "D13" }, { "100101100", "D14" }, { "100101110", "D15" }, { "100110001", "D16" }, { "100110010", "D17" }, { "100110011", "D18" }, { "110010100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }, 
{ "100011010", "S1"} };

unordered_map<string, string> STATION_ID_TO_EXPECTED_STATION_ID_MAP = {
  { "D1", "S1" }, { "S1", "D2" }, { "D2", "D3" }, { "D3", "D4" }, { "D4", "D5" }, { "D5", "D6" }, { "D6", "D7" }, { "D7", "D8" }, { "D8", "D9" }, { "D9", "D10" }, { "D10", "D11" }, { "D11", "D12" }, { "D12", "D13" }, { "D13", "D14" }, { "D14", "D15" }, { "D15", "D16" }, { "D16", "D17" }, { "D17", "D18" }, { "D18", "D1" }, { "I1", "I12" }, { "I11", "D2" }, { "I12", "I11" }
};

unordered_map<string, int> STATION_ID_TO_SPEED_MAP = {
  { "S1", S2 }, { "D1", S1 }, { "D2", S2 }, { "D3", S2 }, { "D4", S2 }, { "D5", S2 }, { "D6", S2 }, { "D7", S2 }, { "D8", S2 }, { "D9", S2 }, { "D10", S2 }, { "D11", S2 }, { "D12", S2 }, { "D13", S2 }, { "D14", S2 }, { "D15", S2 }, { "D16", S2 }, { "D17", S2 }, { "D18", S2 }, { "I1", S0_3 }, { "I11", S1 }, { "I12", S1 }
};

Preferences prefs;
int errorCode = -1;
bool errorMode = false;
String expectedStation = "X";
bool localisationFlag = true;

// Infeed stop variables
int _intermediateColumnFramesCount = 0;
int _instantaneousFramesCount = 0;
uint16_t _instantaneousIntensityMaxArray[3] = { 0 };
uint16_t _instantaneousIntensityMinArray[3] = { 1023 };
unsigned long stationReadStartTime;
int columnNo = 0;

int GetSetSpeedAsPerStation(const String &stationId) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationId.c_str());
  if (STATION_ID_TO_SPEED_MAP.find(stationCodeString) != STATION_ID_TO_SPEED_MAP.end()) {
    int speed = STATION_ID_TO_SPEED_MAP.at(stationCodeString);
    return speed;
  }
  return S0_5;
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
      if (_intermediateColumnFramesCount >= 5) {
        _finalColumnCodeArray[2] = _finalColumnCodeArray[1];
        _finalColumnCodeArray[1] = _finalColumnCodeArray[0];
        _finalColumnCodeArray[0] = _previousIntermediateColumnCode;
        columnNo++;
        add_log("Col No.: " + String(columnNo));
        if (columnNo > 3) add_log("Bot is reading more than 3 columns");
      } else {
        add_log("Potential fluke column. No. of frames: " + String(_previousIntermediateColumnCode));
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

void ErrorHandling() {
  if (errorCode == 3) {
    add_log("ER3: UNEXPECTED_STATION_ERROR");
    add_log("Expected Station: " + expectedStation + " Detected Station: " + currentStation);
    Beep();
    while(mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
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
      delay(1000);
      setTagetVelocity(GetSetSpeedAsPerStation(currentStation));
      expectedStation = GetExpectedStationFromStationId(currentStation);
    } else {
      errorMode = true;
      errorCode = 4;
    }
  }
  
  if (errorCode == 4) {
    add_log("ER4: INVALID_STATION_ERROR");
    Beep();
    while(mcp.digitalRead(PANEL_LED_BUTTON)) {
      // Until panel button is pressed
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
      delay(1000);
      setTagetVelocity(GetSetSpeedAsPerStation(currentStation));
      expectedStation = GetExpectedStationFromStationId(currentStation);
    } else {
      add_log("Two consecutive invalid station found - Please check the station codes in the system.");
    }
  }
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
  CANConfig();
  MotorConfig();
  LoadBotID();
  Beep();
  delay(2000);
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  printFileName();
  while(mcp.digitalRead(PANEL_LED_BUTTON)){}
  setTagetVelocity(S2);
  add_log("Starting the bot!");
}

String prevCombo = "00";
int count00 = 0;
int count01 = 0;
int count10 = 0;
int count11 = 0;
int columnNo00 = 1;
int columnNo01 = 1;
int columnNo10 = 1;
int columnNo11 = 1;

void loop() {
  if (errorMode) {
    haltMovement();
    ErrorHandling();
    errorMode = false;
    errorCode = -1;
  }
  
  ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

  String stationCodeMain = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

  if (stationCodeMain != "" && stationCodeMain.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    add_log("Current station: " + currentStation);
    if (CheckStationValidity(stationCodeMain)) {
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

  int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
  int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);

  String combo = String(trafficDetected) + String(columnDetected);

  if (combo != prevCombo) {
    if (prevCombo == "00") {
      add_log("Combo: " + prevCombo + ", Column No.: " + String(columnNo00++) + ", FC: " + String(count00));
    } else if (prevCombo == "01") {
      add_log("Combo: " + prevCombo + ", Column No.: " + String(columnNo01++) + ", FC: " + String(count01));
    } else if (prevCombo == "10") {
      add_log("Combo: " + prevCombo + ", Column No.: " + String(columnNo10++) + ", FC: " + String(count10));
    } else {
      add_log("Combo: " + prevCombo + ", Column No.: " + String(columnNo11++) + ", FC: " + String(count11));
    }
    count00 = 0;
    count01 = 0;
    count10 = 0;
    count11 = 0;
  }

  prevCombo = combo;

  if (combo == "00") {
    count00++;
  } else if (combo == "01") {
    count01++;
  } else if (combo == "10") {
    count10++;
  } else {
    count11++;
  }
}

void printFileName() {
  Serial.println(__FILE__);
}

// Function to load values from NVM
void LoadBotID() {
  prefs.begin("diverter", true);
  BOT_ID = prefs.getString("BotId", "Bx");
  prefs.end();
  Serial.println("Diverter config loaded from NVM.");
}

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
  }
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
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT_PULLUP);
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT_PULLUP);
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

// Function to make ahardware beep for 250 ms
void Beep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}