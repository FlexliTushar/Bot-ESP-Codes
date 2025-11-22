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

// Health monitoring
enum HealthStatus {
  SAFE,
  WARNING,
  CRITICAL
};

// Column detection state machine variables
enum ColumnState {
  BETWEEN_COLUMNS,  // In 00 zone between columns
  IN_COLUMN         // On a column (01, 10, or 11)
};

// Constant variables
const String CODE_ID = "d499";
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

// Map to store expected unique columns and empty spaces between station pairs
// Format: "StationA-StationB" -> {expected_columns, expected_spaces}
unordered_map<string, pair<int, int>> STATION_EDGE_EXPECTED_MAP = {
  { "D1-S1", {0, 0} }, { "S1-D2", {0, 0} }, { "D2-D3", {10, 11} }, { "D3-D4", {10, 11} }, 
  { "D4-D5", {10, 11} }, { "D5-D6", {10, 11} }, { "D6-D7", {10, 11} }, { "D7-D8", {10, 11} },
  { "D8-D9", {10, 11} }, { "D9-D10", {10, 11} }, { "D10-D11", {10, 11} }, { "D11-D12", {10, 11} },
  { "D12-D13", {10, 11} }, { "D13-D14", {10, 11} }, { "D14-D15", {10, 11} }, { "D15-D16", {10, 11} },
  { "D16-D17", {10, 11} }, { "D17-D18", {10, 11} }, { "D18-D1", {10, 11} },
  { "I1-I12", {0, 0} }, { "I11-D2", {0, 0} }, { "I12-I11", {0, 0} }
};

ColumnState columnState = BETWEEN_COLUMNS;

// Frame counts for each possible column state
int frameCount00 = 0;
int frameCount01 = 0;
int frameCount10 = 0;
int frameCount11 = 0;

// Confirmation variables
int exitConfirmationCount = 0;  // Count of consecutive 00 samples
const int EXIT_CONFIRMATION_THRESHOLD = 10;  // Need 10 consecutive 00s to confirm exit

// Column tracking
int totalColumnsDetected = 0;
String detectedColumnCode = "";  // The actual column code (01, 10, or 11)
int totalFramesBetweenSpaces = 0;  // Total frames from last empty space to current empty space

// Health monitoring thresholds
const float MAJORITY_SAFE_THRESHOLD = 80.0;      // > 80% is safe
const float MAJORITY_WARNING_THRESHOLD = 60.0;   // 60-80% is warning, < 60% is critical
const float MINORITY_SAFE_THRESHOLD = 20.0;      // < 20% is safe
const float MINORITY_WARNING_THRESHOLD = 40.0;   // 20-40% is warning, > 40% is critical
const int DEFAULT_COLUMN_FRAME_THRESHOLD = 5;    // Conservative threshold for column width
const int MIN_COLUMN_FRAMES_THRESHOLD = 3;       // Minimum frames to consider valid column (filter noise)
const int DEFAULT_EMPTY_SPACE_THRESHOLD = 30;    // Expected minimum frames for empty space (4cm at 2m/s ~20ms = 20-40 frames)
const float EMPTY_SPACE_NOISE_THRESHOLD = 5.0;   // < 5% non-00 readings is safe for empty space

HealthStatus majorityHealth = SAFE;
HealthStatus minorityHealth = SAFE;
HealthStatus overallHealth = SAFE;

// Empty space tracking for health monitoring
int emptySpaceFrameCount = 0;        // Frame count for current empty space (pure 00s)
int emptySpaceNoiseFrameCount = 0;   // Count of non-00 frames detected during empty space (for health)
int emptySpaceTotalFrameCount = 0;   // Total frames in empty space including noise
HealthStatus emptySpaceHealth = SAFE; // Health of empty space (should have minimal non-00 readings)
HealthStatus emptySpaceMajorityHealth = SAFE;  // Majority health (00 frames)
HealthStatus emptySpaceMinorityHealth = SAFE;  // Minority health (noise frames)
int totalEmptySpacesDetected = 0;    // Total empty spaces detected

// Column completion flag and data - set when column successfully read
bool columnJustCompleted = false;      // Flag: true when column just finished reading
String lastCompletedColumnCode = "";   // The column code that was just read (01, 10, 11)
HealthStatus lastCompletedColumnHealth = SAFE;  // Health of the column that was just read

// Tracking variables for station-to-station segment
String previousStation = "X";
int uniqueColumnsDetectedInSegment = 0;  // Count of unique columns (01, 10, 11) between stations
int emptySpacesDetectedInSegment = 0;    // Count of 00 zones between stations

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
        // add_log("Potential fluke column. No. of frames: " + String(_previousIntermediateColumnCode));
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

String GetExpectedStationFromStationId(const String &stationCode) {
  // Check if the station code is present in the StationCodeToId map
  string stationCodeString = string(stationCode.c_str());
  if (STATION_ID_TO_EXPECTED_STATION_ID_MAP.find(stationCodeString) != STATION_ID_TO_EXPECTED_STATION_ID_MAP.end()) {
    string stationID = STATION_ID_TO_EXPECTED_STATION_ID_MAP.at(stationCodeString);
    return String(stationID.c_str());
  }
  return "";
}

// Get expected unique columns and empty spaces for a station-to-station edge
// Returns pair<expectedColumns, expectedSpaces>. Returns {-1, -1} if edge not found.
pair<int, int> GetExpectedSegmentData(const String &fromStation, const String &toStation) {
  String edgeKey = fromStation + "-" + toStation;
  string edgeKeyStr = string(edgeKey.c_str());
  
  if (STATION_EDGE_EXPECTED_MAP.find(edgeKeyStr) != STATION_EDGE_EXPECTED_MAP.end()) {
    return STATION_EDGE_EXPECTED_MAP.at(edgeKeyStr);
  }
  return make_pair(-1, -1);  // Return -1, -1 if edge not found
}

// Validate and log the station-to-station segment analysis
void ValidateStationSegment(const String &fromStation, const String &toStation, int actualColumns, int actualSpaces) {
  auto expected = GetExpectedSegmentData(fromStation, toStation);
  int expectedColumns = expected.first;
  int expectedSpaces = expected.second;
  
  if (expectedColumns == -1) {
    add_log("Segment " + fromStation + "-" + toStation + ": No validation data available");
    return;
  }
  
  // Log segment analysis
  add_log("Segment Analysis :" + fromStation + "-" + toStation);
  add_log("Unique Columns: " + String(actualColumns) + " (Expected: " + String(expectedColumns) + ")");
  add_log("Empty Spaces: " + String(actualSpaces) + " (Expected: " + String(expectedSpaces) + ")");
  
  // Check column count mismatch
  if (actualColumns != expectedColumns) {
    add_log("WARNING: Column count mismatch! Difference: " + String(actualColumns - expectedColumns));
    if (actualColumns < expectedColumns) {
      add_log("Possible Issue: Missing columns detected or bot skipped columns");
    } else {
      add_log("Possible Issue: Extra columns detected or noise interference");
    }
  }
  
  // Check empty space count mismatch
  if (actualSpaces != expectedSpaces) {
    add_log("WARNING: Empty space count mismatch! Difference: " + String(actualSpaces - expectedSpaces));
    if (actualSpaces < expectedSpaces) {
      add_log("Possible Issue: Spaces not detected or continuous column reading");
    } else {
      add_log("Possible Issue: Extra spaces detected or column detection failure");
    }
  }
  
  // Log success if everything matches
  if (actualColumns == expectedColumns && actualSpaces == expectedSpaces) {
    add_log("Status: SEGMENT OK - All columns and spaces match expected values");
  }
}

// Function to read TI and CI sensor combo
String ReadSensorCombo() {
  int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
  int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);
  return String(trafficDetected) + String(columnDetected);
}

// Function to handle column detection and processing
void ProcessColumnDetection(String combo) {
  // Column detection state machine
  switch (columnState) {
    case BETWEEN_COLUMNS:
      // We're in the 00 zone between columns
      if (combo == "00") {
        emptySpaceFrameCount++;
      } else {
        // Detected non-00 in empty space - transition to column
        columnState = IN_COLUMN;
        totalFramesBetweenSpaces = 0;
        emptySpaceNoiseFrameCount = 0;
        frameCount00 = 0;
        frameCount01 = 0;
        frameCount10 = 0;
        frameCount11 = 0;
        exitConfirmationCount = 0;
        
        // Count the first frame that triggered the transition
        if (combo == "01") {
          frameCount01 = 1;
          totalFramesBetweenSpaces = 1;
        } else if (combo == "10") {
          frameCount10 = 1;
          totalFramesBetweenSpaces = 1;
        } else if (combo == "11") {
          frameCount11 = 1;
          totalFramesBetweenSpaces = 1;
        }
      }
      break;

    case IN_COLUMN:
      // Count all frames by type
      if (combo == "00") {
        frameCount00++;
      } else if (combo == "01") {
        frameCount01++;
        totalFramesBetweenSpaces++;
        if (frameCount01 > max(frameCount10, frameCount11)) {
          detectedColumnCode = "01";
        }
      } else if (combo == "10") {
        frameCount10++;
        totalFramesBetweenSpaces++;
        if (frameCount10 > max(frameCount01, frameCount11)) {
          detectedColumnCode = "10";
        }
      } else if (combo == "11") {
        frameCount11++;
        totalFramesBetweenSpaces++;
        if (frameCount11 > max(frameCount01, frameCount10)) {
          detectedColumnCode = "11";
        }
      }
      
      // Check if we're exiting the column
      if (combo == "00") {
        exitConfirmationCount++;
        if (exitConfirmationCount >= EXIT_CONFIRMATION_THRESHOLD) {
          // Validate minimum frames threshold
          if (totalFramesBetweenSpaces < MIN_COLUMN_FRAMES_THRESHOLD) {
            add_log("Fluke: " + String(totalFramesBetweenSpaces) + "f (01:" + String(frameCount01) + ",10:" + String(frameCount10) + ",11:" + String(frameCount11) + ") Rec:" + String(emptySpaceFrameCount + exitConfirmationCount));
            
            emptySpaceFrameCount += EXIT_CONFIRMATION_THRESHOLD;
            emptySpaceTotalFrameCount = emptySpaceFrameCount + totalFramesBetweenSpaces;
            emptySpaceNoiseFrameCount += totalFramesBetweenSpaces;
            columnState = BETWEEN_COLUMNS;
            return;
          } else {
            emptySpaceTotalFrameCount = emptySpaceFrameCount;
          }
          
          // Valid column detected - calculate empty space health
          if (emptySpaceFrameCount > 0) {
            totalEmptySpacesDetected++;
            emptySpacesDetectedInSegment++;
            
            // Calculate empty space health using majority and minority approach
            emptySpaceMajorityHealth = CalculateMajorityHealth(emptySpaceFrameCount, DEFAULT_EMPTY_SPACE_THRESHOLD);
            emptySpaceMinorityHealth = CalculateMinorityHealth(emptySpaceNoiseFrameCount, emptySpaceTotalFrameCount);
            emptySpaceHealth = GetWorstHealth(emptySpaceMajorityHealth, emptySpaceMinorityHealth);
            
            add_log("TES:" + String(totalEmptySpacesDetected) + " ES" + String(emptySpacesDetectedInSegment) + ":" + String(emptySpaceFrameCount) + "f " + HealthToString(emptySpaceHealth));
            emptySpaceNoiseFrameCount = 0;
          }
          
          totalColumnsDetected++;
          uniqueColumnsDetectedInSegment++;
          
          columnJustCompleted = true;
          lastCompletedColumnCode = detectedColumnCode;
          lastCompletedColumnHealth = AnalyzeColumnHealth();
          
          columnState = BETWEEN_COLUMNS;
          emptySpaceFrameCount = EXIT_CONFIRMATION_THRESHOLD;
        }
      } else {
        exitConfirmationCount = 0;
      }
      break;
  }
}

// Function to handle column-based decisions
void HandleColumnDecisions() {
  if (columnJustCompleted) {
    if (lastCompletedColumnCode == "11") {
      add_log("Traffic Detected");
    } else if (lastCompletedColumnCode == "01") {
      add_log("Traffic Not Detected");
    } else if (lastCompletedColumnCode == "10") {
      add_log("WARNING: Column Indicator malfunction detected! Reading '10' (CI=0, TI=1)");
    }
    
    if (lastCompletedColumnHealth == CRITICAL) {
      // Column health is critical - may need maintenance
    }
    
    columnJustCompleted = false;
  }
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
  add_log("Starting the bot for test!");
}

void loop() {
  if (errorMode) {
    haltMovement();
    ErrorHandling();
    errorMode = false;
    errorCode = -1;
  }
  
  ReadStationViaPhotoDiode();
  String stationCodeMain = GetStationCodeFromDataReadByStationSensor();

  if (stationCodeMain != "" && stationCodeMain.length() == 9) {
    currentStation = TransformStationCodeIntoStationId(stationCodeMain);
    add_log("Current station: " + currentStation);
    
    if (previousStation != "X" && currentStation != "X") {
      ValidateStationSegment(previousStation, currentStation, 
                           uniqueColumnsDetectedInSegment, emptySpacesDetectedInSegment);
    }
    
    if (CheckStationValidity(stationCodeMain)) {
      if (!localisationFlag) {
        if (expectedStation != currentStation) {
          errorCode = 3;
          errorMode = true;
        }
      } else {
        localisationFlag = false;
      }

      if (!errorMode) {
        expectedStation = GetExpectedStationFromStationId(currentStation);
        previousStation = currentStation;
        uniqueColumnsDetectedInSegment = 0;
        emptySpacesDetectedInSegment = 0;
      }
    } else {
      add_log("Invalid station code: " + stationCodeMain);
      errorCode = 4;
      errorMode = true;
    }
  }

  // Read TI/CI sensors and process column detection
  String combo = ReadSensorCombo();
  ProcessColumnDetection(combo);
  HandleColumnDecisions();
}

// Calculate health status based on majority frame count
HealthStatus CalculateMajorityHealth(int majorityCount, int threshold) {
  if (threshold == 0) return SAFE;
  
  float percentage = (float)majorityCount / threshold * 100.0;
  
  if (percentage > MAJORITY_SAFE_THRESHOLD) {
    return SAFE;
  } else if (percentage >= MAJORITY_WARNING_THRESHOLD) {
    return WARNING;
  } else {
    return CRITICAL;
  }
}

// Calculate health status based on minority frame counts
HealthStatus CalculateMinorityHealth(int minorityTotal, int totalFrames) {
  if (totalFrames == 0) return SAFE;
  
  float percentage = (float)minorityTotal / totalFrames * 100.0;
  
  if (percentage < MINORITY_SAFE_THRESHOLD) {
    return SAFE;
  } else if (percentage <= MINORITY_WARNING_THRESHOLD) {
    return WARNING;
  } else {
    return CRITICAL;
  }
}

// Calculate empty space health using majority (00 frames) and minority (noise frames) approach
HealthStatus CalculateEmptySpaceHealth(int majorityFrames, int noiseFrames, int totalFrames) {
  if (totalFrames == 0) return SAFE;
  
  // Calculate majority health (00 frames should be > 80% of total)
  HealthStatus majorityHealth = CalculateMajorityHealth(majorityFrames, DEFAULT_EMPTY_SPACE_THRESHOLD);
  
  // Calculate minority health (noise frames should be < 20% of total)
  HealthStatus minorityHealth = CalculateMinorityHealth(noiseFrames, totalFrames);
  
  // Overall health is worst of both (AND operation)
  return GetWorstHealth(majorityHealth, minorityHealth);
}

// Get worst health status between two statuses
HealthStatus GetWorstHealth(HealthStatus h1, HealthStatus h2) {
  if (h1 == CRITICAL || h2 == CRITICAL) return CRITICAL;
  if (h1 == WARNING || h2 == WARNING) return WARNING;
  return SAFE;
}

// Convert health status to string
String HealthToString(HealthStatus health) {
  switch(health) {
    case SAFE: return "SAFE";
    case WARNING: return "WARNING";
    case CRITICAL: return "CRITICAL";
    default: return "UNKNOWN";
  }
}

// Analyze column detection and report health
HealthStatus AnalyzeColumnHealth() {
  if (detectedColumnCode == "") return SAFE;
  
  // Total frames during column reading (excluding 00 exit confirmation frames)
  int totalColumnFrames = frameCount01 + frameCount10 + frameCount11;
  int totalFrames = frameCount00 + totalColumnFrames;  // All frames including noise
  
  // Determine majority and minority counts
  int majorityCount = 0;
  int minorityTotal = 0;
  String majorityCode = "";
  
  // Find the majority (should match detected column code)
  if (detectedColumnCode == "01") {
    majorityCount = frameCount01;
    majorityCode = "01";
    minorityTotal = frameCount10 + frameCount11;  // Other column readings (not 00)
  } else if (detectedColumnCode == "10") {
    majorityCount = frameCount10;
    majorityCode = "10";
    minorityTotal = frameCount01 + frameCount11;
  } else if (detectedColumnCode == "11") {
    majorityCount = frameCount11;
    majorityCode = "11";
    minorityTotal = frameCount01 + frameCount10;
  }
  
  // Calculate health
  // Majority: Use static threshold (DEFAULT_COLUMN_FRAME_THRESHOLD or maxCount)
  majorityHealth = CalculateMajorityHealth(majorityCount, DEFAULT_COLUMN_FRAME_THRESHOLD);
  
  // Minority: Use total frames captured from previous empty space to current empty space
  minorityHealth = CalculateMinorityHealth(minorityTotal, totalFramesBetweenSpaces);
  overallHealth = GetWorstHealth(majorityHealth, minorityHealth);
  
  // Compact log for memory efficiency
  add_log("C" + String(totalColumnsDetected) + "[" + detectedColumnCode + "]:" + String(totalFramesBetweenSpaces) + "f(" + String(frameCount01) + "," + String(frameCount10) + "," + String(frameCount11) + ") " + HealthToString(overallHealth));
  return overallHealth;
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