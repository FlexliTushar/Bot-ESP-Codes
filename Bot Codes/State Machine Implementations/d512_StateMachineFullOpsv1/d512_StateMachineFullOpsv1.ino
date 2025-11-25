#define FIRMWARE_ID "d512"

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

// ==============================================================================
// MOTOR SPEED LEVELS
// ==============================================================================
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
#define ACCELERATION_RATE 800  // 800 rpm/s
#define DECELERATION_RATE 800  // 800 rpm/s

// ==============================================================================
// GPIO PIN ASSIGNMENTS
// ==============================================================================
#define SR_D1 36                      // ESP32 pin attached to Photo diode 1 of station reader
#define SR_D2 39                      // ESP32 pin attached to Photo diode 2 of station reader
#define SR_D3 34                      // ESP32 pin attached to Photo diode 3 of station reader
#define BUZZER 13                     // ESP32 pin attached to Buzzer
#define EMG_PIN 12                    // ESP32 pin attached to Electromagnets
#define PANEL_LED_PIN 8               // ESP32 pin attached to Panel LED
#define COLUMN_INDICATOR_SENSOR 33    // ESP32 pin attached to Column Indicator Sensor
#define TRAFFIC_INDICATOR_SENSOR 35   // ESP32 pin attached to Traffic Indicator Sensor
#define PANEL_LED_BUTTON 7            // ESP32 pin attached to Panel LED Button
#define FRONT_SERVO 2                 // ESP32 pin attached to Front Servo of diverter
#define REAR_SERVO 3                  // ESP32 pin attached to Rear Servo of diverter
#define AFC_PIN_1 19                  // ESP32 pin attached to AFC Control Pin 1
#define AFC_PIN_2 18                  // ESP32 pin attached to AFC Control Pin 2

// ==============================================================================
// MISCELLANEOUS MACROS
// ==============================================================================
#define ENABLE 1
#define DISABLE 0

// ==============================================================================
// ENUMS FOR STATE MACHINES
// ==============================================================================

// Health monitoring
enum HealthStatus {
  SAFE = 1,             // System is in a safe state
  WARNING = 0,          // System is in a warning state
  CRITICAL = -1         // System is in a critical state
};

// WTM column detector state machine
enum WTMReaderState {
  WTM_BETWEEN_COLUMNS = 0,  // In 00 zone between columns
  WTM_IN_COLUMN = 1         // On a column (01, 10, or 11)
};

// Drive motor direction
enum DriveMotorDirection {
  DRIVE_MOTOR_FORWARD = 1,  // Forward direction
  DRIVE_MOTOR_REVERSE = -1  // Reverse direction
};

// Drive motor state transition output
enum DriveMotorStateOutput {
  DRIVE_MOTOR_STATE_NO_CHANGE = 0,  // No change in state
  DRIVE_MOTOR_STATE_CHANGED = 1,    // State has changed
  DRIVE_MOTOR_UNKNOWN_STATE = 2     // Unknown state
};

// Drive motor state machine
enum DriveMotorState {
  DRIVE_MOTOR_STOP = 1,             // Motor is stopped
  DRIVE_MOTOR_STOPPING = 2,         // Motor is in the process of stopping
  DRIVE_MOTOR_RUNNING = 3,          // Motor is running
  DRIVE_MOTOR_SWEEPING_COLUMN = 4,  // Motor is sweeping a column
  DRIVE_MOTOR_ERROR = 5             // Motor is in an error state
};

// ==============================================================================
// DIVERTER STATE MACHINE DEFINITIONS
// ==============================================================================

enum DiverterState : uint8_t {
    DIVERTER_LEFT = 1,
    DIVERTER_RIGHT = 2,
    DIVERTER_SWITCHING = 3,
    DIVERTER_ERROR = 4,
    DIVERTER_STOP = 5
};

enum DiverterDirection : uint8_t {
    DIVERTER_DIRECTION_LEFT = 1,
    DIVERTER_DIRECTION_RIGHT = 2,
    DIVERTER_DIRECTION_UNKNOWN = 0
};

enum DiverterPosition : uint8_t {
    DIVERTER_POSITION_LEFT = 1,
    DIVERTER_POSITION_RIGHT = 2,
    DIVERTER_POSITION_INTERIM = 3,
    DIVERTER_POSITION_UNKNOWN = 0
};

enum DiverterStateChangeOptions : uint8_t {
    DIVERTER_STATE_NO_CHANGE = 0,
    DIVERTER_STATE_CHANGED = 1,
    DIVERTER_UNEXPECTED_CONDITION = 2
};

// ==============================================================================
// GLOBAL CONSTANTS
// ==============================================================================

// WiFi Credentials
const char* SSID = "Server_PC";         // WiFi SSID
const char* PASSWORD = "msort@flexli";  // WiFi Password

// Station reader constants
const static uint16_t PT_UPPER_THR_LIMIT = 600;  // Threshold limit for rising photo diode reading.
const static uint16_t PT_LOWER_THR_LIMIT = 400;  // Threshold limit for falling photo diode reading

// WTM reader constants
const float MAJORITY_SAFE_THRESHOLD = 80.0;      // > 80% is safe
const float MAJORITY_WARNING_THRESHOLD = 60.0;   // 60-80% is warning, < 60% is critical
const float MINORITY_SAFE_THRESHOLD = 20.0;      // < 20% is safe
const float MINORITY_WARNING_THRESHOLD = 40.0;   // 20-40% is warning, > 40% is critical
const int DEFAULT_COLUMN_FRAME_THRESHOLD = 5;    // Conservative threshold for column width
const int MIN_COLUMN_FRAMES_THRESHOLD = 3;       // Minimum frames to consider valid column (filter noise)
const int DEFAULT_EMPTY_SPACE_THRESHOLD = 30;    // Expected minimum frames for empty space (4cm at 2m/s ~20ms = 20-40 frames)
const float EMPTY_SPACE_NOISE_THRESHOLD = 5.0;   // < 5% non-00 readings is safe for empty space

// Diverter constants
const int MAX_REATTEMPTS_FOR_SERVO_COM = 5;      // Max reattempts for diverter servo communication

// Drive motor constants
const int DRIVE_MOTOR_SWEEP_SPEED = S0_05;       // Sweep speed of the drive motor

Preferences prefs;
Adafruit_MCP23X17 mcp;

// ==============================================================================
// SYSTEM RESOURCE MONITORING VARIABLES
// ==============================================================================

unsigned long maxLoopTime;                // Maximum loop execution time observed
unsigned long loopStartTime;              // Start time of the current loop iteration
unsigned long maxLatency = 0;             // Maximum loop function latency observed
size_t maxMemoryUsed = 0;                 // Maximum memory used observed

// ==============================================================================
// LOGGER VARIABLES
// ==============================================================================

String BOT_ID = "B";                                                    // Bot identifier
String debugLoggingString = "";                                         // Accumulated debug log string
HTTPClient httpDebugger;                                                // HTTP client for debugging
TaskHandle_t httpDebugLog;                                              // Task handle for HTTP debug logging
bool loggerFlag = true;                                                 // Flag to enable/disable logging
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";  // URL for HTTP debug server

// ==============================================================================
// WTM READER VARIABLES
// ==============================================================================

WTMReaderState wtm_column_state = WTM_BETWEEN_COLUMNS;         // WTM column detection current state
// Frame counts for each possible column state when IN_COLUMN
int frameCount00 = 0;
int frameCount01 = 0;
int frameCount10 = 0;
int frameCount11 = 0;
int exitConfirmationCount = 0;                // Count of consecutive 00 samples
const int EXIT_CONFIRMATION_THRESHOLD = 10;   // Need 10 consecutive 00s to confirm exit
int totalFramesBetweenSpaces = 0;
// WTM column tracking 
String detectedColumnCode = "";               // The actual column code (01, 10, or 11)
String currentColumnCode = "";                // Current column code being read
HealthStatus majorityHealthWTM = SAFE;        // Majority's health status of the WTM column
HealthStatus minorityHealthWTM = SAFE;        // Minority's health status of the WTM column
HealthStatus overallHealthWTM = SAFE;         // Overall health status of the WTM column
// Empty space tracking
int emptySpaceFrameCount = 0;                          // Frame count for current empty space (pure 00s)
int emptySpaceNoiseFrameCount = 0;                     // Count of non-00 frames detected during empty space (for health)
int emptySpaceTotalFrameCount = 0;                     // Total frames in empty space including noise
HealthStatus emptySpaceHealthWTM = SAFE;               // Health of empty space (should have minimal non-00 readings)
HealthStatus emptySpaceMajorityHealthWTM = SAFE;       // Majority health (00 frames)
HealthStatus emptySpaceMinorityHealthWTM = SAFE;       // Minority health (noise frames)
int totalEmptySpacesDetectedWTM = 0;                   // Total empty spaces detected
int totalWTMColumnsDetected = 0;                       // Total WTM columns detected
// Column completion flag and data - set when column successfully read
bool wtmColumnJustCompleted = false;               // Flag: true when column just finished reading
String lastCompletedWTMColumnCode = "";            // The column code that was just read (01, 10, 11)
HealthStatus lastCompletedWTMColumnHealth = SAFE;  // Health of the column that was just read
// Tracking variables for station-to-station segment
String previousStation = "X";
int uniqueColumnsDetectedInSegment = 0;             // Count of unique columns (01, 10, 11) between stations
int emptySpacesDetectedInSegment = 0;               // Count of 00 zones between stations
// Tracking permission because of traffic
bool traffic_permission = true;                     // true: if lastCompletedWTMColumnCode == "11", false: if lastCompletedWTMColumnCode == "01"

// ==============================================================================
// ERROR MODE VARIABLES
// ==============================================================================
bool global_error_status = false;           // false: normal operation, true: enter error mode
volatile bool firstTimeError = false;       // Flag to indicate first time entering error mode

// ==============================================================================
// PICK/DROP ASSIGNMENT VARIABLES
// ==============================================================================
bool current_assignment = true;             // true: pick/drop journey, false: maintenance  

// ==============================================================================
// DRIVE MOTOR VARIABLES
// ==============================================================================
int permitted_edge_speed = 0;                                             // Permitted speed on the current edge
bool column_detected_in_sweep = false;                                    // true if a column was detected during sweep
int drive_motor_current_speed = 0;                                        // Current speed of the drive motor - maxon feedback
int drive_motor_previous_set_speed = 0;                                   // Previous set speed of the drive motor
DriveMotorState drive_motor_current_state = DRIVE_MOTOR_STOP;             // Current state of the drive motor
DriveMotorState drive_motor_previous_state = DRIVE_MOTOR_STOP;            // Previous state of the drive motor
DriveMotorDirection drive_motor_current_direction = DRIVE_MOTOR_FORWARD;  // Current direction of the drive motor
int drive_motor_set_speed = 0;                                            // Set speed of the drive motor
int drive_motor_direction_value = 1;                                      // Direction value for motor control - 1: forward, -1: reverse
bool drive_motor_error_status = false;                                    // Drive motor error status

// ==============================================================================
// DIVERTER VARIABLES
// ==============================================================================

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


// ==============================================================================
// STATION READER VARIABLES
// ==============================================================================
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
  { "100011110", "D1" }, { "101110011", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, { "101011010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010011", "D8" }, { "100010101", "D9" }, { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100110100", "D13" }, { "100101100", "D14" }, { "100101110", "D15" }, { "100110001", "D16" }, { "100110010", "D17" }, { "100110011", "D18" }, { "110010100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }
};

unordered_map<string, bool> STATION_ID_TO_DIRECTION_MAP = {
  { "D1", 0 }, { "D2", 0 }, { "D3", 1 }, { "D4", 1 }, { "D5", 1 }, { "D6", 1 }, { "D7", 1 }, { "D8", 1 }, { "D9", 1 }, { "D10", 1 }, { "D11", 1 }, { "D12", 1 }, { "D13", 1 }, { "D14", 1 }, { "D15", 1 }, { "D16", 1 }, { "D17", 1 }, { "D18", 1 }, { "I1", 1 }, { "I11", 0 }, { "I12", 1 }
};

unordered_map<string, string> STATION_ID_TO_EXPECTED_STATION_ID_MAP = {
  { "D1", "I1" }, { "D2", "D3" }, { "D3", "D4" }, { "D4", "D5" }, { "D5", "D6" }, { "D6", "D7" }, { "D7", "D8" }, { "D8", "D9" }, { "D9", "D10" }, { "D10", "D11" }, { "D11", "D12" }, { "D12", "D13" }, { "D13", "D14" }, { "D14", "D15" }, { "D15", "D16" }, { "D16", "D17" }, { "D17", "D18" }, { "D18", "D1" }, { "I1", "I12" }, { "I11", "D2" }, { "I12", "I11" }
};

unordered_map<string, int> STATION_ID_TO_SPEED_MAP = {
  { "D1", S1 }, { "D2", S2 }, { "D3", S2 }, { "D4", S2 }, { "D5", S2 }, { "D6", S2 }, { "D7", S2 }, { "D8", S2 }, { "D9", S2 }, { "D10", S2 }, { "D11", S2 }, { "D12", S2 }, { "D13", S2 }, { "D14", S2 }, { "D15", S2 }, { "D16", S2 }, { "D17", S2 }, { "D18", S2 }, { "I1", S0_3 }, { "I11", S1 }, { "I12", S0_05 }
};

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

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// Convert enum to string functions
String DiverterStateToString(DiverterState state) {
  switch(state) {
    case DIVERTER_LEFT: return "LEFT";
    case DIVERTER_RIGHT: return "RIGHT";
    case DIVERTER_SWITCHING: return "SWITCHING";
    case DIVERTER_ERROR: return "ERROR";
    case DIVERTER_STOP: return "STOP";
    default: return "UNKNOWN";
  }
}

String DiverterDirectionToString(DiverterDirection dir) {
  switch(dir) {
    case DIVERTER_DIRECTION_LEFT: return "LEFT";
    case DIVERTER_DIRECTION_RIGHT: return "RIGHT";
    case DIVERTER_DIRECTION_UNKNOWN: return "UNKNOWN";
    default: return "UNKNOWN";
  }
}

String DiverterPositionToString(DiverterPosition pos) {
  switch(pos) {
    case DIVERTER_POSITION_LEFT: return "LEFT";
    case DIVERTER_POSITION_RIGHT: return "RIGHT";
    case DIVERTER_POSITION_INTERIM: return "INTERIM";
    case DIVERTER_POSITION_UNKNOWN: return "UNKNOWN";
    default: return "UNKNOWN";
  }
}

// Update high-level position based on current encoder position
void UpdateDiverterPositionHighLevel() {
  if (front_diverter_current_position >= (front_diverter_right_thresold - front_diverter_tolerance) && front_diverter_current_position <= front_diverter_right_limit &&
  rear_diverter_current_position <= (rear_diverter_right_thresold + rear_diverter_tolerance) && rear_diverter_current_position >= rear_diverter_right_limit) {

    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_RIGHT;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  else if (front_diverter_current_position >= front_diverter_left_limit && front_diverter_current_position <= (front_diverter_left_thresold + front_diverter_tolerance)  &&
  rear_diverter_current_position <= rear_diverter_left_limit && rear_diverter_current_position >= (rear_diverter_left_thresold - rear_diverter_tolerance)) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_LEFT;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  else if (front_diverter_current_position < (front_diverter_right_thresold - front_diverter_tolerance) && front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance) &&
  rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance) && rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_INTERIM;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  else {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }
}

// ==============================================================================
// STATE UPDATE FUNCTION
// ==============================================================================

DiverterStateChangeOptions UpdateDiverterState() {
  /*
  Evaluates global variables and determines if state should change
  Updates Current_State based on conditions and valid transitions
  Returns: state change status
  */
  diverter_previous_state = diverter_current_state;
  
  // ----- FROM LEFT STATE -----
  if (diverter_current_state == DIVERTER_LEFT) {
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to STOP_DIVERTER
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to SWITCHING
    if (error_diverter_status == false && 
      global_error_status == false && 
      (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT ||
      current_diverter_position_high_level != DIVERTER_POSITION_LEFT)) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in LEFT
    diverter_current_state = DIVERTER_LEFT;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM RIGHT STATE -----
  else if (diverter_current_state == DIVERTER_RIGHT) {
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
      
    // Transition to STOP_DIVERTER
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to SWITCHING
    if (error_diverter_status == false && 
      global_error_status == false && 
      (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT ||
      current_diverter_position_high_level != DIVERTER_POSITION_RIGHT)) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in RIGHT
    diverter_current_state = DIVERTER_RIGHT;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM SWITCHING STATE -----
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    // add_log("Track edge direction: " + DiverterDirectionToString(diverter_track_edge_direction) + ", pos high level: " + DiverterPositionToString(current_diverter_position_high_level));
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to STOP_DIVERTER
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to LEFT
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT && 
      current_diverter_position_high_level == DIVERTER_POSITION_LEFT) {
      diverter_current_state = DIVERTER_LEFT;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to RIGHT
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT && 
      current_diverter_position_high_level == DIVERTER_POSITION_RIGHT) {
      diverter_current_state = DIVERTER_RIGHT;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Continue switching - remain in SWITCHING
    diverter_current_state = DIVERTER_SWITCHING;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM STOP_DIVERTER STATE -----
  else if (diverter_current_state == DIVERTER_STOP) {
    // Transition to ERROR_DIVERTER
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Transition to SWITCHING
    if (error_diverter_status == false && 
      global_error_status == false) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in STOP_DIVERTER
    diverter_current_state = DIVERTER_STOP;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM ERROR_DIVERTER STATE -----
  else if (diverter_current_state == DIVERTER_ERROR) {
    // Transition to STOP_DIVERTER
    if (error_diverter_status == false) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    // Remain in ERROR_DIVERTER
    diverter_current_state = DIVERTER_ERROR;
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  return DIVERTER_UNEXPECTED_CONDITION;
}

// ==============================================================================
// STATE IMPLEMENTER / ACTUATOR
// ==============================================================================

void UpdateDiverterActuatorInputVariables() {
  /*
  Executes the actions required for the current state
  */
  
  if (diverter_current_state == DIVERTER_LEFT) {
    // ===== LEFT STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter in " + DiverterStateToString(DIVERTER_LEFT) + " position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_RIGHT) {
    // ===== RIGHT STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter in " + DiverterStateToString(DIVERTER_RIGHT) + " position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    // ===== SWITCHING STATE =====
    diverter_set_torque_flag = true;  // Enable torque for movement
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_left_thresold;
      rear_diverter_set_position_value = rear_diverter_left_thresold;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      add_log("Switching to " + DiverterDirectionToString(DIVERTER_DIRECTION_LEFT) + " - moving to front threshold: " + String(front_diverter_left_thresold) + " and rear threshold: " + String(rear_diverter_left_thresold));
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_right_thresold;
      rear_diverter_set_position_value = rear_diverter_right_thresold;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      add_log("Switching to " + DiverterDirectionToString(DIVERTER_DIRECTION_RIGHT) + " - moving to front threshold: " + String(front_diverter_right_thresold) + " and rear threshold: " + String(rear_diverter_right_thresold));
    }
  }
  
  else if (diverter_current_state == DIVERTER_ERROR) {
    // ===== ERROR_DIVERTER STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter error detected! Torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_STOP) {
    // ===== STOP_DIVERTER STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter stopped - torque disabled.");
  }
}

DriveMotorStateOutput UpdateDriveMotorState() {
    /*
    Evaluates global variables and determines if state should change
    Updates Current_State based on conditions and valid transitions
    Returns: true if state was changed, false if no change
    */
    drive_motor_previous_state = drive_motor_current_state;    
    // ----- FROM STOP STATE -----
    if (drive_motor_current_state == DRIVE_MOTOR_STOP) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to RUNNING
        if (current_assignment == true && traffic_permission == true && global_error_permission == true) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to SWEEPING_COLUMN
        if (current_assignment == true && drive_motor_current_speed == 0 && traffic_permission == false && global_error_permission == true) {
            drive_motor_current_state = DRIVE_MOTOR_SWEEPING_COLUMN;
            column_detected_in_sweep = false;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // No valid transition condition met and remain in STOP
        drive_motor_current_state = DRIVE_MOTOR_STOP;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM STOPPING STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOP (when vehicle has stopped)
        if (drive_motor_current_speed == 0) {
            drive_motor_current_state = DRIVE_MOTOR_STOP;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to RUNNING (if assignment and permission restored)
        if (current_assignment == true && traffic_permission == true && global_error_permission == true) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Still stopping - remain in STOPPING
        drive_motor_current_state = DRIVE_MOTOR_STOPPING;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM RUNNING STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOPPING (traffic permission removed)
        if (global_error_permission == false || (current_assignment == true && traffic_permission == false)) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Continue moving - remain in RUNNING
        drive_motor_current_state = DRIVE_MOTOR_RUNNING;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM SWEEPING_COLUMN STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
        // Transition to ERROR_MOTOR
        if (drive_motor_error_status == true) {
            drive_motor_current_state = DRIVE_MOTOR_ERROR;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOPPING (column crossed)
        if (global_error_permission == false || (current_assignment == true && traffic_permission == false && column_detected_in_sweep == true)) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to RUNNING (if traffic permission granted)
        if (current_assignment == true && traffic_permission == true && global_error_permission == true) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Continue sweeping - remain in SWEEPING_COLUMN
        drive_motor_current_state = DRIVE_MOTOR_SWEEPING_COLUMN;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM ERROR_MOTOR STATE -----
    else if (drive_motor_current_state == DRIVE_MOTOR_ERROR) {
        // Transition to STOP (error cleared and vehicle stopped)
        if (drive_motor_error_status == false && drive_motor_current_speed == 0) {
            drive_motor_current_state = DRIVE_MOTOR_STOP;
            return DRIVE_MOTOR_STATE_CHANGED;
        }

        // Transition to STOPPING (error cleared but vehicle still moving)
        if (drive_motor_error_status == false && drive_motor_current_speed > 0) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Still in error state - remain in ERROR_MOTOR
        drive_motor_current_state = DRIVE_MOTOR_ERROR;
        return DRIVE_MOTOR_STATE_NO_CHANGE;
    }
    
    return DRIVE_MOTOR_UNKNOWN_STATE;  // No state change
}

// ==============================================================================
// STATE IMPLEMENTER / ACTUATOR
// ==============================================================================

// Execute state actions when update_state() returns true
void ImplementDriveMotorState() {
    /*
    Executes the actions required for the current state
    */
    
    if (drive_motor_current_state == DRIVE_MOTOR_STOP) {
        // ===== STOP STATE =====
        drive_motor_set_speed = 0;
        add_log("Stopping the bot.");
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
        // ===== STOPPING STATE =====
        drive_motor_set_speed = 0;
        add_log("Stopping the bot.");
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
        // ===== RUNNING STATE =====
        drive_motor_set_speed = permitted_edge_speed;
        drive_motor_current_direction = DRIVE_MOTOR_FORWARD; // Reset current direction
        add_log("Running the bot at speed: " + drive_motor_set_speed);
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
        // ===== SWEEPING COLUMN STATE =====
        
        // Toggle Direction
        if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
            drive_motor_current_direction = DRIVE_MOTOR_REVERSE;
        }
        else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
            drive_motor_current_direction = DRIVE_MOTOR_FORWARD;
        }
        
        // Set Direction Flag
        if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
            drive_motor_direction_value = 1;
        }
        else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
            drive_motor_direction_value = -1;
        }
        
        // Set speed with direction
        drive_motor_set_speed = drive_motor_sweep_speed * drive_motor_direction_value;
        add_log("Sweeping column at speed: " + String(drive_motor_sweep_speed) + " in direction: " + String(drive_motor_direction_value));
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_ERROR) {
        // ===== ERROR_MOTOR STATE =====
        drive_motor_set_speed = 0;
        add_log("Drive motor error detected! Halting the bot.");
    }
}

// ==============================================================================
// RTOS TASKS
// ==============================================================================

// Diverter State Machine Task
void SENSOR_READING_AND_UPDATE_STATE_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);  // 10ms cycle time
    global_error_status = error_diverter_status;
    
    DiverterStateChangeOptions stateChange = UpdateDiverterState();
    if (stateChange != DIVERTER_STATE_NO_CHANGE) Serial.println(stateChange);
    if (stateChange == DIVERTER_STATE_CHANGED) {
      add_log("State changed to: " + DiverterStateToString(diverter_current_state) + " from " + DiverterStateToString(diverter_previous_state));
      add_log("Variables - Global_Error: " + String(global_error_status) + ", Demand_Dir: " + DiverterDirectionToString(diverter_track_edge_direction) + ", Pos_HL: " + DiverterPositionToString(current_diverter_position_high_level) + ", Error_Status: " + String(error_diverter_status));
      add_log("Front current position: " + String(front_diverter_current_position) + ", Rear current position: " + String(rear_diverter_current_position));
      UpdateDiverterActuatorInputVariables();
    } else if (stateChange == DIVERTER_UNEXPECTED_CONDITION) {
      add_log("Diverter State Machine in UNEXPECTED CONDITION!");
    }
  }
}

// Diverter Actuation Task
void DIVERTER_ACTUATION_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Update motor speed if changed
    if (!diverter_set_torque_flag ) {
      if (prev_diverter_set_torque_flag)
      {
        delay(1);
        sm.EnableTorque(FRONT_SERVO, DISABLE);
        if (sm.getLastError()) {
          add_log("Front torque disabling communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        delay(1);
        sm.EnableTorque(REAR_SERVO, DISABLE);
        if (sm.getLastError()) {
          add_log("Rear torque disabling communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }
    } else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      if (front_diverter_set_position_value != -1 && front_diverter_current_position < (front_diverter_right_thresold - front_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(FRONT_SERVO, front_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Front position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        // prev_front_diverter_set_position_value = front_diverter_set_position_value;
      }

      if (rear_diverter_set_position_value != -1 && rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Rear position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        // prev_rear_diverter_set_position_value = rear_diverter_set_position_value;
      }
    } else if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      if (front_diverter_set_position_value != -1 && front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(FRONT_SERVO, front_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Front position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        // prev_front_diverter_set_position_value = front_diverter_set_position_value;
      }

      if (rear_diverter_set_position_value != -1 && rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          add_log("Rear position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        // prev_rear_diverter_set_position_value = rear_diverter_set_position_value;
      }
    }
    prev_diverter_set_torque_flag = diverter_set_torque_flag;
    delay(1);
    front_diverter_current_position = sm.ReadPos(FRONT_SERVO);
    // add_log("Current front diverter position: " + String(front_diverter_current_position));
    if (sm.getLastError()) {
      add_log("Front position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }
    delay(1);
    rear_diverter_current_position = sm.ReadPos(REAR_SERVO);
    // add_log("Current rear diverter position: " + String(rear_diverter_current_position));
    if (sm.getLastError()) {
      add_log("Rear position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }

    UpdateDiverterPositionHighLevel();
  }
}

// Auxiliary Task - Toggles demand direction every 5 seconds
void AUXILIARY_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(500 / portTICK_PERIOD_MS);  // 100ms cycle time
    
    unsigned long current_time = millis();
    
    // Check if 5 seconds have passed and no errors
    if ((current_time - last_toggle_time >= TOGGLE_INTERVAL) && 
      (error_diverter_status == false) && 
      (global_error_status == false)) {
      
      // Toggle demand direction
      if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
        diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;
        add_log("AUX: Toggling demand direction to " + DiverterDirectionToString(DIVERTER_DIRECTION_RIGHT));
      } 
      else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
        diverter_track_edge_direction = DIVERTER_DIRECTION_LEFT;
        add_log("AUX: Toggling demand direction to " + DiverterDirectionToString(DIVERTER_DIRECTION_LEFT));
      }
      else if (diverter_track_edge_direction == DIVERTER_DIRECTION_UNKNOWN) {
        // Initialize to LEFT on first toggle
        diverter_track_edge_direction = DIVERTER_DIRECTION_LEFT;
        add_log("AUX: Initializing demand direction to " + DiverterDirectionToString(DIVERTER_DIRECTION_LEFT));
      }
      
      last_toggle_time = current_time;
    }
  }
}

// ==============================================================================
// SETUP AND INITIALIZATION
// ==============================================================================

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
        2,
        &httpDebugLog,
        0);
    ServoConfig();
    LoadDiverterConfig();
    Beep();
    delay(1000);
    
    Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
    printFileName();
    add_log("Starting diverter test!");
    
    // Create RTOS tasks
    xTaskCreatePinnedToCore(
        DIVERTER_STATE_TASK,
        "diverter_state_task",
        10000,
        NULL,
        5,
        &diverterStateTask,
        1);
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    xTaskCreatePinnedToCore(
        DIVERTER_ACTUATION_TASK,
        "diverter_actuation_task",
        10000,
        NULL,
        4,
        &diverterActuationTask,
        1);
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // xTaskCreatePinnedToCore(
    //     AUXILIARY_TASK,
    //     "auxiliary_task",
    //     10000,
    //     NULL,
    //     3,
    //     &auxiliaryTask,
    //     1);
    
    last_toggle_time = millis();
}

void loop() {
  // Empty loop as tasks handle functionality
}

void printFileName() {
  Serial.println(__FILE__);
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

void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("MCP Error.");
  }
}

void Beep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

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

void add_log(String log) {
  debugLoggingString += " | " + log;
  Serial.println(log);  // Also print to serial for debugging
}

void WiFiConfig() {
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}

void PinConfig() {
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
}

void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
}
