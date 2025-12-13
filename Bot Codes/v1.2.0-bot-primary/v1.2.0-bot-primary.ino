#define FIRMWARE_ID                     "v1.2.0"
// v1.2.0-bot-primary.ino
// Primary bot firmware - Version 1.2.0
// Over v1.1.0 layer, added optimised infeed stopping logic

#include "driver/twai.h"
#include "driver/gpio.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <unordered_map>
#include "esp_system.h"
#include <Adafruit_MCP23X17.h>
#include <Update.h>
#include <ArduinoJson.h>
using namespace std;

//=============================================================================
// DEBUG LOGGING CONFIGURATION
//=============================================================================
#define SSID                        "Server_PC"
#define PASSWORD                    "msort@flexli"

// Debug logging string buffer
String BOT_ID = "Bx";
const String DEBUG_SERVER_URL = "http://192.168.2.109:5000/log";
String debugLoggingString = BOT_ID + " " + FIRMWARE_ID + ": ";
bool loggerFlag = true;  // Enable/disable HTTP debug logging

//=============================================================================
// GPIO PIN DEFINITIONS
//=============================================================================
#define BUZZER                      13
#define COLUMN_INDICATOR_SENSOR     33
#define TRAFFIC_INDICATOR_SENSOR    35
#define PANEL_LED_BUTTON 7
#define EMG_PIN 12
#define AFC_PIN_1 19
#define AFC_PIN_2 18

// Station Reader Photodiode Pins
#define SR_D1                       36
#define SR_D2                       39
#define SR_D3                       34

//=============================================================================
// SERVO CONFIGURATION (MODBUS RTU)
//=============================================================================
#define RX_PIN              25
#define TX_PIN              32
#define BAUDRATE            115200

#define FRONT_SERVO_ID      2
#define REAR_SERVO_ID       3

#define ENABLE              1
#define DISABLE             0

// SMS Servo Register Addresses (Modbus)
#define REG_TORQUE_ENABLE   129
#define REG_GOAL_POSITION   128
#define REG_PRESENT_POS     257

// Modbus Function Codes
#define FC_READ_REGISTERS   3
#define FC_WRITE_SINGLE     6
#define FC_WRITE_MULTIPLE   16

//=============================================================================
// CAN HARDWARE CONFIGURATION
//=============================================================================
#define CAN_TX_PIN                  GPIO_NUM_14
#define CAN_RX_PIN                  GPIO_NUM_15
#define NODE_ID                     1

// CANopen COB-IDs for node 1
#define SDO_TX                      (0x600 + NODE_ID)  // Client->Server (our requests)
#define SDO_RX                      (0x580 + NODE_ID)  // Server->Client (EPOS responses)

//=============================================================================
// EPOS OBJECT DICTIONARY INDICES
//=============================================================================
#define CONTROLWORD                 0x6040
#define OPERATION_MODE              0x6060
#define TARGET_VELOCITY             0x60FF
#define VELOCITY_ACTUAL_VALUE_AVG   0x30D3
#define MAX_PROFILE_VELOCITY        0x607F
#define PROFILE_ACCELERATION        0x6083
#define PROFILE_DECELERATION        0x6084
#define FOLLOWING_ERROR_WINDOW      0x6065

//=============================================================================
// MOTOR CONTROL CONSTANTS
//=============================================================================
// Operation modes
#define PROFILE_VELOCITY_MODE       3

// Controlword values
#define CONTROLWORD_SHUTDOWN        6     // 0x0006 - disable motor
#define CONTROLWORD_ENABLE          15    // 0x000F - enable and start
#define CONTROLWORD_HALT            271   // 0x010F - halt movement (bit 8 set)

// Motor speed levels (RPM)
#define S0_03         15    // 0.03 m/s
#define S0_05         26    // 0.05 m/s
#define S0_1          53    // 0.1 m/s
#define S0_2          106   // 0.2 m/s
#define S0_3          160   // 0.3 m/s
#define S0_5          267   // 0.5 m/s
#define S1            535   // 1.0 m/s
#define S1_5          802   // 1.5 m/s
#define S2            1070  // 2.0 m/s
#define S2_5          1337  // 2.5 m/s
#define S3            1604  // 3.0 m/s

//=============================================================================
// TIMING AND COMMUNICATION CONSTANTS
//=============================================================================
#define SDO_RESPONSE_TIMEOUT        10    // CAN SDO response timeout (ms)
#define MOTOR_POWERUP_DELAY         5000  // Motor controller power-up delay (ms)

//=============================================================================
// MOTOR CONFIGURATION CONSTANTS
//=============================================================================
const uint32_t MAX_VELOCITY_RPM = 1800;           // Maximum profile velocity (RPM)
const uint32_t PROFILE_ACCEL_RPM_S = 800;         // Profile acceleration (RPM/s)
const uint32_t PROFILE_DECEL_RPM_S = 800;         // Profile deceleration (RPM/s)
const uint32_t QUICK_STOP_DECEL_RPM_S = 800;      // Quick stop deceleration (RPM/s)
const uint32_t MOTION_PROFILE_TYPE = 0;           // Trapezoidal motion profile

//=============================================================================
// DIVERTER SERVO CONFIGURATION CONSTANTS
//=============================================================================
const uint16_t DIVERTER_SPEED = 100;              // Diverter servo speed
const uint16_t DIVERTER_ACCELERATION = 0;         // Diverter servo acceleration
const uint8_t DIVERTER_MAX_RETRIES = 3;           // Maximum retry attempts for diverter operations
const uint16_t POSITION_WRITE_COOLDOWN_MS = 300;  // Minimum interval between position writes (ms)

//=============================================================================
// ENUMERATIONS AND TYPE DEFINITIONS
//=============================================================================

// CAN/CANopen operation status codes
enum CANStatus : uint8_t {
  CAN_OK = 0,                   // Successful operation
  CAN_TX_TIMEOUT,               // Transmission timeout
  CAN_RX_TIMEOUT,               // Reception timeout
  CAN_SDO_ABORT,                // SDO abort received
  CAN_ERROR                     // General CAN error
};

// Health monitoring status
enum HealthStatus : uint8_t {
  SAFE,                           // System is healthy
  WARNING,                        // System is in warning state
  CRITICAL                        // System is in critical state
};

// Column detection state machine states
enum WTMColumnState : uint8_t {
  WTM_BETWEEN_COLUMNS,            // In 00 zone between columns
  WTM_IN_COLUMN                   // On a column (01, 10, or 11)
};

// Drive motor state machine states
enum DriveMotorState : uint8_t {
  DRIVE_MOTOR_STOP = 1,             // Motor is stopped
  DRIVE_MOTOR_STOPPING = 2,         // Motor is stopping
  DRIVE_MOTOR_RUNNING = 3,          // Motor is running
  DRIVE_MOTOR_SWEEPING_COLUMN = 4,  // Motor is sweeping column
  DRIVE_MOTOR_ERROR = 5             // Motor error state
};

// Drive motor direction
enum DriveMotorDirection : uint8_t {
  DRIVE_MOTOR_FORWARD = 1,        // Forward direction
  DRIVE_MOTOR_REVERSE = 0         // Reverse direction
};

// State update return codes
enum DriveMotorStateOutput : uint8_t {
  DRIVE_MOTOR_STATE_NO_CHANGE = 0,        // No change in state
  DRIVE_MOTOR_STATE_CHANGED = 1,          // State has changed
  DRIVE_MOTOR_UNKNOWN_STATE = 2           // Unknown state
};

// Diverter state machine states
enum DiverterState : uint8_t {
    DIVERTER_STOP = 0,                  // Diverter is stopped
    DIVERTER_LEFT = 1,                  // Diverter is moving left
    DIVERTER_RIGHT = 2,                 // Diverter is moving right
    DIVERTER_SWITCHING = 3,             // Diverter is switching
    DIVERTER_ERROR = 4                  // Diverter error state
};

// Diverter direction
enum DiverterDirection : uint8_t {
    DIVERTER_DIRECTION_UNKNOWN = 0,        // Unknown direction
    DIVERTER_DIRECTION_LEFT = 1,           // Left direction
    DIVERTER_DIRECTION_RIGHT = 2           // Right direction
};

// Diverter position high-level states
enum DiverterPosition : uint8_t {
    DIVERTER_POSITION_UNKNOWN = 0,        // Unknown position
    DIVERTER_POSITION_LEFT = 1,           // Left position
    DIVERTER_POSITION_RIGHT = 2,          // Right position
    DIVERTER_POSITION_INTERIM = 3         // Interim position
};

// Diverter state change return codes
enum DiverterStateChangeOptions : uint8_t {
    DIVERTER_STATE_NO_CHANGE = 0,          // No change in diverter state
    DIVERTER_STATE_CHANGED = 1,            // Diverter state has changed
    DIVERTER_UNEXPECTED_CONDITION = 2      // Unexpected condition encountered
};

// Simple CAN frame structure (maps to TWAI message)
struct SimpleCANFrame {
  uint32_t id;                              // 11-bit or 29-bit identifier
  uint8_t  dlc;                             // Data length code (0-8)
  uint8_t  data[8];                         // Data bytes
  bool     extended;                        // Extended frame format flag
  bool     rtr;                             // Remote transmission request flag
};

//=============================================================================
// GLOBAL VARIABLES - STATION READER
//=============================================================================

// Station reader sensor state variables
bool photo_transistor_current_state_array[3] = { 0, 0, 0 };       // Array to store current state of sensors
bool photo_transistor_previous_state_array[3] = { 0, 0, 0 };      // Array to store previous state of sensors
uint16_t photo_transistor_current_value_array[3] = { 0, 0, 0 };   // Array to store the analog value sent by the sensor

// Station reading state variables
String intermediate_column_code = "000";                        // String to store intermediate column codes
bool reading_in_process = false;                                // Flag to track if station reading is in progress
String final_column_code_array[3] = { "000", "000", "000" };     // Array to store the 3 columns of the station
bool right_station_detected = false;                            // Flag to check if a station has been detected
String previous_intermediate_column_code = "000";
String station_code = "";                                       // String to store entire stationCode
String current_station = "X";                                   // String to store the current station Id
String expected_station = "X";                                  // String to store the expected station Id
bool localisation_flag = true;                                  // Flag to indicate if localisation has happened or not
bool station_reader_error_status = false;                       // Flag to indicate station reader error status

// Station reading metrics
int intermediate_column_frames_count = 0;                       // Count of frames for the intermediate column
int instantaneous_frames_count = 0;                             // Count of frames for the instantaneous column
uint16_t instantaneous_intensity_max_array[3] = { 0 };          // Max intensity values for the instantaneous column
uint16_t instantaneous_intensity_min_array[3] = { 1023 };       // Min intensity values for the instantaneous column
unsigned long station_read_start_time;                          // Timestamp when station reading started
int column_no = 0;                                              // Column number counter

// Photodiode thresholds
const static uint16_t PT_UPPER_THR_LIMIT = 600;              // Threshold limit for rising photo diode reading
const static uint16_t PT_LOWER_THR_LIMIT = 400;              // Threshold limit for falling photo diode reading

// Station code to station ID mapping
unordered_map<string, string> STATION_CODE_TO_STATION_ID_MAP = {
  { "100011110", "D1" }, { "101110011", "D2" }, { "101110100", "D3" }, { "101011001", "D4" }, 
  { "101011010", "D5" }, { "100011001", "D6" }, { "110101110", "D7" }, { "100010101", "D9" }, 
  { "100101011", "D10" }, { "100101010", "D11" }, { "100101001", "D12" }, { "100110100", "D13" }, 
  { "100101100", "D14" }, { "100101110", "D15" }, { "100110001", "D16" }, { "100110010", "D17" }, 
  { "100110011", "D18" }, { "110010100", "I1" }, { "100010100", "I11" }, { "110101100", "I12" }
};

// Station ID to direction mapping
unordered_map<string, bool> STATION_ID_TO_DIRECTION_MAP = {
  { "D1", 0 }, { "D2", 0 }, { "D3", 1 }, { "D4", 1 }, { "D5", 1 }, { "D6", 1 }, 
  { "D7", 1 }, { "D9", 1 }, { "D10", 1 }, { "D11", 1 }, { "D12", 1 }, { "D13", 1 }, 
  { "D14", 1 }, { "D15", 1 }, { "D16", 1 }, { "D17", 1 }, { "D18", 1 }, 
  { "I1", 1 }, { "I11", 0 }, { "I12", 1 }
};

// Station ID to expected station ID mapping
unordered_map<string, string> STATION_ID_TO_EXPECTED_STATION_ID_MAP = {
  { "D1", "I1" }, { "D2", "D3" }, { "D3", "D4" }, { "D4", "D5" }, { "D5", "D6" }, 
  { "D6", "D7" }, { "D7", "D9" }, { "D9", "D10" }, { "D10", "D11" }, { "D11", "D12" }, 
  { "D12", "D13" }, { "D13", "D14" }, { "D14", "D15" }, { "D15", "D16" }, { "D16", "D17" }, 
  { "D17", "D18" }, { "D18", "D1" }, { "I1", "I12" }, { "I11", "D2" }, { "I12", "I11" }
};

unordered_map<string, int> STATION_ID_TO_SPEED_MAP = {
  { "D1", S1 }, { "D2", S2 }, { "D3", S2 }, { "D4", S2 }, { "D5", S2 }, { "D6", S2 }, 
  { "D7", S2 }, { "D9", S2 }, { "D10", S2 }, { "D11", S2 }, { "D12", S2 }, { "D13", S2 }, 
  { "D14", S2 }, { "D15", S2 }, { "D16", S2 }, { "D17", S2 }, { "D18", S2 }, 
  { "I1", S0_3 }, { "I11", S1 }, { "I12", S0_05 }
};

//=============================================================================
// GLOBAL VARIABLES - DRIVE MOTOR STATE MACHINE
//=============================================================================

bool traffic_permission = true;                                                   // Traffic permission status
bool current_assignment = true;                                                   // Current assignment status
int permitted_edge_speed = S0_5;                                                  // Permitted edge speed
bool column_detected_in_sweep = false;                                            // Column detected during sweep

int drive_motor_current_speed = 0;                                              // Current speed of the drive motor
int drive_motor_previous_set_speed = 0;                                         // Previous set speed of the drive motor
DriveMotorState drive_motor_current_state = DRIVE_MOTOR_STOP;                   // Current state of the drive motor
DriveMotorState drive_motor_previous_state = DRIVE_MOTOR_STOP;                  // Previous state of the drive motor
DriveMotorDirection drive_motor_current_direction = DRIVE_MOTOR_FORWARD;        // Current direction of the drive motor
DriveMotorDirection drive_motor_previous_direction = DRIVE_MOTOR_FORWARD;       // Previous direction of the drive motor
int drive_motor_set_speed = 0;                                                  // Set speed of the drive motor
int drive_motor_direction_value = 1;                                            // Direction value of the drive motor
bool drive_motor_error_status = false;                                          // Error status of the drive motor

const int DRIVE_MOTOR_SWEEP_SPEED = S0_05;                                      // Sweep speed of the drive motor

//=============================================================================
// GLOBAL VARIABLES - DIVERTER STATE MACHINE
//=============================================================================

Preferences prefs;  // NVM storage for persistent configuration

// Servo Limit Config variables.
int front_diverter_left_limit = 0;              // Left limit for front diverter servo
int front_diverter_right_limit = 0;             // Right limit for front diverter servo
int front_diverter_tolerance = 0;               // Tolerance for front diverter servo
int front_diverter_left_threshold = 0;          // Left threshold for front diverter servo
int front_diverter_right_threshold = 0;         // Right threshold for front diverter servo
int rear_diverter_left_limit = 0;               // Left limit for rear diverter servo
int rear_diverter_right_limit = 0;              // Right limit for rear diverter servo
int rear_diverter_tolerance = 0;                // Tolerance for rear diverter servo
int rear_diverter_left_threshold = 0;           // Left threshold for rear diverter servo
int rear_diverter_right_threshold = 0;          // Right threshold for rear diverter servo

bool global_error_status = false;                                                               // Global error status
DiverterDirection diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;                     // Diverter track edge direction
DiverterDirection previous_diverter_track_edge_direction = DIVERTER_DIRECTION_UNKNOWN;          // Previous diverter track edge direction
DiverterPosition current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;              // Current diverter position high-level

DiverterState diverter_current_state = DIVERTER_STOP;                                // Current diverter state
DiverterState diverter_previous_state = DIVERTER_STOP;                               // Previous diverter state
volatile bool error_diverter_status = false;                                         // Error status of the diverter
bool diverter_set_torque_flag = true;                                                // Flag to set torque for diverter
bool prev_diverter_set_torque_flag = true;                                           // Previous flag to set torque for diverter
int front_diverter_set_position_value = -1;                                          // Set position value for front diverter
int rear_diverter_set_position_value = -1;                                           // Set position value for rear diverter

int front_diverter_current_position = -1;                                            // Current position of front diverter
int rear_diverter_current_position = -1;                                             // Current position of rear diverter

bool error_front_servo = false;                                                      // Error status of front servo
bool error_rear_servo = false;                                                       // Error status of rear servo

int last_commanded_front_position = -1;                                              // Last commanded position for front servo
int last_commanded_rear_position = -1;                                               // Last commanded position for rear servo

// Retry tracking for diverter operations
uint8_t front_position_write_retry_count = 0;      // Retry counter for front position writes
uint8_t rear_position_write_retry_count = 0;       // Retry counter for rear position writes
uint8_t front_position_read_retry_count = 0;       // Retry counter for front position reads
uint8_t rear_position_read_retry_count = 0;        // Retry counter for rear position reads
uint8_t front_torque_enable_retry_count = 0;       // Retry counter for front torque enable
uint8_t rear_torque_enable_retry_count = 0;        // Retry counter for rear torque enable
uint8_t front_torque_disable_retry_count = 0;      // Retry counter for front torque disable
uint8_t rear_torque_disable_retry_count = 0;       // Retry counter for rear torque disable

// Timing for position write cooldown
unsigned long last_front_position_write_time = 0;  // Timestamp of last front position write
unsigned long last_rear_position_write_time = 0;   // Timestamp of last rear position write

unsigned long last_position_read_time = 0;
const unsigned long POSITION_READ_INTERVAL = 50;  // 50ms between position reads

//=============================================================================
// GLOBAL VARIABLES - WTM COLUMN STATE MACHINE
//=============================================================================

WTMColumnState wtm_column_state = WTM_BETWEEN_COLUMNS;

// Frame counts for each possible column state
int frameCount00 = 0;
int frameCount01 = 0;
int frameCount10 = 0;
int frameCount11 = 0;

// Confirmation variables
int exitConfirmationCount = 0;  // Count of consecutive 00 samples
const int EXIT_CONFIRMATION_THRESHOLD = 3;  // Need 5 consecutive 00s to confirm exit (relaxed from 10)

// Column tracking
String detectedColumnCode = "";  // The actual column code (01, 10, or 11)
String currentColumnCode = "";    // Current column code being read
String previousColumnCode = "";   // Previous column code that was detected

// Health monitoring thresholds
const float MAJORITY_SAFE_THRESHOLD = 80.0;      // > 80% is safe
const float MAJORITY_WARNING_THRESHOLD = 60.0;   // 60-80% is warning, < 60% is critical
const float MINORITY_SAFE_THRESHOLD = 20.0;      // < 20% is safe
const float MINORITY_WARNING_THRESHOLD = 40.0;   // 20-40% is warning, > 40% is critical
const int DEFAULT_COLUMN_FRAME_THRESHOLD = 5;    // Conservative threshold for column width
const int MIN_COLUMN_FRAMES_THRESHOLD = 3;       // Minimum frames to consider valid column (filter noise)
const int DEFAULT_EMPTY_SPACE_THRESHOLD = 10;    // Expected minimum frames for empty space (relaxed from 30)
const float EMPTY_SPACE_NOISE_THRESHOLD = 5.0;   // < 5% non-00 readings is safe for empty space

HealthStatus majorityHealthWTM = SAFE;
HealthStatus minorityHealthWTM = SAFE;
HealthStatus overallHealthWTM = SAFE;

// Empty space tracking for health monitoring
int emptySpaceFrameCount = 0;        // Frame count for current empty space (pure 00s)
int emptySpaceNoiseFrameCount = 0;   // Count of non-00 frames detected during empty space (for health)
int emptySpaceTotalFrameCount = 0;   // Total frames in empty space including noise
HealthStatus emptySpaceHealth = SAFE; // Health of empty space (should have minimal non-00 readings)
HealthStatus emptySpaceMajorityHealth = SAFE;  // Majority health (00 frames)
HealthStatus emptySpaceMinorityHealth = SAFE;  // Minority health (noise frames)
int totalEmptySpacesDetected = 0;    // Total empty spaces detected
int totalFramesBetweenSpaces = 0;

// Column completion flag and data - set when column successfully read
bool columnJustCompleted = false;      // Flag: true when column just finished reading
String lastCompletedColumnCode = "";   // The column code that was just read (01, 10, 11)
HealthStatus lastCompletedColumnHealth = SAFE;  // Health of the column that was just read

// Tracking variables for station-to-station segment
int uniqueColumnsDetectedInSegment = 0;  // Count of unique columns (01, 10, 11) between stations
int emptySpacesDetectedInSegment = 0;    // Count of 00 zones between stations

int totalColumnsDetected = 0;

// =============================================================================
// GLOBAL VARIABLES - DROPPING AND DESTINATION UPDATES AND FIRMWARE
// =============================================================================

String previous_station = "X";
String dropoff_station = "X";
bool get_destination_api_flag = false;
bool clear_infeed_api_flag = false;
bool start_infeed_api_column_counter = false;
int infeed_api_column_counter = 0;
bool previous_detection_for_infeed_api = 0;
bool update_parcel_dropped_in_the_dump_api_flag = false;
bool update_parcel_dropped_api_flag = false;

bool start_column_counter = false;
int column_counter = 0;
bool previous_detection = 0;

TaskHandle_t api_task;
const String DMS_URL_PREFIX = "http://192.168.2.109:8443/m-sort/m-sort-distribution-server/";
HTTPClient _http;

const char* checkVersionEndpoint = "/ota-and-debugger/checkCurrentFirmwareVersion";
const char* updateFirmwareEndpoint = "/ota-and-debugger/updateDeviceRoleFirmware";
const char* deviceRole = "bot_primary";

// Add these variables at the top of your file with other global variables
int consecutive_detections = 0;
const int REQUIRED_CONSECUTIVE_DETECTIONS = 5;

// Retry configuration
const int maxRetries = 5;
const int initialRetryDelay = 5000;  // 5 seconds
const int maxRetryDelay = 300000;    // 5 minutes

// =============================================================================
// GLOBAL VARIABLES - FLAP CONTROL
// =============================================================================
bool closing_flap = false;
unsigned long start_closing_time;

//=============================================================================
// MISCELLANEOUS GLOBAL VARIABLES
//=============================================================================
int application_error_code = -1;
bool first_time_error = false;
volatile bool diverter_direction;
bool beep_flag = false;
int previous_permitted_edge_speed = -1;

// Task handles
TaskHandle_t reading_sensor_and_update_state_task;
TaskHandle_t actuation_task;
TaskHandle_t http_debug_log_task;

// Synchronization primitives
SemaphoreHandle_t xDebugLogMutex;  // Protects debugLoggingString (accessed from multiple cores/tasks)

Adafruit_MCP23X17 mcp;              // I/O expander object

//=============================================================================
// FORWARD DECLARATIONS - MODBUS RTU FUNCTIONS
//=============================================================================
uint16_t calcCRC(uint8_t *data, uint8_t len);
void sendPacket(uint8_t *buffer, uint8_t length);
int readResponse(uint8_t *buffer, int expectedLen, int timeoutMs = 100);

//=============================================================================
// STATION READER FUNCTIONS
//=============================================================================

// Function to read station data via photodiodes
void ReadStationViaPhotoDiode() {
  for (int i = 0; i < 3; i++) {
    photo_transistor_previous_state_array[i] = photo_transistor_current_state_array[i];
    switch (i) {
      case 0:
        photo_transistor_current_value_array[i] = map(analogRead(SR_D1), 0, 4096, 0, 1024);
        break;
      case 1:
        photo_transistor_current_value_array[i] = map(analogRead(SR_D2), 0, 4096, 0, 1024);
        break;
      case 2:
        photo_transistor_current_value_array[i] = map(analogRead(SR_D3), 0, 4096, 0, 1024);
        break;
    }
    if (photo_transistor_current_value_array[i] > PT_UPPER_THR_LIMIT) {
      photo_transistor_current_state_array[i] = 1;
    } else if (photo_transistor_current_value_array[i] < PT_LOWER_THR_LIMIT) {
      photo_transistor_current_state_array[i] = 0;
    }
  }
}

// Function to check intensity warning levels for a column segment
void CheckColumnIntensityWarningLevel(String segment) {
  // Loop through each bit of the segment
  for (int i = 0; i < 3; i++) {
    int bit = segment[i] - '0';
    int intensity_max = instantaneous_intensity_max_array[i];
    int intensity_min = instantaneous_intensity_min_array[i];
    String status = "Healthy";

    if (bit == 1) {
      if (intensity_max >= 800)
        status = "Healthy";
      else if (intensity_max >= 700 && intensity_max < 800)
        status = "Warning";
      else
        status = "Critical";
    } else { // bit == 0
      if (intensity_min <= 200)
        status = "Healthy";
      else if (intensity_min > 200 && intensity_min <= 300)
        status = "Warning";
      else
        status = "Critical";
    }

    if (status != "Healthy") {
      add_log("Photodiode " + String(i + 1) + ": " + status);
    }
  }
}

// Function to reinitialize variables related to station reading
void ReinitialiseVariables() {
  for (int i = 0 ; i < 3 ; i++) {
    photo_transistor_previous_state_array[i] = 0;
    final_column_code_array[i] = "000";
    photo_transistor_current_value_array[i] = 0;
    photo_transistor_current_state_array[i] = 0;
  }
  intermediate_column_code = "000";
  previous_intermediate_column_code = "000";
  column_no = 0;
}

// Function to get station code from data read by station sensor
String GetStationCodeFromDataReadByStationSensor() {
  String instColumnCode = String(photo_transistor_current_state_array[0]) + String(photo_transistor_current_state_array[1]) + String(photo_transistor_current_state_array[2]);
  String prevInstColumnCode = String(photo_transistor_previous_state_array[0]) + String(photo_transistor_previous_state_array[1]) + String(photo_transistor_previous_state_array[2]);
  if (instColumnCode == prevInstColumnCode) {
    instantaneous_frames_count++;
    for (int i = 0 ; i < 3 ; i++) {
      instantaneous_intensity_max_array[i] = max(instantaneous_intensity_max_array[i], photo_transistor_current_value_array[i]);
      instantaneous_intensity_min_array[i] = min(instantaneous_intensity_min_array[i], photo_transistor_current_value_array[i]);
    }
  } else {
    add_log("Inst. Col: " + prevInstColumnCode);
    add_log("Inst. FC: " + String(instantaneous_frames_count));
    add_log("Inst. Max. Intensity: { " + String(instantaneous_intensity_max_array[0]) + ", " + String(instantaneous_intensity_max_array[1]) + ", " + String(instantaneous_intensity_max_array[2]) + " }");
    add_log("Inst. Min. Intensity: { " + String(instantaneous_intensity_min_array[0]) + ", " + String(instantaneous_intensity_min_array[1]) + ", " + String(instantaneous_intensity_min_array[2]) + " }");
    CheckColumnIntensityWarningLevel(prevInstColumnCode);
    instantaneous_frames_count = 1;
    for (int i = 0 ; i < 3 ; i++) {
      instantaneous_intensity_max_array[i] = photo_transistor_current_value_array[i];
      instantaneous_intensity_min_array[i] = photo_transistor_current_value_array[i];
    }
  }

  if (instColumnCode == "111") {
    intermediate_column_code = instColumnCode;
  }

  // START STATION READING
  if (!reading_in_process && instColumnCode == "111") {
    reading_in_process = true;
    station_read_start_time = millis();
  }

  // STATION READING COMPLETED
  String stationCode;
  if (instColumnCode == "000" && reading_in_process) {
    if (final_column_code_array[2] != "000" && final_column_code_array[1] != "000" && final_column_code_array[0] != "000") {
      stationCode = final_column_code_array[2] + final_column_code_array[1] + final_column_code_array[0];
      right_station_detected = true;
      reading_in_process = false;
    }
    add_log("Read time: " + String(millis() - station_read_start_time));
    if (column_no < 3) add_log("Bot has skipped " + String(3 - column_no) + " columns");
    ReinitialiseVariables();
  }

  // OPEN ZONE TRAVEL
  if (instColumnCode == "000") {
    reading_in_process = false;
  }

  if (reading_in_process) {
    int currentStateSum = photo_transistor_current_state_array[0] + photo_transistor_current_state_array[1] + photo_transistor_current_state_array[2];
    int prevStateSum = photo_transistor_previous_state_array[0] + photo_transistor_previous_state_array[1] + photo_transistor_previous_state_array[2];
    if (currentStateSum < prevStateSum) {
      intermediate_column_code = instColumnCode;
      intermediate_column_frames_count = 1;
    } else if (intermediate_column_code == instColumnCode) {
      // Nothing
      if (currentStateSum > 0 && currentStateSum < 3) {
        intermediate_column_frames_count++;
      }
    }

    if (previous_intermediate_column_code != "000" && previous_intermediate_column_code != "111" && intermediate_column_code == "111" && intermediate_column_code != "000" && final_column_code_array[0] != previous_intermediate_column_code) {
      if (intermediate_column_frames_count >= 5) {
        final_column_code_array[2] = final_column_code_array[1];
        final_column_code_array[1] = final_column_code_array[0];
        final_column_code_array[0] = previous_intermediate_column_code;
        column_no++;
        add_log("Col No.: " + String(column_no));
        if (column_no > 3) add_log("Bot is reading more than 3 columns");
      } else {
        add_log("Potential fluke column. No. of frames: " + String(intermediate_column_frames_count));
      }
      add_log("Inter. Col: " + previous_intermediate_column_code);
      add_log("Inter. FC: " + String(intermediate_column_frames_count));
    }
    previous_intermediate_column_code = intermediate_column_code;
  }

  String stationCodeMain = "";
  if (right_station_detected) {
    right_station_detected = false;
    stationCodeMain = stationCode;
  }
  return stationCodeMain;
}

// Function to transform station code into station ID
String TransformStationCodeIntoStationId(const String &station_code_param) {
  string station_code_string = string(station_code_param.c_str());
  if (STATION_CODE_TO_STATION_ID_MAP.find(station_code_string) != STATION_CODE_TO_STATION_ID_MAP.end()) {
    string station_id = STATION_CODE_TO_STATION_ID_MAP.at(station_code_string);
    return String(station_id.c_str());
  }
  return "";
}

// Function to check if station code is valid
bool CheckStationValidity(const String &station_code_param) {
  string station_code_string = string(station_code_param.c_str());
  return STATION_CODE_TO_STATION_ID_MAP.find(station_code_string) != STATION_CODE_TO_STATION_ID_MAP.end();
}

// Function to get expected station ID from current station ID
String GetExpectedStationFromStationId(const String &station_code_param) {
  string station_code_string = string(station_code_param.c_str());
  if (STATION_ID_TO_EXPECTED_STATION_ID_MAP.find(station_code_string) != STATION_ID_TO_EXPECTED_STATION_ID_MAP.end()) {
    string station_id = STATION_ID_TO_EXPECTED_STATION_ID_MAP.at(station_code_string);
    return String(station_id.c_str());
  }
  return "";
}

// Function to get direction from station ID
bool GetDirectionFromStationId(const String &station_code_param) {
  string station_code_string = string(station_code_param.c_str());
  if (STATION_ID_TO_DIRECTION_MAP.find(station_code_string) != STATION_ID_TO_DIRECTION_MAP.end()) {
    bool direction = STATION_ID_TO_DIRECTION_MAP.at(station_code_string);
    return direction;
  }
  return 1;
}

// Function to get speed from station ID
int GetSetSpeedAsPerStation(const String &station_id_param) {
  string station_code_string = string(station_id_param.c_str());
  if (STATION_ID_TO_SPEED_MAP.find(station_code_string) != STATION_ID_TO_SPEED_MAP.end()) {
    int speed = STATION_ID_TO_SPEED_MAP.at(station_code_string);
    return speed;
  }
  return S0_5;
}

//=============================================================================
// DRIVE MOTOR STATE MACHINE FUNCTIONS
//=============================================================================

// Function to update drive motor state based on current conditions
DriveMotorStateOutput UpdateDriveMotorState() {
  drive_motor_previous_state = drive_motor_current_state;
  
  // ----- FROM STOP STATE -----
  if (drive_motor_current_state == DRIVE_MOTOR_STOP) {
    if (drive_motor_error_status == true) {
      drive_motor_current_state = DRIVE_MOTOR_ERROR;
      return DRIVE_MOTOR_STATE_CHANGED;
    }

    if (current_assignment == true && traffic_permission == true && global_error_status == false) {
      drive_motor_current_state = DRIVE_MOTOR_RUNNING;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    if (current_assignment == true && drive_motor_current_speed == 0 && traffic_permission == false && global_error_status == false) {
      drive_motor_current_state = DRIVE_MOTOR_SWEEPING_COLUMN;
      column_detected_in_sweep = false;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    return DRIVE_MOTOR_STATE_NO_CHANGE;
  }
  
  // ----- FROM STOPPING STATE -----
  else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
    if (drive_motor_error_status == true) {
      drive_motor_current_state = DRIVE_MOTOR_ERROR;
      return DRIVE_MOTOR_STATE_CHANGED;
    }

    if (drive_motor_current_speed == 0) {
      drive_motor_current_state = DRIVE_MOTOR_STOP;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    if (current_assignment == true && traffic_permission == true && global_error_status == false) {
      drive_motor_current_state = DRIVE_MOTOR_RUNNING;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    return DRIVE_MOTOR_STATE_NO_CHANGE;
  }
  
  // ----- FROM RUNNING STATE -----
  else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
    if (drive_motor_error_status == true) {
      drive_motor_current_state = DRIVE_MOTOR_ERROR;
      return DRIVE_MOTOR_STATE_CHANGED;
    }

    if (global_error_status == true || (current_assignment == true && traffic_permission == false)) {
      drive_motor_current_state = DRIVE_MOTOR_STOPPING;
      return DRIVE_MOTOR_STATE_CHANGED;
    }

    if (previous_permitted_edge_speed != permitted_edge_speed) {
      drive_motor_current_state = DRIVE_MOTOR_RUNNING;
      previous_permitted_edge_speed = permitted_edge_speed;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    return DRIVE_MOTOR_STATE_NO_CHANGE;
  }
  
  // ----- FROM SWEEPING_COLUMN STATE -----
  else if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
    if (drive_motor_error_status == true) {
      drive_motor_current_state = DRIVE_MOTOR_ERROR;
      return DRIVE_MOTOR_STATE_CHANGED;
    }

    if (global_error_status == true || (current_assignment == true && traffic_permission == false && column_detected_in_sweep == true)) {
      drive_motor_current_state = DRIVE_MOTOR_STOPPING;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    if (current_assignment == true && traffic_permission == true && global_error_status == false) {
      drive_motor_current_state = DRIVE_MOTOR_RUNNING;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    return DRIVE_MOTOR_STATE_NO_CHANGE;
  }
  
  // ----- FROM ERROR_MOTOR STATE -----
  else if (drive_motor_current_state == DRIVE_MOTOR_ERROR) {
    if (drive_motor_error_status == false && drive_motor_current_speed == 0) {
      drive_motor_current_state = DRIVE_MOTOR_STOP;
      return DRIVE_MOTOR_STATE_CHANGED;
    }

    if (drive_motor_error_status == false && drive_motor_current_speed > 0) {
      drive_motor_current_state = DRIVE_MOTOR_STOPPING;
      return DRIVE_MOTOR_STATE_CHANGED;
    }
    
    return DRIVE_MOTOR_STATE_NO_CHANGE;
  }
  
  return DRIVE_MOTOR_UNKNOWN_STATE;
}

// Function to update drive motor actuator input variables based on current state
void UpdateDriveMotorActuatorInputVariables() {
  if (drive_motor_current_state == DRIVE_MOTOR_STOP) {
    drive_motor_set_speed = 0;
  }
  else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
    drive_motor_set_speed = 0;
  }
  else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
    drive_motor_set_speed = permitted_edge_speed;
    drive_motor_current_direction = DRIVE_MOTOR_FORWARD;
  }
  else if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
    if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
      drive_motor_current_direction = DRIVE_MOTOR_REVERSE;
    }
    else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
      drive_motor_current_direction = DRIVE_MOTOR_FORWARD;
    }
    
    if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
      drive_motor_direction_value = 1;
    }
    else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
      drive_motor_direction_value = -1;
    }
    
    drive_motor_set_speed = DRIVE_MOTOR_SWEEP_SPEED * drive_motor_direction_value;
  }
  else if (drive_motor_current_state == DRIVE_MOTOR_ERROR) {
    drive_motor_set_speed = 0;
  }
}

// Set the target velocity
CANStatus setTargetVelocity(int velocity) {
  add_log("→ Set velocity " + String(velocity) + " rpm");
  
  CANStatus status = SDO_Write(TARGET_VELOCITY, 0x00, (uint32_t)velocity);
  if (status != CAN_OK) {
    add_log("✗ Failed to set velocity");
    return status;
  }
  
  status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
  if (status == CAN_OK) {
    add_log("✓ Velocity set");
  } else {
    add_log("✗ Failed to enable after velocity set");
  }
  
  return status;
}

// Halt the movement
CANStatus haltMovement() {
  add_log("→ Halt...");
  CANStatus status = SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_HALT);
  if (status == CAN_OK) {
    add_log("✓ Halt complete");
  }
  return status;
}

// Get the target velocity
CANStatus getTargetVelocity(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(TARGET_VELOCITY, 0x00, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;
  } else {
    *velocity = -1;
  }
  return status;
}

// Get the actual velocity (averaged)
CANStatus getVelocityActualValueAveraged(int32_t* velocity) {
  uint32_t raw;
  CANStatus status = SDO_Read(VELOCITY_ACTUAL_VALUE_AVG, 0x01, &raw);
  if (status == CAN_OK) {
    *velocity = (int32_t)raw;
  } else {
    *velocity = -1;
  }
  return status;
}

//=============================================================================
// DIVERTER STATE MACHINE FUNCTIONS
//=============================================================================

// Function to update high-level diverter position based on current servo positions
void UpdateDiverterPositionHighLevel() {
  if (front_diverter_current_position >= (front_diverter_right_threshold - front_diverter_tolerance) && 
      front_diverter_current_position <= front_diverter_right_limit &&
      rear_diverter_current_position <= (rear_diverter_right_threshold + rear_diverter_tolerance) && 
      rear_diverter_current_position >= rear_diverter_right_limit) {
    current_diverter_position_high_level = DIVERTER_POSITION_RIGHT;
  }
  else if (front_diverter_current_position >= front_diverter_left_limit && 
           front_diverter_current_position <= (front_diverter_left_threshold + front_diverter_tolerance) &&
           rear_diverter_current_position <= rear_diverter_left_limit && 
           rear_diverter_current_position >= (rear_diverter_left_threshold - rear_diverter_tolerance)) {
    current_diverter_position_high_level = DIVERTER_POSITION_LEFT;
  }
  else if ((front_diverter_current_position < (front_diverter_right_threshold - front_diverter_tolerance) && 
           front_diverter_current_position > (front_diverter_left_threshold + front_diverter_tolerance)) ||
           (rear_diverter_current_position > (rear_diverter_right_threshold + rear_diverter_tolerance) && 
           rear_diverter_current_position < (rear_diverter_left_threshold - rear_diverter_tolerance))) {
    current_diverter_position_high_level = DIVERTER_POSITION_INTERIM;
  }
  else {
    current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;
  }
}

// Function to update diverter state based on current conditions
DiverterStateChangeOptions UpdateDiverterState() {
  diverter_previous_state = diverter_current_state;
  
  // ----- FROM LEFT STATE -----
  if (diverter_current_state == DIVERTER_LEFT) {
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (error_diverter_status == false && global_error_status == false && 
        (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT ||
         current_diverter_position_high_level != DIVERTER_POSITION_LEFT)) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM RIGHT STATE -----
  else if (diverter_current_state == DIVERTER_RIGHT) {
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
      
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (error_diverter_status == false && global_error_status == false && 
        (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT ||
         current_diverter_position_high_level != DIVERTER_POSITION_RIGHT)) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM SWITCHING STATE -----
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (global_error_status == true) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT && 
        current_diverter_position_high_level == DIVERTER_POSITION_LEFT) {
      diverter_current_state = DIVERTER_LEFT;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT && 
        current_diverter_position_high_level == DIVERTER_POSITION_RIGHT) {
      diverter_current_state = DIVERTER_RIGHT;
      return DIVERTER_STATE_CHANGED;
    }

    if (previous_diverter_track_edge_direction != diverter_track_edge_direction) {
      diverter_current_state = DIVERTER_SWITCHING;
      previous_diverter_track_edge_direction = diverter_track_edge_direction;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM STOP_DIVERTER STATE -----
  else if (diverter_current_state == DIVERTER_STOP) {
    if (error_diverter_status == true) {
      diverter_current_state = DIVERTER_ERROR;
      return DIVERTER_STATE_CHANGED;
    }
    
    if (error_diverter_status == false && global_error_status == false) {
      diverter_current_state = DIVERTER_SWITCHING;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  // ----- FROM ERROR_DIVERTER STATE -----
  else if (diverter_current_state == DIVERTER_ERROR) {
    if (error_diverter_status == false) {
      diverter_current_state = DIVERTER_STOP;
      return DIVERTER_STATE_CHANGED;
    }
    
    return DIVERTER_STATE_NO_CHANGE;
  }
  
  return DIVERTER_UNEXPECTED_CONDITION;
}

// Function to update diverter actuator input variables based on current state
void UpdateDiverterActuatorInputVariables() {
  if (diverter_current_state == DIVERTER_LEFT) {
    diverter_set_torque_flag = false;
    // add_log("Diverter in LEFT position - torque disabled.");
  }
  else if (diverter_current_state == DIVERTER_RIGHT) {
    diverter_set_torque_flag = false;
    // add_log("Diverter in RIGHT position - torque disabled.");
  }
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    diverter_set_torque_flag = true;
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      front_diverter_set_position_value = front_diverter_left_limit;
      rear_diverter_set_position_value = rear_diverter_left_limit;
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      front_diverter_set_position_value = front_diverter_right_limit;
      rear_diverter_set_position_value = rear_diverter_right_limit;
    }
  }
  else if (diverter_current_state == DIVERTER_ERROR) {
    diverter_set_torque_flag = false;
    // add_log("Diverter error detected! Torque disabled.");
  }
  else if (diverter_current_state == DIVERTER_STOP) {
    diverter_set_torque_flag = false;
    // add_log("Diverter stopped - torque disabled.");
  }
}

//=============================================================================
// DIVERTER SERVO COMMANDS - FRONT SERVO
//=============================================================================

// Enable or disable torque for the front servo
bool enableTorqueFront(bool enable) {
  uint8_t txBuf[8], rxBuf[8];
  
  txBuf[0] = FRONT_SERVO_ID;
  txBuf[1] = FC_WRITE_SINGLE;
  txBuf[2] = (REG_TORQUE_ENABLE >> 8) & 0xFF;
  txBuf[3] = REG_TORQUE_ENABLE & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = enable ? 0x01 : 0x00;
  
  sendPacket(txBuf, 6);
  return (readResponse(rxBuf, 8, 100) == 8);
}

// Set position, speed, and acceleration for the front servo
bool setPositionFront(int16_t position, uint16_t speed, uint8_t acc) {
  uint8_t txBuf[20], rxBuf[8];
  
  txBuf[0] = FRONT_SERVO_ID;
  txBuf[1] = FC_WRITE_MULTIPLE;
  txBuf[2] = (REG_GOAL_POSITION >> 8) & 0xFF;
  txBuf[3] = REG_GOAL_POSITION & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x04;
  txBuf[6] = 0x08;
  
  txBuf[7] = (position >> 8) & 0xFF;
  txBuf[8] = position & 0xFF;
  txBuf[9] = 0x00;
  txBuf[10] = 0x01;
  txBuf[11] = (acc >> 8) & 0xFF;
  txBuf[12] = acc & 0xFF;
  txBuf[13] = (speed >> 8) & 0xFF;
  txBuf[14] = speed & 0xFF;
  
  sendPacket(txBuf, 15);
  return (readResponse(rxBuf, 8, 100) == 8);
}

// Read the current position of the front servo
int16_t readPositionFront() {
  uint8_t txBuf[8], rxBuf[10];
  
  txBuf[0] = FRONT_SERVO_ID;
  txBuf[1] = FC_READ_REGISTERS;
  txBuf[2] = (REG_PRESENT_POS >> 8) & 0xFF;
  txBuf[3] = REG_PRESENT_POS & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x01;
  
  sendPacket(txBuf, 6);
  
  int result = readResponse(rxBuf, 7, 100);
  if (result == 7) {
    return (rxBuf[3] << 8) | rxBuf[4];
  }
  return -1;
}

//=============================================================================
// DIVERTER SERVO COMMANDS - REAR SERVO
//=============================================================================

// Enable or disable torque for the rear servo
bool enableTorqueRear(bool enable) {
  uint8_t txBuf[8], rxBuf[8];
  
  txBuf[0] = REAR_SERVO_ID;
  txBuf[1] = FC_WRITE_SINGLE;
  txBuf[2] = (REG_TORQUE_ENABLE >> 8) & 0xFF;
  txBuf[3] = REG_TORQUE_ENABLE & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = enable ? 0x01 : 0x00;
  
  sendPacket(txBuf, 6);
  return (readResponse(rxBuf, 8, 100) == 8);
}

// Set position, speed, and acceleration for the rear servo
bool setPositionRear(int16_t position, uint16_t speed, uint8_t acc) {
  uint8_t txBuf[20], rxBuf[8];
  
  txBuf[0] = REAR_SERVO_ID;
  txBuf[1] = FC_WRITE_MULTIPLE;
  txBuf[2] = (REG_GOAL_POSITION >> 8) & 0xFF;
  txBuf[3] = REG_GOAL_POSITION & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x04;
  txBuf[6] = 0x08;
  
  txBuf[7] = (position >> 8) & 0xFF;
  txBuf[8] = position & 0xFF;
  txBuf[9] = 0x00;
  txBuf[10] = 0x01;
  txBuf[11] = (acc >> 8) & 0xFF;
  txBuf[12] = acc & 0xFF;
  txBuf[13] = (speed >> 8) & 0xFF;
  txBuf[14] = speed & 0xFF;
  
  sendPacket(txBuf, 15);
  return (readResponse(rxBuf, 8, 100) == 8);
}

// Read the current position of the rear servo
int16_t readPositionRear() {
  uint8_t txBuf[8], rxBuf[10];
  
  txBuf[0] = REAR_SERVO_ID;
  txBuf[1] = FC_READ_REGISTERS;
  txBuf[2] = (REG_PRESENT_POS >> 8) & 0xFF;
  txBuf[3] = REG_PRESENT_POS & 0xFF;
  txBuf[4] = 0x00;
  txBuf[5] = 0x01;
  
  sendPacket(txBuf, 6);
  
  int result = readResponse(rxBuf, 7, 100);
  if (result == 7) {
    return (rxBuf[3] << 8) | rxBuf[4];
  }
  return -1;
}

// =============================================================================
// WTM SENSOR READING AND PROCESSING FUNCTIONS
// =============================================================================

// Function to read TI and CI sensor combo
String ReadSensorCombo() {
  int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
  int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);
  return String(trafficDetected) + String(columnDetected);
}

// Function to handle column detection and processing
void ProcessColumnDetection(String combo) {
  // Column detection state machine
  switch (wtm_column_state) {
    case WTM_BETWEEN_COLUMNS:
      // We're in the 00 zone between columns
      if (combo == "00") {
        emptySpaceFrameCount++;
      } else {
        // Detected non-00 in empty space - transition to column
        wtm_column_state = WTM_IN_COLUMN;
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

    case WTM_IN_COLUMN:
      // Count all frames by type
      if (combo == "00") {
        frameCount00++;
      } else if (combo == "01") {
        frameCount01++;
        totalFramesBetweenSpaces++;
        if (frameCount01 > max(frameCount10, frameCount11)) {
          detectedColumnCode = "01";
          previousColumnCode = currentColumnCode;
          currentColumnCode = "01";
        }
      } else if (combo == "10") {
        frameCount10++;
        totalFramesBetweenSpaces++;
        if (frameCount10 > max(frameCount01, frameCount11)) {
          detectedColumnCode = "10";
          previousColumnCode = currentColumnCode;
          currentColumnCode = "10";
        }
      } else if (combo == "11") {
        frameCount11++;
        totalFramesBetweenSpaces++;
        if (frameCount11 > max(frameCount01, frameCount10)) {
          detectedColumnCode = "11";
          previousColumnCode = currentColumnCode;
          currentColumnCode = "11";
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
            wtm_column_state = WTM_BETWEEN_COLUMNS;
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
          
          // Update previous and current column codes
          previousColumnCode = detectedColumnCode;
          currentColumnCode = "00";
          
          columnJustCompleted = true;
          lastCompletedColumnCode = detectedColumnCode;
          lastCompletedColumnHealth = AnalyzeWTMColumnHealth();
          
          wtm_column_state = WTM_BETWEEN_COLUMNS;
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
    if (drive_motor_current_state == DRIVE_MOTOR_SWEEPING_COLUMN) {
      column_detected_in_sweep = true;
    }
    if (lastCompletedColumnCode == "11") {
      add_log("Traffic Detected");
      traffic_permission = false;
    } else if (lastCompletedColumnCode == "01") {
      add_log("Traffic Not Detected");
      traffic_permission = true;
    } else if (lastCompletedColumnCode == "10") {
      add_log("WARNING: Column Indicator malfunction detected! Reading '10' (CI=0, TI=1)");
    }
    
    if (lastCompletedColumnHealth == CRITICAL) {
      // Column health is critical - may need maintenance
      // add_log("WARNING: Column health is CRITICAL!");
    }
    
    columnJustCompleted = false;
  }
}

// ==============================================================================
// DROPPING FLAP CONTROLS
// ==============================================================================

// Function to close the flap
void CloseFlap() {
  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, HIGH);
}

// Function to open the flap
void OpenFlap() {
  digitalWrite(EMG_PIN, LOW);
}

// Function to stage the flap
void StageFlap() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, LOW);
}

// ==============================================================================
// DESTINATION AND INFEED API HANDLING FUNCTIONS
// ==============================================================================

void GetDestinationAndJourneyPath() {
  String url = DMS_URL_PREFIX + "GetDestinationStationIdForBot?botId=" + BOT_ID + "&infeedStationId=I1&strategyType=LiveClientServerStrategy&retries=0&sectionId=S1";
  int httpCode = 0;
  add_log("Get Destination API call: " + url);
  _http.begin(url);
  httpCode = _http.GET();
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
    add_log("Get Destination API response: " + response);
  } else {
    add_log("Error: " + _http.getString());
  }
  _http.end();
  
  if (response == "") {
    dropoff_station = "D18";
  } else {
    int destinationIndex = response.indexOf('-');
    dropoff_station = response.substring(0, destinationIndex);
  }
}

void UpdateParcelDropped() {
  String url = DMS_URL_PREFIX + "UpdateParcelDroppedForBot?botId=" + BOT_ID + "&sectionIds=S1&infeedStationId=I1";
  int httpCode = 0;
    add_log("Update succesful drop API call: " + url);
  _http.begin(url);
  httpCode = _http.PUT(url);
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
    add_log("Update API response: " + response);
  } else {
    add_log("Error: " + _http.getString());
  }
  _http.end();
}

void UpdateParcelDroppedInTheDump() {
  String url = DMS_URL_PREFIX + "UpdateParcelDroppedInDumpForBot?botId=" + BOT_ID + "&sectionIds=S1&infeedStationId=I1&reason=FailedDetection";
  int httpCode = 0;
  add_log("Update Parcel Dropped In Dump API call: " + url);
  _http.begin(url);
  httpCode = _http.PUT(url);
  String response = "";
  if (httpCode == HTTP_CODE_OK) {
    response = _http.getString();
    add_log("Update API response: " + response);  
  } else {
    add_log("Error: " + _http.getString());
  }
  _http.end();
}

void ClearInfeedStation() {
  String url = DMS_URL_PREFIX + "ClearInfeedStations?infeedStationId=I1";
  int httpCode = 0;
  add_log("Clear Infeed Station API call: " + url);
  _http.begin(url);
  while (httpCode != HTTP_CODE_OK) {
    httpCode = _http.PUT(url);
    add_log("Clear Infeed Station API response: " + String(_http.getString()));
    delay(100);
  }
  _http.end();
}

// ==============================================================================
// RTOS TASKS
// ==============================================================================

// Diverter State Machine Task
void READING_SENSOR_AND_UPDATE_STATE_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms cycle time
    
    // Update error statuses
    error_diverter_status = error_front_servo | error_rear_servo;
    global_error_status = station_reader_error_status | drive_motor_error_status | error_diverter_status;

    ReadStationViaPhotoDiode();  // Function to read the analog values from the 3 photo diodes and assign a digital value depending upon the analog value is greeater than threshold or not, assign 1 if value greater than threshold upper limit, 0 is value lower than lower limit

    String station_code_main = GetStationCodeFromDataReadByStationSensor(); // generate the full station code in string format based on the columns read.

    if (station_code_main != "" && station_code_main.length() == 9) { // If station reading is complete only then function will be run, 2 conditions must be satisfied for station reading to be complete, all 3 columns must be read, only then length of stationcode string will be 9, also stationCode must not be empty for station reading to be complete
      if (CheckStationValidity(station_code_main)) {
        current_station = TransformStationCodeIntoStationId(station_code_main);
        if (!localisation_flag) {
          if (expected_station != current_station) {
            application_error_code = 3;
            station_reader_error_status = true;
            first_time_error = true;
          }
        } else {
          localisation_flag = false;
        }

        diverter_direction = GetDirectionFromStationId(current_station);
        if (!diverter_direction) diverter_track_edge_direction = DIVERTER_DIRECTION_LEFT;
        else diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;
        permitted_edge_speed = GetSetSpeedAsPerStation(current_station);
        expected_station = GetExpectedStationFromStationId(current_station);
      } else {
        add_log("Invalid station code: " + station_code_main);
        application_error_code = 4;
        station_reader_error_status = true;
        first_time_error = true;
      }
      add_log("Current Station: " + current_station + ", Expected Station: " + expected_station + ", Direction: " + String(diverter_track_edge_direction) + ", Permitted Speed: " + String(permitted_edge_speed));
    }

    // Read TI/CI sensors and process column detection
    int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
    int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);
    String combo = String(trafficDetected) + String(columnDetected);
    ProcessColumnDetection(combo);
    HandleColumnDecisions();

    // ---------------------------------------------------------------- Infeed Controller --------------------------------------------------------
    if (previous_station != current_station) {
      if (current_station == "I12") {
        start_column_counter = true;
      }
    }
    
    if (columnDetected) {
      consecutive_detections++;
      
      if (consecutive_detections >= REQUIRED_CONSECUTIVE_DETECTIONS) {
        if (!previous_detection) {
          if (start_column_counter) {
            column_counter++;
            if(column_counter >= 1 && traffic_permission && wtm_column_state == WTM_BETWEEN_COLUMNS) {
              add_log("Actual speed changed to 1 m/s at infeed");
              permitted_edge_speed = S1;
              beep_flag = true;
              start_column_counter = false;
              column_counter = 0;
              start_infeed_api_column_counter = true;
            }
          }
          previous_detection = true;
        }
      }
    } else {
      consecutive_detections = 0;
      previous_detection = false;
    }
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------------------- Destination Controller --------------------------------------------------------
    if (previous_station != current_station) {
      if(current_station == dropoff_station) {
        if (dropoff_station == "D18") {
          update_parcel_dropped_in_the_dump_api_flag = true;
        } else {
          update_parcel_dropped_api_flag = true;
        }
      }
    }
    
    if (columnDetected) {
      if(!previous_detection_for_infeed_api) {
        if (start_infeed_api_column_counter) {
          if (drive_motor_current_direction == DRIVE_MOTOR_FORWARD) {
            infeed_api_column_counter++;
          } else if (drive_motor_current_direction == DRIVE_MOTOR_REVERSE) {
            infeed_api_column_counter -= 2;
          }
          if(infeed_api_column_counter >= 10) {
            beep_flag = true;
            get_destination_api_flag = true;
            infeed_api_column_counter = 0;
            start_infeed_api_column_counter = false;
          }
        }
      }
    }
    previous_detection_for_infeed_api = columnDetected;
    // -------------------------------------------------------------------------------------------------------------------------------------------

    // --------------------------------------------------------------- AFC Controller ------------------------------------------------------------
    if (previous_station != current_station) {
      if(current_station == dropoff_station && dropoff_station != "X") {
        add_log("Opening the flap");
        OpenFlap();
        closing_flap = true;
        start_closing_time = millis();
      } else if (current_station == "I12") {
        add_log("Staging the flap");
        StageFlap();
      }
    }

    if (closing_flap && millis() - start_closing_time >= 1000) {
      add_log("Closing the flap");
      CloseFlap();
      closing_flap = false;
    }
    // -------------------------------------------------------------------------------------------------------------------------------------------

    previous_station = current_station;

    // Update drive motor state machine
    if (UpdateDriveMotorState() == DRIVE_MOTOR_STATE_CHANGED) {
      add_log("Drive motor state changed to: " + DriveMotorStateToString(drive_motor_current_state) + " from " + DriveMotorStateToString(drive_motor_previous_state));
      add_log("All shared variables - Traffic_Permission: " + String(traffic_permission) + ", Drive_Motor_Error_Status: " + String(drive_motor_error_status) + ", Global_Error_Status: " + String(global_error_status) + ", Current_Assignment: " + String(current_assignment) + ", Permitted_Edge_Speed: " + String(permitted_edge_speed) + ", column_detected_in_sweep: ," + String(column_detected_in_sweep));
      UpdateDriveMotorActuatorInputVariables();
    }
    
    DiverterStateChangeOptions state_change = UpdateDiverterState();
    if (state_change != DIVERTER_STATE_NO_CHANGE) Serial.println(state_change);
    if (state_change == DIVERTER_STATE_CHANGED) {
      add_log("Diverter state changed to: " + DiverterStateToString(diverter_current_state) + " from " + DiverterStateToString(diverter_previous_state));
      add_log("Variables - Global_Error: " + String(global_error_status) + ", Demand_Dir: " + DiverterDirectionToString(diverter_track_edge_direction) + ", Pos_HL: " + DiverterPositionToString(current_diverter_position_high_level) + ", Error_Status: " + String(error_diverter_status));
      add_log("Front current position: " + String(front_diverter_current_position) + ", Rear current position: " + String(rear_diverter_current_position));
      UpdateDiverterActuatorInputVariables();
    } else if (state_change == DIVERTER_UNEXPECTED_CONDITION) {
      add_log("Diverter State Machine in UNEXPECTED CONDITION!");
    }
  }
}

// Actuation Task
void ACTUATION_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms cycle time

    // Update motor speed if changed
    if (drive_motor_set_speed != drive_motor_previous_set_speed) {
      CANStatus success = setTargetVelocity(drive_motor_set_speed);
      if (success == CAN_OK) {
        int32_t getSetVelocity;
        if (getTargetVelocity(&getSetVelocity) == CAN_OK) {
          if (getSetVelocity == drive_motor_set_speed) {
            drive_motor_previous_set_speed = drive_motor_set_speed;
          }
        } else {
          add_log("Failed to verify drive motor speed setpoint.");
        }
      } else {
        add_log("Failed to update drive motor speed to " + String(drive_motor_set_speed));
        drive_motor_error_status = true;
      }
    }

    // Monitor drive motor current speed
    int32_t current_velocity;
    if (getVelocityActualValueAveraged(&current_velocity) == CAN_OK) {
      drive_motor_current_speed = current_velocity;
    } else {
      add_log("Failed to read drive motor actual speed.");
    }

    // Diverter actuation control - structured by enable state and direction
    if (!diverter_set_torque_flag) {
      // Torque disabled - handle disable sequence if state changed
      if (diverter_set_torque_flag != prev_diverter_set_torque_flag) {
        // Disable torque with retry mechanism
        delay(1);
        if (!enableTorqueFront(DISABLE)) {
          front_torque_disable_retry_count++;
          if (front_torque_disable_retry_count >= DIVERTER_MAX_RETRIES) {
            add_log("✗ Front torque disable failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
            error_front_servo = true;
            front_torque_disable_retry_count = 0;  // Reset for next attempt
          } else {
            add_log("⚠ Front torque disable retry " + String(front_torque_disable_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
          }
        } else {
          front_torque_disable_retry_count = 0;  // Reset on success
        }
        
        delay(1);
        if (!enableTorqueRear(DISABLE)) {
          rear_torque_disable_retry_count++;
          if (rear_torque_disable_retry_count >= DIVERTER_MAX_RETRIES) {
            add_log("✗ Rear torque disable failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
            error_rear_servo = true;
            rear_torque_disable_retry_count = 0;  // Reset for next attempt
          } else {
            add_log("⚠ Rear torque disable retry " + String(rear_torque_disable_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
          }
        } else {
          rear_torque_disable_retry_count = 0;  // Reset on success
        }
        
        // Reset commanded position tracking when torque disabled
        last_commanded_front_position = -1;
        last_commanded_rear_position = -1;
        prev_diverter_set_torque_flag = diverter_set_torque_flag;
      }
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      // Torque enabled - moving LEFT
      prev_diverter_set_torque_flag = diverter_set_torque_flag;
      
      if (front_diverter_set_position_value != -1 && rear_diverter_set_position_value != -1) {
        // Front servo - LEFT direction
        bool front_target_changed = (front_diverter_set_position_value != last_commanded_front_position);
        bool front_position_error = abs(front_diverter_current_position - front_diverter_left_threshold) > front_diverter_tolerance;
        
        if ((front_target_changed || front_position_error) && 
            front_diverter_current_position > (front_diverter_left_threshold + front_diverter_tolerance) &&
            (millis() - last_front_position_write_time >= POSITION_WRITE_COOLDOWN_MS)) {
          delay(1);
          if (!setPositionFront(front_diverter_set_position_value, DIVERTER_SPEED, DIVERTER_ACCELERATION)) {
            front_position_write_retry_count++;
            if (front_position_write_retry_count >= DIVERTER_MAX_RETRIES) {
              add_log("✗ Front position write failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
              error_front_servo = true;
              front_position_write_retry_count = 0;
            } else {
              add_log("⚠ Front position write retry " + String(front_position_write_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
            }
          } else {
            last_commanded_front_position = front_diverter_set_position_value;
            last_front_position_write_time = millis();
            front_position_write_retry_count = 0;
          }
        }
        
        // Rear servo - LEFT direction
        bool rear_target_changed = (rear_diverter_set_position_value != last_commanded_rear_position);
        bool rear_position_error = abs(rear_diverter_current_position - rear_diverter_left_threshold) > rear_diverter_tolerance;
        
        if ((rear_target_changed || rear_position_error) && 
            rear_diverter_current_position < (rear_diverter_left_threshold - rear_diverter_tolerance) &&
            (millis() - last_rear_position_write_time >= POSITION_WRITE_COOLDOWN_MS)) {
          delay(1);
          if (!setPositionRear(rear_diverter_set_position_value, DIVERTER_SPEED, DIVERTER_ACCELERATION)) {
            rear_position_write_retry_count++;
            if (rear_position_write_retry_count >= DIVERTER_MAX_RETRIES) {
              add_log("✗ Rear position write failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
              error_rear_servo = true;
              rear_position_write_retry_count = 0;
            } else {
              add_log("⚠ Rear position write retry " + String(rear_position_write_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
            }
          } else {
            last_commanded_rear_position = rear_diverter_set_position_value;
            last_rear_position_write_time = millis();
            rear_position_write_retry_count = 0;
          }
        }
      }
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      // Torque enabled - moving RIGHT
      prev_diverter_set_torque_flag = diverter_set_torque_flag;
      
      if (front_diverter_set_position_value != -1 && rear_diverter_set_position_value != -1) {
        // Front servo - RIGHT direction
        bool front_target_changed = (front_diverter_set_position_value != last_commanded_front_position);
        bool front_position_error = abs(front_diverter_current_position - front_diverter_right_threshold) > front_diverter_tolerance;
        
        if ((front_target_changed || front_position_error) && 
            front_diverter_current_position < (front_diverter_right_threshold - front_diverter_tolerance) &&
            (millis() - last_front_position_write_time >= POSITION_WRITE_COOLDOWN_MS)) {
          delay(1);
          if (!setPositionFront(front_diverter_set_position_value, DIVERTER_SPEED, DIVERTER_ACCELERATION)) {
            front_position_write_retry_count++;
            if (front_position_write_retry_count >= DIVERTER_MAX_RETRIES) {
              add_log("✗ Front position write failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
              error_front_servo = true;
              front_position_write_retry_count = 0;
            } else {
              add_log("⚠ Front position write retry " + String(front_position_write_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
            }
          } else {
            last_commanded_front_position = front_diverter_set_position_value;
            last_front_position_write_time = millis();
            front_position_write_retry_count = 0;
          }
        }
        
        // Rear servo - RIGHT direction
        bool rear_target_changed = (rear_diverter_set_position_value != last_commanded_rear_position);
        bool rear_position_error = abs(rear_diverter_current_position - rear_diverter_right_threshold) > rear_diverter_tolerance;
        
        if ((rear_target_changed || rear_position_error) && 
            rear_diverter_current_position > (rear_diverter_right_threshold + rear_diverter_tolerance) &&
            (millis() - last_rear_position_write_time >= POSITION_WRITE_COOLDOWN_MS)) {
          delay(1);
          if (!setPositionRear(rear_diverter_set_position_value, DIVERTER_SPEED, DIVERTER_ACCELERATION)) {
            rear_position_write_retry_count++;
            if (rear_position_write_retry_count >= DIVERTER_MAX_RETRIES) {
              add_log("✗ Rear position write failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
              error_rear_servo = true;
              rear_position_write_retry_count = 0;
            } else {
              add_log("⚠ Rear position write retry " + String(rear_position_write_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
            }
          } else {
            last_commanded_rear_position = rear_diverter_set_position_value;
            last_rear_position_write_time = millis();
            rear_position_write_retry_count = 0;
          }
        }
      }
    }
    
    // Read positions periodically
    unsigned long current_time = millis();
    if (current_time - last_position_read_time >= POSITION_READ_INTERVAL) {
      delay(1);
      int16_t front_pos = readPositionFront();
      if (front_pos == -1) {
        front_position_read_retry_count++;
        if (front_position_read_retry_count >= DIVERTER_MAX_RETRIES) {
          add_log("✗ Front position read failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
          error_front_servo = true;
          front_position_read_retry_count = 0;  // Reset for next attempt
        } else {
          add_log("⚠ Front position read retry " + String(front_position_read_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
        }
      } else {
        front_diverter_current_position = front_pos;
        error_front_servo = false;  // Clear error on successful read
        front_position_read_retry_count = 0;  // Reset on success
      }
      
      delay(1);
      int16_t rear_pos = readPositionRear();
      if (rear_pos == -1) {
        rear_position_read_retry_count++;
        if (rear_position_read_retry_count >= DIVERTER_MAX_RETRIES) {
          add_log("✗ Rear position read failed after " + String(DIVERTER_MAX_RETRIES) + " retries");
          error_rear_servo = true;
          rear_position_read_retry_count = 0;  // Reset for next attempt
        } else {
          add_log("⚠ Rear position read retry " + String(rear_position_read_retry_count) + "/" + String(DIVERTER_MAX_RETRIES));
        }
      } else {
        rear_diverter_current_position = rear_pos;
        error_rear_servo = false;  // Clear error on successful read
        rear_position_read_retry_count = 0;  // Reset on success
      }
      
      UpdateDiverterPositionHighLevel();
      last_position_read_time = current_time;
    }

    if (beep_flag) {
      Beep();
      beep_flag = false;
    }
  }
}

// API Handling Task
void API_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (get_destination_api_flag) {
      GetDestinationAndJourneyPath();
      clear_infeed_api_flag = true;
      get_destination_api_flag = false;
    }

    if (clear_infeed_api_flag) {
      ClearInfeedStation();
      clear_infeed_api_flag = false;
    }

    if (update_parcel_dropped_in_the_dump_api_flag) {
      UpdateParcelDroppedInTheDump();
      update_parcel_dropped_in_the_dump_api_flag = false;
      dropoff_station = "X";
    }

    if (update_parcel_dropped_api_flag) {
      UpdateParcelDropped();
      update_parcel_dropped_api_flag = false;
      dropoff_station = "X";
    }
  }
}

void checkAndUpdateFirmware() {
  Serial.println("\n--- Checking firmware version ---");
  
  // Step 1: Check firmware version with server
  String firmwareStatus = checkFirmwareVersion();
  
  Serial.print("Firmware status: ");
  Serial.println(firmwareStatus);
  
  // Step 2: If update required, download and install
  if (firmwareStatus == "update-needed") {
    Serial.println("Update required! Starting download...");
    
    bool success = downloadAndInstallFirmware();
    
    if (success) {
      Serial.println("Firmware update successful! Rebooting...");
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("Firmware update failed. Continuing with current version.");
    }
  } else if (firmwareStatus == "up-to-date") {
    Serial.println("Firmware is up to date. No update needed.");
  } else if (firmwareStatus == "NO_RELEASE_SET") {
    Serial.println("No release configured on server. Skipping update.");
  } else {
    Serial.println("Unknown status. Skipping update.");
  }
}

String checkFirmwareVersion() {
  HTTPClient http;
  // ?deviceRole=bot_primary&deviceId=B7&fileName=11
  // Build URL with query parameters
  String url = String(DMS_URL_PREFIX) + checkVersionEndpoint + 
               "?deviceRole=" + deviceRole +
               "&deviceId=" + BOT_ID + "&fileName=" + FIRMWARE_ID ;
  
  Serial.print("Calling: ");
  Serial.println(url);
  
  http.begin(url);
  http.setTimeout(10000);  // 10 second timeout
  
  int httpCode = http.GET();
  String status = "ERROR";
  
  if (httpCode == 200) {
    String response = http.getString();
    Serial.print("Response: ");
    Serial.println(response);
    
    // Parse JSON response
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, response);
    
    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      status = "ERROR";
    } else {
      // Extract status attribute from JSON
      if (doc.containsKey("status")) {
        status = doc["status"].as<String>();
      } else {
        Serial.println("No 'status' field in JSON response");
        status = "ERROR";
      }
    }
  } else {
    Serial.print("HTTP Error: ");
    Serial.println(httpCode);
    Serial.println(http.getString());
  }
  
  http.end();
  return status;
}

bool downloadAndInstallFirmware() {
  int retryCount = 0;
  int retryDelay = initialRetryDelay;
  
  while (retryCount < maxRetries) {
    Serial.printf("\n--- Download attempt %d/%d ---\n", retryCount + 1, maxRetries);
    
    HTTPClient http;
    
    // Build URL
    String url = String(DMS_URL_PREFIX) + updateFirmwareEndpoint + 
                 "?deviceId=" + BOT_ID +
                 "&deviceRole=" + deviceRole;
    
    Serial.print("Downloading from: ");
    Serial.println(url);
    
    http.begin(url);
    http.setTimeout(600000);  // 10 minute timeout for large file
    
    int httpCode = http.GET();
    
    if (httpCode == 200) {
      int contentLength = http.getSize();
      Serial.printf("Firmware size: %d bytes (%.2f MB)\n", contentLength, contentLength / 1048576.0);
      
      bool canBegin = Update.begin(contentLength);
      
      if (canBegin) {
        WiFiClient* stream = http.getStreamPtr();
        
        size_t written = Update.writeStream(*stream);
        
        Serial.printf("Downloaded: %d bytes\n", written);
        
        if (written == contentLength) {
          Serial.println("Download complete!");
        } else {
          Serial.printf("Download incomplete: %d / %d bytes\n", written, contentLength);
        }
        
        if (Update.end()) {
          if (Update.isFinished()) {
            Serial.println("Update successfully completed!");
            http.end();
            return true;
          } else {
            Serial.println("Update not finished. Something went wrong!");
          }
        } else {
          Serial.printf("Update error: %d\n", Update.getError());
        }
      } else {
        Serial.println("Not enough space to begin OTA update!");
      }
      
    } else if (httpCode == 429) {
      // Server throttling - retry after delay
      Serial.println("Server busy (429). Retrying...");
      
    } else {
      Serial.printf("HTTP Error: %d\n", httpCode);
    }
    
    http.end();
    
    // Retry with exponential backoff
    retryCount++;
    if (retryCount < maxRetries) {
      Serial.printf("Retrying in %d seconds...\n", retryDelay / 1000);
      delay(retryDelay);
      retryDelay = min(retryDelay * 2, maxRetryDelay);  // Exponential backoff
    }
  }
  
  Serial.println("Max retries reached. Update failed.");
  return false;
}

//=============================================================================
// MAIN SETUP FUNCTION
// Initialization sequence critical for proper operation - DO NOT REORDER
//=============================================================================
void setup() {
  // 1. Initialize Serial for debugging (FIRST)
  Serial.begin(115200);
  Serial.println(__FILE__);
  delay(500);  // Short delay for hardware stabilization
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   BOT PRIMARY FIRMWARE v1.1.0          ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // 2. Initialize mutex for thread-safe logging (BEFORE any add_log() calls)
  xDebugLogMutex = xSemaphoreCreateMutex();
  if (xDebugLogMutex == NULL) {
    Serial.println("✗ FATAL: Failed to create debug log mutex");
    while(1) delay(1000);
  }
  Serial.println("✓ Debug log mutex created");
  
  // 3. Load NVM configuration (BEFORE using BOT_ID or servo limits)
  LoadDiverterConfig();
  Serial.println("✓ NVM config loaded");
  
  // 4. Initialize debug string (AFTER loading BOT_ID from NVM)
  debugLoggingString = BOT_ID + " " + FIRMWARE_ID + ": ";
  add_log("System startup");
  
  // 5. Connect to WiFi (uses add_log, needs mutex and BOT_ID)
  WiFiConfig();

  // 6a. Configure MCP2515 CAN controller
  MCPConfig();
  
  // 6b. Configure GPIO pins
  PinConfig();
  Serial.println("✓ GPIO pins configured");
  
  // 7a. Beep to indicate initialization started (if buzzer available)
  Beep();

  // 7b.
  CloseFlap();
  
  // 8. Motor power-up delay (BEFORE CAN/Motor init)
  Serial.println("⏳ Waiting for motor/driver power-up...");
  Serial.print("   ");
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println("Ready!\n");
  
  // 9. Initialize Serial2 for servos (AFTER power delay)
  Serial2.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);  // Allow UART to stabilize
  Serial.println("✓ Serial2 initialized for servos");
  
  // 10. Initialize CAN
  Serial.println("→ Initializing CAN (500 kbit/s)...");
  add_log("→ CAN init");
  if (!CAN_Init()) {  // Uncomment when CAN_Init is available
    add_log("✗ FATAL: CAN init failed");
    Serial.println("✗ FATAL: CAN initialization failed");
    while (1) delay(1000);
  }
  add_log("✓ CAN initialized");
  Serial.println("✓ CAN initialized (TX: GPIO 14, RX: GPIO 15)\n");
  
  // 11. Configure motor
  if (!MotorConfig()) {  // Uncomment when MotorConfig is available
    add_log("✗ FATAL: Motor config failed");
    Serial.println("\n✗ FATAL: Motor configuration failed");
    Serial.println("   System halted. Fix errors and reset.\n");
    while (1) delay(1000);
  }
  
  // 12. Set initial motor speed (if applicable)
  drive_motor_set_speed = S0_5;  // Uncomment when variable is available

  checkAndUpdateFirmware();
  
  // 13. System ready
  Beep();  // Uncomment when available
  add_log("Bot:" + BOT_ID + " Firmware:" + FIRMWARE_ID);
  Serial.println("Bot: " + BOT_ID + " Firmware: " + FIRMWARE_ID);
  add_log("Starting the bot!");
  
  // 14. Create RTOS Tasks
  // HTTP debug logger task on Core 0 (Priority 2) - LOW priority for background logging
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "http_debug_logger",
    10240,  // 10KB stack for HTTPClient operations (needs buffer for HTTP headers/body)
    NULL,
    2,     // Priority 2 (lower than control tasks)
    &http_debug_log_task,
    0);    // Core 0
  
  Serial.println("✓ HTTP Debug Logger task created on Core 0");
  
  // Sensor reading and state machine task on Core 1 (Priority 5) - HIGH priority
  xTaskCreatePinnedToCore(  // Uncomment when task function is available
    READING_SENSOR_AND_UPDATE_STATE_TASK,
    "reading_sensor_and_update_state_task",
    8192,  // 8KB stack for sensor reading, station code processing, state machines
    NULL,
    5,
    &reading_sensor_and_update_state_task,
    1);
  
  vTaskDelay(pdMS_TO_TICKS(20));  // Small delay between task creations
  
  // Actuation control task on Core 1 (Priority 4) - MEDIUM-HIGH priority
  xTaskCreatePinnedToCore(  // Uncomment when task function is available
    ACTUATION_TASK,
    "actuation_task",
    6144,  // 6KB stack for CAN SDO, Modbus RTU, retry mechanisms, position control
    NULL,
    4,
    &actuation_task,
    1);

  vTaskDelay(pdMS_TO_TICKS(20));
  
  // API handling task on Core 0 (Priority 3) - MEDIUM priority
  xTaskCreatePinnedToCore(
    API_TASK,
    "api_task",
    4096,
    NULL,
    3,
    &api_task,
    0);
  
  Serial.println("✓ Initialization complete\n");
}

void loop() {
  // Empty - all functionality handled by RTOS tasks
  delay(1000); 
}

// =============================================================================
// WTM COLUMN HEALTH ANALYSIS FUNCTIONS
// =============================================================================

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

// Analyze column detection and report health
HealthStatus AnalyzeWTMColumnHealth() {
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
  majorityHealthWTM = CalculateMajorityHealth(majorityCount, DEFAULT_COLUMN_FRAME_THRESHOLD);
  
  // Minority: Use total frames captured from previous empty space to current empty space
  minorityHealthWTM = CalculateMinorityHealth(minorityTotal, totalFramesBetweenSpaces);
  overallHealthWTM = GetWorstHealth(majorityHealthWTM, minorityHealthWTM);
  
  // Compact log for memory efficiency
  if (overallHealthWTM == CRITICAL) add_log("C" + String(totalColumnsDetected) + "[" + detectedColumnCode + "]:" + String(totalFramesBetweenSpaces) + "f(" + String(frameCount01) + "," + String(frameCount10) + "," + String(frameCount11) + ") " + HealthToString(overallHealthWTM));
  return overallHealthWTM;
}

//=============================================================================
// DIAGNOSTIC FUNCTIONS - DRIVE MOTOR
//=============================================================================

// Diagnose SDO abort codes (CANopen standard + EPOS specific)
void diagnoseSDOAbort(uint32_t abort_code, uint16_t index) {
  String log_msg = "SDO_ABORT:0x" + String(abort_code, HEX) + "@0x" + String(index, HEX);
  add_log(log_msg);
  
  Serial.print("\n┌─ SDO ABORT DIAGNOSTIC ─────────────────");
  Serial.print("\n│ Abort Code: 0x");
  Serial.println(abort_code, HEX);
  Serial.print("│ Object Index: 0x");
  Serial.println(index, HEX);
  Serial.println("│");
  
  switch (abort_code) {
    // Standard CANopen abort codes
    case 0x05030000:
      add_log("│ Error: Toggle bit not alternated");
      add_log("│ Cause: SDO protocol error");
      add_log("│ Action: Retry command, check CAN bus integrity");
      break;
      
    case 0x05040000:
      add_log("│ Error: SDO protocol timed out");
      add_log("│ Cause: EPOS didn't respond in time");
      add_log("│ Action: Check EPOS power, CAN connection, node ID");
      break;
      
    case 0x05040001:
      add_log("│ Error: Client/server command specifier not valid");
      add_log("│ Cause: Invalid SDO command byte");
      add_log("│ Action: Check SDO frame format (0x22 for write, 0x40 for read)");
      break;
      
    case 0x06010000:
      add_log("│ Error: Unsupported access to an object");
      add_log("│ Cause: Object cannot be accessed (read-only, write-only, etc.)");
      add_log("│ Action: Check EPOS documentation for object access rights");
      break;
      
    case 0x06010001:
      add_log("│ Error: Attempt to read a write-only object");
      add_log("│ Cause: Trying to read from write-only parameter");
      add_log("│ Action: Use correct access direction for this object");
      break;
      
    case 0x06010002:
      add_log("│ Error: Attempt to write a read-only object");
      add_log("│ Cause: Trying to write to read-only parameter");
      add_log("│ Action: Use correct access direction for this object");
      break;
      
    case 0x06020000:
      add_log("│ Error: Object does not exist in object dictionary");
      add_log("│ Cause: Invalid object index or not supported by this EPOS");
      add_log("│ Action: Check object index, verify EPOS firmware version");
      break;
      
    case 0x06040041:
      add_log("│ Error: Object cannot be mapped to PDO");
      add_log("│ Cause: This object doesn't support PDO mapping");
      add_log("│ Action: Use SDO access instead of PDO");
      break;
      
    case 0x06040042:
      add_log("│ Error: PDO length exceeded");
      add_log("│ Cause: Too many objects mapped to PDO");
      add_log("│ Action: Reduce PDO mapping or split into multiple PDOs");
      break;
      
    case 0x06040043:
      add_log("│ Error: General parameter incompatibility");
      add_log("│ Cause: Parameter conflict or invalid configuration");
      add_log("│ Action: Check parameter dependencies and valid ranges");
      break;
      
    case 0x06040047:
      add_log("│ Error: General internal incompatibility");
      add_log("│ Cause: Internal EPOS state prevents this operation");
      add_log("│ Action: Check EPOS state machine, may need reset");
      break;
      
    case 0x06060000:
      add_log("│ Error: Access failed due to hardware error");
      add_log("│ Cause: Hardware fault detected");
      add_log("│ Action: Check motor connections, power supply, encoder");
      break;
      
    case 0x06070010:
      add_log("│ Error: Data type does not match");
      add_log("│ Cause: Wrong data size for this object");
      add_log("│ Action: Check object data type (16-bit vs 32-bit)");
      break;
      
    case 0x06070012:
      add_log("│ Error: Data type length too high");
      add_log("│ Cause: Sending too many data bytes");
      add_log("│ Action: Use correct SDO command specifier (0x22 vs 0x23)");
      break;
      
    case 0x06070013:
      add_log("│ Error: Data type length too low");
      add_log("│ Cause: Not enough data bytes sent");
      add_log("│ Action: Use correct SDO command specifier");
      break;
      
    case 0x06090011:
      add_log("│ Error: Sub-index does not exist");
      add_log("│ Cause: Invalid subindex for this object");
      add_log("│ Action: Check object dictionary for valid subindices");
      break;
      
    case 0x06090030:
      add_log("│ Error: Value range exceeded");
      add_log("│ Cause: Parameter value out of valid range");
      add_log("│ Action: Check min/max limits in EPOS documentation");
      if (index == TARGET_VELOCITY) {
        add_log("│ Note: Target velocity may exceed configured limits");
        add_log("│       Check Max Profile Velocity (0x607F)");
      }
      break;
      
    case 0x06090031:
      add_log("│ Error: Value too high");
      add_log("│ Cause: Parameter exceeds maximum limit");
      add_log("│ Action: Reduce value or increase configured maximum");
      break;
      
    case 0x06090032:
      add_log("│ Error: Value too low");
      add_log("│ Cause: Parameter below minimum limit");
      add_log("│ Action: Increase value or decrease configured minimum");
      break;
      
    case 0x08000000:
      add_log("│ Error: General error");
      add_log("│ Cause: Unspecified error condition");
      add_log("│ Action: Check EPOS error register, may need reset");
      break;
      
    case 0x08000020:
      add_log("│ Error: Data cannot be transferred or stored");
      add_log("│ Cause: EPOS cannot process this command now");
      add_log("│ Action: Check EPOS state, wait for current operation to finish");
      break;
      
    case 0x08000021:
      add_log("│ Error: Data cannot be transferred (local control)");
      add_log("│ Cause: EPOS in local control mode");
      add_log("│ Action: Disable local control, switch to remote mode");
      break;
      
    case 0x08000022:
      add_log("│ Error: Data cannot be transferred (device state)");
      add_log("│ Cause: EPOS not in correct state for this operation");
      add_log("│ Action: Check state machine, may need to enable device first");
      if (index == TARGET_VELOCITY || index == CONTROLWORD) {
        add_log("│ Note: Motor may not be enabled - try shutdown() then enable()");
      }
      break;
      
    // EPOS-specific abort codes
    case 0x0F00FFC0:
      add_log("│ Error: EPOS password protected");
      add_log("│ Cause: Object requires password to access");
      add_log("│ Action: Unlock EPOS using password command");
      break;
      
    default:
      add_log("│ Error: Unknown abort code");
      add_log("│ Cause: Undocumented or vendor-specific error");
      add_log("│ Action: Check EPOS manual, consider firmware update");
      break;
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

// Diagnose TWAI alerts
void diagnoseTWAIAlert(uint32_t alerts) {
  // Log critical alerts only
  if (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR)) {
    add_log("TWAI_ALERT:0x" + String(alerts, HEX));
  }
  
  Serial.println("\n┌─ TWAI ALERT DIAGNOSTIC ────────────────");
  Serial.print("│ Alert Flags: 0x");
  Serial.println(alerts, HEX);
  Serial.println("│");
  
  if (alerts & TWAI_ALERT_TX_IDLE) {
    add_log("│ ℹ TX Idle: No transmission in progress");
  }
  
  if (alerts & TWAI_ALERT_TX_SUCCESS) {
    add_log("│ ✓ TX Success: Frame transmitted successfully");
  }
  
  if (alerts & TWAI_ALERT_RX_DATA) {
    add_log("│ ℹ RX Data: Frame received");
  }
  
  if (alerts & TWAI_ALERT_BELOW_ERR_WARN) {
    add_log("│ ✓ Below Error Warning: Bus healthy");
  }
  
  if (alerts & TWAI_ALERT_ERR_ACTIVE) {
    add_log("│ ⚠ Error Active: Error counters elevated");
    add_log("│ Cause: Occasional transmission errors");
    add_log("│ Action: Check CAN bus termination, wiring, baud rate");
  }
  
  if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    add_log("│ ⚠ Recovery in Progress: Attempting bus-off recovery");
    add_log("│ Cause: Controller was in bus-off, now recovering");
    add_log("│ Action: Wait for recovery, check bus health");
  }
  
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    add_log("│ ✓ Bus Recovered: Successfully recovered from bus-off");
  }
  
  if (alerts & TWAI_ALERT_ARB_LOST) {
    add_log("│ ⚠ Arbitration Lost: Lost bus arbitration");
    add_log("│ Cause: Another node transmitted at same time with higher priority");
    add_log("│ Action: Normal in multi-node networks, will retry automatically");
  }
  
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
    add_log("│ ⚠ Above Error Warning: Error counters high (>96)");
    add_log("│ Cause: Frequent bus errors, nearing bus-off");
    add_log("│ Action: URGENT - Check termination, baud rate, wiring quality");
  }
  
  if (alerts & TWAI_ALERT_BUS_ERROR) {
    add_log("│ ✗ Bus Error: CAN protocol error detected");
    add_log("│ Cause: Bit/stuff/CRC/form/ACK error");
    add_log("│ Action: Check bus quality, termination, EMI");
  }
  
  if (alerts & TWAI_ALERT_TX_FAILED) {
    add_log("│ ✗ TX Failed: Transmission failed after retries");
    add_log("│ Cause: No ACK from any node, or bus-off");
    add_log("│ Action: Check if other nodes present, check EPOS power");
  }
  
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    add_log("│ ✗ RX Queue Full: Receive buffer overflow");
    add_log("│ Cause: Not reading frames fast enough");
    add_log("│ Action: Increase RX queue size or read more frequently");
  }
  
  if (alerts & TWAI_ALERT_ERR_PASS) {
    add_log("│ ✗ Error Passive: Error counters critical (>127)");
    add_log("│ Cause: Many bus errors, one step from bus-off");
    add_log("│ Action: CRITICAL - Fix bus issues immediately");
    add_log("│        Check: termination resistors (120Ω), baud rate match,");
    add_log("│               cable quality, EMI shielding, ground loops");
  }
  
  if (alerts & TWAI_ALERT_BUS_OFF) {
    add_log("│ ✗✗✗ BUS-OFF: Controller shut down due to errors");
    add_log("│ Cause: Error count exceeded 255, bus completely unreliable");
    add_log("│ Action: CRITICAL FAILURE");
    add_log("│   1. Check CAN transceiver power and connections");
    add_log("│   2. Verify 120Ω termination at both bus ends");
    add_log("│   3. Confirm baud rate matches all nodes (500 kbit/s)");
    add_log("│   4. Check CANH/CANL not swapped or shorted");
    add_log("│   5. Measure bus voltage (recessive: 2.5V, dominant: 1.5V/3.5V)");
    add_log("│   6. Will auto-recover, but root cause MUST be fixed");
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

// Diagnose ESP_ERR codes
void diagnoseESPError(esp_err_t err, const char* operation) {
  if (err != ESP_OK) {
    add_log("ESP_ERR:" + String(err) + "@" + String(operation));
  }
  
  Serial.print("\n┌─ ESP ERROR DIAGNOSTIC ─────────────────");
  Serial.print("\n│ Operation: ");
  add_log(operation);
  Serial.print("│ Error Code: ");
  Serial.println(err);
  Serial.println("│");
  
  switch (err) {
    case ESP_OK:
      add_log("│ Status: Success");
      break;
      
    case ESP_ERR_TIMEOUT:
      add_log("│ Error: Timeout");
      add_log("│ Cause: Operation didn't complete in time");
      if (strcmp(operation, "twai_transmit") == 0) {
        add_log("│ Details: TX queue full or bus stuck");
        add_log("│ Action: Check if EPOS responding, verify bus not saturated");
      } else if (strcmp(operation, "twai_receive") == 0) {
        add_log("│ Details: No frame received within timeout");
        add_log("│ Action: Normal if no traffic, check if EPOS responding");
      }
      break;
      
    case ESP_ERR_INVALID_STATE:
      add_log("│ Error: Invalid State");
      add_log("│ Cause: TWAI driver not started or in wrong mode");
      add_log("│ Action: Check twai_start() was called, not in bus-off");
      break;
      
    case ESP_ERR_INVALID_ARG:
      add_log("│ Error: Invalid Argument");
      add_log("│ Cause: Bad parameter passed to TWAI function");
      add_log("│ Action: Check message structure (ID, DLC, data)");
      break;
      
    case ESP_FAIL:
      add_log("│ Error: General Failure");
      add_log("│ Cause: Unspecified error in TWAI driver");
      add_log("│ Action: Check TWAI initialization, may need restart");
      break;
      
    case ESP_ERR_NOT_SUPPORTED:
      add_log("│ Error: Not Supported");
      add_log("│ Cause: Feature not available on this ESP32 variant");
      add_log("│ Action: Check ESP32 model supports TWAI/CAN");
      break;
      
    default:
      Serial.print("│ Error: Unknown (");
      Serial.print(err);
      add_log(")");
      add_log("│ Action: Check ESP-IDF documentation");
      break;
  }
  
  Serial.println("└────────────────────────────────────────\n");
}

//=============================================================================
// CAN/TWAI LOW-LEVEL COMMUNICATION
//=============================================================================

// Initialize CAN hardware (500 kbit/s with full alert monitoring)
bool CAN_Init() {
  Serial.println("\n┌─ CAN INITIALIZATION ──────────────────");
  add_log("CAN init start");
  
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN, 
    CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  
  // Enable all alerts for comprehensive monitoring
  g_config.alerts_enabled = TWAI_ALERT_ALL;
  
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  Serial.println("│ Config: 500 kbit/s, Normal Mode");
  Serial.print("│ TX Pin: GPIO");
  Serial.println(CAN_TX_PIN);
  Serial.print("│ RX Pin: GPIO");
  Serial.println(CAN_RX_PIN);
  Serial.println("│");
  
  // Install TWAI driver
  Serial.println("│ Installing TWAI driver...");
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.println("│ ✗ Driver install FAILED");
    add_log("✗ CAN driver install failed: " + String(err));
    diagnoseESPError(err, "twai_driver_install");
    Serial.println("└──────────────────────────────────────────\n");
    return false;
  }
  Serial.println("│ ✓ Driver installed");
  add_log("✓ CAN driver installed");
  
  // Start TWAI driver
  Serial.println("│ Starting TWAI driver...");
  err = twai_start();
  if (err != ESP_OK) {
    Serial.println("│ ✗ Driver start FAILED");
    add_log("✗ CAN start failed: " + String(err));
    diagnoseESPError(err, "twai_start");
    Serial.println("└──────────────────────────────────────────\n");
    return false;
  }
  
  Serial.println("│ ✓ Driver started");
  Serial.println("│ ✓ CAN bus ready for communication");
  Serial.println("└──────────────────────────────────────────\n");
  add_log("✓ CAN initialized @ 500kbps");
  
  return true;
}

// Check for TWAI alerts
void checkTWAIAlerts() {
  uint32_t alerts;
  esp_err_t err = twai_read_alerts(&alerts, 0);  // Non-blocking check
  
  if (err == ESP_OK && alerts != 0) {
    diagnoseTWAIAlert(alerts);
    
    // Auto-recovery from bus-off
    if (alerts & TWAI_ALERT_BUS_OFF) {
      add_log("⚠ BUS-OFF recovery");
      Serial.println("⚠ Initiating bus-off recovery...");
      twai_initiate_recovery();
    }
  }
}

// Send a CAN frame with error diagnostics
CANStatus CAN_Send(SimpleCANFrame* frame, uint32_t timeout_ms) {
  twai_message_t tx_msg;
  
  tx_msg.identifier       = frame->id;
  tx_msg.data_length_code = frame->dlc;
  tx_msg.extd             = frame->extended ? 1 : 0;
  tx_msg.rtr              = frame->rtr ? 1 : 0;
  
  for (int i = 0; i < frame->dlc; i++) {
    tx_msg.data[i] = frame->data[i];
  }
  
  esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(timeout_ms));
  
  if (result == ESP_OK) {
    return CAN_OK;
  } else {
    // Log transmission failure with frame details
    String frame_dump = "TX_FAIL ID:0x" + String(frame->id, HEX) + " DLC:" + String(frame->dlc) + " [";
    for (int i = 0; i < frame->dlc; i++) {
      if (i > 0) frame_dump += " ";
      if (frame->data[i] < 16) frame_dump += "0";
      frame_dump += String(frame->data[i], HEX);
    }
    frame_dump += "]";
    add_log(frame_dump);
    
    diagnoseESPError(result, "twai_transmit");
    checkTWAIAlerts();  // Check for bus issues
    
    if (result == ESP_ERR_TIMEOUT) {
      add_log("TX timeout - queue full or bus stuck");
      return CAN_TX_TIMEOUT;
    } else {
      add_log("TX error: " + String(result));
      return CAN_ERROR;
    }
  }
}

// Receive a CAN frame with error diagnostics
CANStatus CAN_Receive(SimpleCANFrame* frame, uint32_t timeout_ms) {
  twai_message_t rx_msg;
  
  esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(timeout_ms));
  
  if (result == ESP_OK) {
    frame->id       = rx_msg.identifier;
    frame->dlc      = rx_msg.data_length_code;
    frame->extended = rx_msg.extd;
    frame->rtr      = rx_msg.rtr;
    
    for (int i = 0; i < rx_msg.data_length_code && i < 8; i++) {
      frame->data[i] = rx_msg.data[i];
    }
    
    return CAN_OK;
  } else if (result == ESP_ERR_TIMEOUT) {
    return CAN_RX_TIMEOUT;
  } else {
    diagnoseESPError(result, "twai_receive");
    return CAN_ERROR;
  }
}

//=============================================================================
// CANOPEN SDO PROTOCOL (MOTOR COMMUNICATION)
//=============================================================================

// SDO Write (Download): Send parameter to EPOS motor controller
CANStatus SDO_Write(uint16_t index, uint8_t subindex, uint32_t value) {
  // Log SDO write request
  add_log("SDO_WR 0x" + String(index, HEX) + "." + String(subindex, HEX) + "=0x" + String(value, HEX));
  
  SimpleCANFrame tx_frame;
  
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x22;
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = (index >> 8) & 0xFF;
  tx_frame.data[3] = subindex;
  tx_frame.data[4] = value & 0xFF;
  tx_frame.data[5] = (value >> 8) & 0xFF;
  tx_frame.data[6] = (value >> 16) & 0xFF;
  tx_frame.data[7] = (value >> 24) & 0xFF;
  
  // Send request
  CANStatus status = CAN_Send(&tx_frame, 4);
  if (status != CAN_OK) {
    add_log("✗ TX failed 0x" + String(index, HEX) + " (status:" + String(status) + ")");
    Serial.print("✗ TX failed for object 0x");
    Serial.print(index, HEX);
    Serial.print(" status: ");
    Serial.println(status);
    return status;
  }
  
  // Wait for SDO response
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < SDO_RESPONSE_TIMEOUT) {
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      if (rx_frame.id == SDO_RX) {
        uint8_t cmd = rx_frame.data[0];
        
        // SDO abort (0x80)
        if (cmd == 0x80) {
          uint32_t abort_code = rx_frame.data[4] 
                              | (rx_frame.data[5] << 8)
                              | (rx_frame.data[6] << 16)
                              | (rx_frame.data[7] << 24);
          add_log("✗ SDO_ABORT 0x" + String(index, HEX) + " code:0x" + String(abort_code, HEX));
          diagnoseSDOAbort(abort_code, index);
          return CAN_SDO_ABORT;
        }
        
        // Download response (0x60)
        if (cmd == 0x60) {
          add_log("✓ SDO_WR_OK 0x" + String(index, HEX));
          return CAN_OK;
        }
      }
    }
  }
  
  // Timeout - log detailed context
  add_log("✗ SDO timeout 0x" + String(index, HEX) + " after " + String(SDO_RESPONSE_TIMEOUT) + "ms");
  Serial.print("✗ SDO response timeout for object 0x");
  Serial.print(index, HEX);
  Serial.println();
  Serial.println("│ Request: Write 0x" + String(value, HEX) + " to [0x" + String(index, HEX) + "." + String(subindex, HEX) + "]");
  Serial.println("│ No response received within " + String(SDO_RESPONSE_TIMEOUT) + " ms");
  Serial.println("│ Possible causes:");
  Serial.println("│   - EPOS not powered or not responding");
  Serial.println("│   - Wrong node ID (check NODE_ID = 1)");
  Serial.println("│   - CAN bus disconnected or terminated incorrectly");
  Serial.println("│   - EPOS in fault state (check error LED)");
  Serial.println("│   - Baud rate mismatch (should be 500 kbit/s)");
  checkTWAIAlerts();
  
  return CAN_RX_TIMEOUT;
}

// Send SDO Upload (read) and wait for response
CANStatus SDO_Read(uint16_t index, uint8_t subindex, uint32_t* value) {
  // Log SDO read request (commented out to avoid log spam on frequent reads)
  // add_log("SDO_RD 0x" + String(index, HEX) + "." + String(subindex, HEX));
  
  SimpleCANFrame tx_frame;
  
  tx_frame.id       = SDO_TX;
  tx_frame.dlc      = 8;
  tx_frame.extended = false;
  tx_frame.rtr      = false;
  
  tx_frame.data[0] = 0x40;
  tx_frame.data[1] = index & 0xFF;
  tx_frame.data[2] = (index >> 8) & 0xFF;
  tx_frame.data[3] = subindex;
  tx_frame.data[4] = 0x00;
  tx_frame.data[5] = 0x00;
  tx_frame.data[6] = 0x00;
  tx_frame.data[7] = 0x00;
  
  CANStatus status = CAN_Send(&tx_frame, 4);
  if (status != CAN_OK) {
    add_log("✗ SDO_RD TX fail 0x" + String(index, HEX));
    return status;
  }
  
  SimpleCANFrame rx_frame;
  uint32_t start = millis();
  
  while ((millis() - start) < SDO_RESPONSE_TIMEOUT) {
    status = CAN_Receive(&rx_frame, 1);
    
    if (status == CAN_OK) {
      if (rx_frame.id == SDO_RX) {
        uint8_t cmd = rx_frame.data[0];
        
        if (cmd == 0x80) {
          uint32_t abort_code = rx_frame.data[4] 
                              | (rx_frame.data[5] << 8)
                              | (rx_frame.data[6] << 16)
                              | (rx_frame.data[7] << 24);
          add_log("✗ SDO_RD_ABORT 0x" + String(index, HEX) + " code:0x" + String(abort_code, HEX));
          diagnoseSDOAbort(abort_code, index);
          return CAN_SDO_ABORT;
        }
        
        if ((cmd & 0xE0) == 0x40) {
          *value = rx_frame.data[4] 
                 | (rx_frame.data[5] << 8)
                 | (rx_frame.data[6] << 16)
                 | (rx_frame.data[7] << 24);
          // Log successful read (commented to avoid spam, uncomment for debugging)
          // add_log("✓ SDO_RD 0x" + String(index, HEX) + "=0x" + String(*value, HEX));
          return CAN_OK;
        }
      }
    }
  }
  
  add_log("✗ SDO_RD timeout 0x" + String(index, HEX) + " after " + String(SDO_RESPONSE_TIMEOUT) + "ms");
  return CAN_RX_TIMEOUT;
}

//=============================================================================
// DRIVE MOTOR CONTROL COMMANDS
//=============================================================================

// Shutdown motor (transition to "Ready to Switch On" state)
CANStatus shutdown() {
  return SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_SHUTDOWN);
}

// Enable motor operation
CANStatus enable() {
  return SDO_Write(CONTROLWORD, 0x00, CONTROLWORD_ENABLE);
}

// Set operation mode
CANStatus setOperationMode(int mode) {
  return SDO_Write(OPERATION_MODE, 0x00, mode);
}

// Set maximum profile velocity
CANStatus setMaxProfileVelocity(uint32_t velocity_rpm) {
  return SDO_Write(MAX_PROFILE_VELOCITY, 0x00, velocity_rpm);
}

// Set profile acceleration
CANStatus setProfileAcceleration(uint32_t accel_rpm_s) {
  return SDO_Write(PROFILE_ACCELERATION, 0x00, accel_rpm_s);
}

// Set profile deceleration
CANStatus setProfileDeceleration(uint32_t decel_rpm_s) {
  return SDO_Write(PROFILE_DECELERATION, 0x00, decel_rpm_s);
}

// Set quick stop deceleration
CANStatus setQuickStopDeceleration(uint32_t decel_rpm_s) {
  return SDO_Write(0x6085, 0x00, decel_rpm_s);
}

// Set motion profile type
CANStatus setMotionProfileType(uint32_t profile_type) {
  return SDO_Write(0x6086, 0x00, profile_type);
}

// Configure motor parameters for velocity mode
bool MotorConfig() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     MOTOR CONFIGURATION SEQUENCE       ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  add_log("Motor config start");
  
  // Additional delay for EPOS internal initialization
  Serial.println("→ Allowing EPOS internal boot time...");
  delay(500);
  
  // Step 1: Set operation mode
  Serial.println("→ Setting Profile Velocity Mode...");
  add_log("→ Set operation mode " + String(PROFILE_VELOCITY_MODE));
  int retries = 3;
  CANStatus status = CAN_ERROR;
  
  for (int i = 0; i < retries; i++) {
    status = setOperationMode(PROFILE_VELOCITY_MODE);
    if (status == CAN_OK) break;
    Serial.println("   Retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ FATAL: Operation mode failed after retries");
    Serial.println("✗ FATAL: Could not set operation mode");
    return false;
  }
  add_log("✓ Operation mode set");
  delay(50);  // Allow EPOS to process mode change
  
  // Step 2: Set max velocity
  add_log("→ Set max velocity " + String(MAX_VELOCITY_RPM));
  Serial.println("→ Setting max velocity to " + String(MAX_VELOCITY_RPM) + " RPM...");
  for (int i = 0; i < retries; i++) {
    status = setMaxProfileVelocity(MAX_VELOCITY_RPM);
    if (status == CAN_OK) break;
    Serial.println("   Retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ Max velocity failed");
    return false;
  }
  add_log("✓ Max velocity set");
  delay(50);
  
  // Step 3: Set acceleration
  add_log("→ Set acceleration " + String(PROFILE_ACCEL_RPM_S));
  Serial.println("→ Setting acceleration to " + String(PROFILE_ACCEL_RPM_S) + " RPM/s...");
  for (int i = 0; i < retries; i++) {
    status = setProfileAcceleration(PROFILE_ACCEL_RPM_S);
    if (status == CAN_OK) break;
    Serial.println("   Retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ Acceleration failed");
    return false;
  }
  add_log("✓ Acceleration set");
  delay(50);
  
  // Step 4: Set deceleration
  add_log("→ Set deceleration " + String(PROFILE_DECEL_RPM_S));
  Serial.println("→ Setting deceleration to " + String(PROFILE_DECEL_RPM_S) + " RPM/s...");
  for (int i = 0; i < retries; i++) {
    status = setProfileDeceleration(PROFILE_DECEL_RPM_S);
    if (status == CAN_OK) break;
    Serial.println("   Retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ Deceleration failed");
    return false;
  }
  add_log("✓ Deceleration set");
  delay(50);
  
  // Step 5: Shutdown sequence (transition to Ready to Switch On)
  Serial.println("→ Executing shutdown sequence...");
  add_log("→ Shutdown...");
  for (int i = 0; i < retries; i++) {
    status = shutdown();
    if (status == CAN_OK) break;
    Serial.println("   Shutdown retry " + String(i + 1) + "/" + String(retries));
    delay(100);
  }
  
  if (status != CAN_OK) {
    add_log("✗ Shutdown failed");
    return false;
  }
  add_log("✓ Shutdown complete");
  delay(100);  // Critical: Allow EPOS state transition time
  
  // Step 6: Enable sequence (transition to Operation Enabled)
  Serial.println("→ Executing enable sequence...");
  add_log("→ Enable...");
  for (int i = 0; i < retries; i++) {
    status = enable();
    if (status == CAN_OK) {
      delay(50);
      // Verify enable was successful by checking statusword
      uint32_t statusword = 0;
      if (SDO_Read(0x6041, 0x00, &statusword) == CAN_OK) {
        Serial.print("   Statusword: 0x");
        Serial.println(statusword, HEX);
        // Bit pattern for Operation Enabled: xxxx x0xx x010 0111 (0x0027 masked with 0x006F)
        if ((statusword & 0x006F) == 0x0027) {
          add_log("✓ Enable complete");
          add_log("✓ Motor enabled and verified");
          Serial.println("   ✓ Motor in Operation Enabled state");
          break;
        } else {
          Serial.println("   ⚠ Motor not in expected state, retrying...");
          status = CAN_ERROR;  // Force retry
        }
      }
    }
    
    if (i < retries - 1) {
      Serial.println("   Enable retry " + String(i + 1) + "/" + String(retries));
      delay(200);  // Longer delay between enable retries
    }
  }
  
  if (status != CAN_OK) {
    add_log("✗ Enable failed");
    return false;
  }
  
  add_log("✓ Motor configured");
  Serial.println("\n✓ Motor configured successfully\n");
  return true;
}

//=============================================================================
// MODBUS RTU PROTOCOL (SERVO COMMUNICATION)
//=============================================================================

// Calculate CRC16 for Modbus RTU
uint16_t calcCRC(uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Send Modbus RTU packet over Serial2
void sendPacket(uint8_t *buffer, uint8_t length) {
  uint16_t crc = calcCRC(buffer, length);
  buffer[length] = crc & 0xFF;
  buffer[length + 1] = crc >> 8;
  
  while (Serial2.available()) Serial2.read();
  Serial2.write(buffer, length + 2);
  Serial2.flush();
  delay(3);
}

// Read Modbus RTU response from Serial2
int readResponse(uint8_t *buffer, int expectedLen, int timeoutMs) {
  unsigned long startTime = millis();
  int bytesRead = 0;
  
  while ((millis() - startTime) < timeoutMs) {
    if (Serial2.available()) {
      buffer[bytesRead++] = Serial2.read();
      if (bytesRead >= expectedLen) break;
    } else {
      delay(1);
    }
  }
  
  if (bytesRead < expectedLen) {
    add_log("✗ Timeout: " + String(bytesRead) + "/" + String(expectedLen));
    // Serial.print(bytesRead);
    // Serial.print("/");
    // Serial.println(expectedLen);
    return -1;
  }
  
  uint16_t receivedCRC = (buffer[bytesRead - 1] << 8) | buffer[bytesRead - 2];
  uint16_t calculatedCRC = calcCRC(buffer, bytesRead - 2);
  
  if (receivedCRC != calculatedCRC) {
    add_log("✗ CRC Error");
    return -2;
  }
  
  return bytesRead;
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

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

String DriveMotorStateToString(DriveMotorState state) {
  switch(state) {
    case DRIVE_MOTOR_STOP: return "STOP";
    case DRIVE_MOTOR_STOPPING: return "STOPPED";
    case DRIVE_MOTOR_RUNNING: return "RUNNING";
    case DRIVE_MOTOR_SWEEPING_COLUMN: return "SWEEPING_COLUMN";
    case DRIVE_MOTOR_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

String DriveMotorDirectionToString(DriveMotorDirection dir) {
  switch(dir) {
    case DRIVE_MOTOR_FORWARD: return "FORWARD";
    case DRIVE_MOTOR_REVERSE: return "REVERSE";
    default: return "UNKNOWN";
  }
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

// Function to make a beep sound using the buzzer
void Beep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

//=============================================================================
// DEBUG LOGGING AND COMMUNICATION
//=============================================================================

// Add log message to debug buffer
void add_log(String log) {
  if (xDebugLogMutex != NULL) {
    if (xSemaphoreTake(xDebugLogMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      debugLoggingString += " | " + log;
      xSemaphoreGive(xDebugLogMutex);
    }
  }
  // Also print to serial for real-time debugging
  // Serial.println(log);
}


// HTTP Debug Logger RTOS Task (runs on Core 0)
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  HTTPClient httpDebugger;
  String localLogBuffer = "";
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(2000));  // Send logs every 2 seconds
    
    // Copy debug string to local buffer with mutex protection
    if (xDebugLogMutex != NULL) {
      if (xSemaphoreTake(xDebugLogMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        localLogBuffer = debugLoggingString;
        xSemaphoreGive(xDebugLogMutex);
      }
    }
    
    // Send logs if there's content and logging is enabled
    if (localLogBuffer != BOT_ID + " " + FIRMWARE_ID + ": ") {
      int httpCode = -1;
      
      if (loggerFlag) {
        String loggerUrl = DEBUG_SERVER_URL + "?entity=bot&entity_id=" + BOT_ID;
        httpDebugger.begin(loggerUrl);
        httpDebugger.addHeader("Content-Type", "text/plain");
        httpCode = httpDebugger.POST(localLogBuffer);
        httpDebugger.end();
      } else {
        httpCode = 200;  // Simulate success when logging disabled
      }
      
      // Clear buffer on success
      if (httpCode == 200) {
        if (xDebugLogMutex != NULL) {
          if (xSemaphoreTake(xDebugLogMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            debugLoggingString = BOT_ID + " " + FIRMWARE_ID + ": ";
            xSemaphoreGive(xDebugLogMutex);
          }
        }
      }
    }
  }
}

//=============================================================================
// GPIO CONFIGURATION
//=============================================================================

// Configure MCP23017 I/O expander
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("MCP Error.");
  }
}

// Configure GPIO pins for sensors and buzzer
void PinConfig() {
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT_PULLUP);       // Traffic indicator pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT_PULLUP);      // Column indicator pin config
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
  pinMode(EMG_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);

  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, LOW);
}

//=============================================================================
// NVM CONFIGURATION FUNCTIONS
//=============================================================================

// Load diverter servo configuration from NVM (Non-Volatile Memory)
void LoadDiverterConfig() {
  if (!prefs.begin("diverter", true)) {
    Serial.println("✗ NVM failed - using defaults");
    add_log("NVM access failed - defaults used");
    return;
  }
  
  // Load Bot ID
  BOT_ID = prefs.getString("BotId", BOT_ID);
  if (BOT_ID != "") {
    // Note: BOT_ID is a #define, can't be changed at runtime
    // This reads the stored value for verification
    Serial.println("NVM Bot ID: " + BOT_ID);
  }
  
  // Load front diverter configuration
  front_diverter_left_limit = prefs.getInt("front_left_lim", 1994);
  front_diverter_right_limit = prefs.getInt("front_right_lim", 2525);
  front_diverter_tolerance = prefs.getInt("front_tol", 106);
  front_diverter_left_threshold = prefs.getInt("front_left_thr", 2100);
  front_diverter_right_threshold = prefs.getInt("front_right_thr", 2419);
  
  // Load rear diverter configuration
  rear_diverter_left_limit = prefs.getInt("rear_left_lim", 2936);
  rear_diverter_right_limit = prefs.getInt("rear_right_lim", 2387);
  rear_diverter_tolerance = prefs.getInt("rear_tol", 109);
  rear_diverter_left_threshold = prefs.getInt("rear_left_thr", 2827);
  rear_diverter_right_threshold = prefs.getInt("rear_right_thr", 2496);
  
  prefs.end();
  
  Serial.println("✓ Config loaded from NVM");
  Serial.println("  Front: L=" + String(front_diverter_left_limit) + " R=" + String(front_diverter_right_limit) + " Tol=" + String(front_diverter_tolerance));
  Serial.println("  Rear:  L=" + String(rear_diverter_left_limit) + " R=" + String(rear_diverter_right_limit) + " Tol=" + String(rear_diverter_tolerance));
  add_log("NVM config loaded");
}

//=============================================================================
// WIFI CONFIGURATION
//=============================================================================

// Configure WiFi connection
void WiFiConfig() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(SSID, PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    add_log("WiFi connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n✗ WiFi connection failed");
    add_log("WiFi failed");
  }
}