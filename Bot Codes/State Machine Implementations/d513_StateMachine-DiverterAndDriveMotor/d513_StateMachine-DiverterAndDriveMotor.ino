#define FIRMWARE_VERSION "d513"
/*
  Diverter State Machine Test Sketch
  ====================================
  Standalone test for diverter state machine control
  
  This sketch tests the diverter state machine by automatically toggling
  the demand direction between LEFT and RIGHT every 5 seconds.
*/

#include <SMS.h>
#include <esp32_can.h>
#include <Adafruit_MCP23X17.h>
#include <can_common.h>
#include <object_dictionary.h>
#include <maxon.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

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
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define PANEL_LED_BUTTON 7

#define ENABLE 1
#define DISABLE 0

// Health monitoring
enum HealthStatus {
  SAFE,
  WARNING,
  CRITICAL
};

// Column detection state machine variables
enum WTMColumnState {
  WTM_BETWEEN_COLUMNS,  // In 00 zone between columns
  WTM_IN_COLUMN         // On a column (01, 10, or 11)
};

enum DriveMotorState {
    DRIVE_MOTOR_STOP = 1,
    DRIVE_MOTOR_STOPPING = 2,
    DRIVE_MOTOR_RUNNING = 3,
    DRIVE_MOTOR_SWEEPING_COLUMN = 4,
    DRIVE_MOTOR_ERROR = 5
};

enum DriveMotorDirection {
    DRIVE_MOTOR_FORWARD = 1,
    DRIVE_MOTOR_REVERSE = -1
};

enum DriveMotorStateOutput {
    DRIVE_MOTOR_STATE_NO_CHANGE = 0,
    DRIVE_MOTOR_STATE_CHANGED = 1,
    DRIVE_MOTOR_UNKNOWN_STATE = 2
};

// Constant variables
const String CODE_ID = "d513";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/log";

Adafruit_MCP23X17 mcp;
SMS sm;
Preferences prefs;

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

WTMColumnState wtm_column_state = WTM_BETWEEN_COLUMNS;

// Frame counts for each possible column state
int frameCount00 = 0;
int frameCount01 = 0;
int frameCount10 = 0;
int frameCount11 = 0;

// Confirmation variables
int exitConfirmationCount = 0;  // Count of consecutive 00 samples
const int EXIT_CONFIRMATION_THRESHOLD = 10;  // Need 10 consecutive 00s to confirm exit

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
const int DEFAULT_EMPTY_SPACE_THRESHOLD = 30;    // Expected minimum frames for empty space (4cm at 2m/s ~20ms = 20-40 frames)
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
String previousStation = "X";
int uniqueColumnsDetectedInSegment = 0;  // Count of unique columns (01, 10, 11) between stations
int emptySpacesDetectedInSegment = 0;    // Count of 00 zones between stations

// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

bool traffic_permission = true;
bool current_assignment = true;
int permitted_edge_speed = S0_5;
bool column_detected_in_sweep = false;

int drive_motor_current_speed = 0;
int drive_motor_previous_set_speed = 0;
DriveMotorState drive_motor_current_state = DRIVE_MOTOR_STOP;
DriveMotorState drive_motor_previous_state = DRIVE_MOTOR_STOP;
DriveMotorDirection drive_motor_current_direction = DRIVE_MOTOR_FORWARD;
DriveMotorDirection drive_motor_previous_direction = DRIVE_MOTOR_FORWARD;
int drive_motor_sweep_speed = S0_05;
int drive_motor_set_speed = 0;
int drive_motor_direction_value = 1;
bool drive_motor_error_status = false;
int totalColumnsDetected = 0;

volatile bool firstTimeError = false;

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

TaskHandle_t readingSensorAndUpdateStateTask;
TaskHandle_t actuationTask;

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
// GLOBAL VARIABLES - DIVERTER STATE MACHINE
// ==============================================================================

// Position thresholds (encoder counts)
int left_threshold = 1000;
int right_threshold = 2000;
int left_position_tolerance = 100;   // +/- tolerance for left position
int right_position_tolerance = 100;  // +/- tolerance for right position

// Shared variables
bool global_error_status = false;
DiverterDirection diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;
volatile DiverterPosition current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;

// State machine variables
DiverterState diverter_current_state = DIVERTER_STOP;
DiverterState diverter_previous_state = DIVERTER_STOP;
volatile bool error_diverter_status = false;
bool diverter_set_torque_flag = true;
bool prev_diverter_set_torque_flag = true;
int front_diverter_set_position_value = -1;
int rear_diverter_set_position_value = -1;
int prev_front_diverter_set_position_value = -1;
int prev_rear_diverter_set_position_value = -1;


// Actual position from encoder
int front_diverter_current_position = -1;
volatile int front_diverter_previous_set_position = -1;
int rear_diverter_current_position = -1;
volatile int rear_diverter_previous_set_position = -1;

// Auxiliary task variables
unsigned long last_toggle_time = 0;
const unsigned long TOGGLE_INTERVAL = 10000;  // 10 s

portMUX_TYPE current_diverter_position_high_level_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE error_diverter_status_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE diverter_set_position_value_mux = portMUX_INITIALIZER_UNLOCKED;

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
            // add_log("Fluke: " + String(totalFramesBetweenSpaces) + "f (01:" + String(frameCount01) + ",10:" + String(frameCount10) + ",11:" + String(frameCount11) + ") Rec:" + String(emptySpaceFrameCount + exitConfirmationCount));
            
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
            
            // add_log("TES:" + String(totalEmptySpacesDetected) + " ES" + String(emptySpacesDetectedInSegment) + ":" + String(emptySpaceFrameCount) + "f " + HealthToString(emptySpaceHealth));
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
      // add_log("Traffic Detected");
      traffic_permission = false;
    } else if (lastCompletedColumnCode == "01") {
      // add_log("Traffic Not Detected");
      traffic_permission = true;
    } else if (lastCompletedColumnCode == "10") {
      // add_log("WARNING: Column Indicator malfunction detected! Reading '10' (CI=0, TI=1)");
    }
    
    if (lastCompletedColumnHealth == CRITICAL) {
      // Column health is critical - may need maintenance
    }
    
    columnJustCompleted = false;
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
        if (current_assignment == true && traffic_permission == true && global_error_status == false) {
            drive_motor_current_state = DRIVE_MOTOR_RUNNING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to SWEEPING_COLUMN
        if (current_assignment == true && drive_motor_current_speed == 0 && traffic_permission == false && global_error_status == false) {
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
        if (current_assignment == true && traffic_permission == true && global_error_status == false) {
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
        if (global_error_status == true || (current_assignment == true && traffic_permission == false)) {
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
        if (global_error_status == true || (current_assignment == true && traffic_permission == false && column_detected_in_sweep == true)) {
            drive_motor_current_state = DRIVE_MOTOR_STOPPING;
            return DRIVE_MOTOR_STATE_CHANGED;
        }
        
        // Transition to RUNNING (if traffic permission granted)
        if (current_assignment == true && traffic_permission == true && global_error_status == false) {
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
        // add_log("Stopping the bot.");
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_STOPPING) {
        // ===== STOPPING STATE =====
        drive_motor_set_speed = 0;
        // add_log("Stopping the bot.");
    }
    
    
    else if (drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
        // ===== RUNNING STATE =====
        drive_motor_set_speed = permitted_edge_speed;
        drive_motor_current_direction = DRIVE_MOTOR_FORWARD; // Reset current direction
        // add_log("Running the bot at speed: " + drive_motor_set_speed);
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
        // add_log("Drive motor error detected! Halting the bot.");
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
    // add_log("Diverter in " + DiverterStateToString(DIVERTER_LEFT) + " position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_RIGHT) {
    // ===== RIGHT STATE =====
    diverter_set_torque_flag = false;
    // add_log("Diverter in " + DiverterStateToString(DIVERTER_RIGHT) + " position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    // ===== SWITCHING STATE =====
    diverter_set_torque_flag = true;  // Enable torque for movement
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_left_thresold;
      rear_diverter_set_position_value = rear_diverter_left_thresold;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      // add_log("Switching to " + DiverterDirectionToString(DIVERTER_DIRECTION_LEFT) + " - moving to front threshold: " + String(front_diverter_left_thresold) + " and rear threshold: " + String(rear_diverter_left_thresold));
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_right_thresold;
      rear_diverter_set_position_value = rear_diverter_right_thresold;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      // add_log("Switching to " + DiverterDirectionToString(DIVERTER_DIRECTION_RIGHT) + " - moving to front threshold: " + String(front_diverter_right_thresold) + " and rear threshold: " + String(rear_diverter_right_thresold));
    }
  }
  
  else if (diverter_current_state == DIVERTER_ERROR) {
    // ===== ERROR_DIVERTER STATE =====
    diverter_set_torque_flag = false;
    // add_log("Diverter error detected! Torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_STOP) {
    // ===== STOP_DIVERTER STATE =====
    diverter_set_torque_flag = false;
    // add_log("Diverter stopped - torque disabled.");
  }
}

// ==============================================================================
// RTOS TASKS
// ==============================================================================

// Diverter State Machine Task
void READING_SENSOR_AND_UPDATE_STATE_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);  // 10ms cycle time
    global_error_status = error_diverter_status || drive_motor_error_status;

    // Read TI/CI sensors and process column detection
    String combo = ReadSensorCombo();
    ProcessColumnDetection(combo);
    HandleColumnDecisions();

    if (UpdateDriveMotorState() == DRIVE_MOTOR_STATE_CHANGED) {
      add_log("State changed to: " + String(drive_motor_current_state) + " from " + String(drive_motor_previous_state));
      // add_log("All shared variables - Traffic_Permission: " + String(traffic_permission) + ", Global_Error_Status: " + String(global_error_status) + ", Current_Assignment: " + String(current_assignment) + ", Permitted_Edge_Speed: " + String(permitted_edge_speed) + ", column_detected_in_sweep: ," + 
                // String(column_detected_in_sweep));
      ImplementDriveMotorState();
    }
    
    DiverterStateChangeOptions stateChange = UpdateDiverterState();
    if (stateChange != DIVERTER_STATE_NO_CHANGE) Serial.println(stateChange);
    if (stateChange == DIVERTER_STATE_CHANGED) {
      add_log("State changed to: " + DiverterStateToString(diverter_current_state) + " from " + DiverterStateToString(diverter_previous_state));
      // add_log("Variables - Global_Error: " + String(global_error_status) + ", Demand_Dir: " + DiverterDirectionToString(diverter_track_edge_direction) + ", Pos_HL: " + DiverterPositionToString(current_diverter_position_high_level) + ", Error_Status: " + String(error_diverter_status));
      // add_log("Front current position: " + String(front_diverter_current_position) + ", Rear current position: " + String(rear_diverter_current_position));
      UpdateDiverterActuatorInputVariables();
    } else if (stateChange == DIVERTER_UNEXPECTED_CONDITION) {
      add_log("Diverter State Machine in UNEXPECTED CONDITION!");
    }
  }
}

// Actuation Task
void ACTUATION_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Update motor speed if changed
    if (!diverter_set_torque_flag ) {
      if (prev_diverter_set_torque_flag)
      {
        delay(1);
        sm.EnableTorque(FRONT_SERVO, DISABLE);
        if (sm.getLastError()) {
          // add_log("Front torque disabling communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
        delay(1);
        sm.EnableTorque(REAR_SERVO, DISABLE);
        if (sm.getLastError()) {
          // add_log("Rear torque disabling communication failed!");
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
          // add_log("Front position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }

      if (rear_diverter_set_position_value != -1 && rear_diverter_current_position > (rear_diverter_right_thresold + rear_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          // add_log("Rear position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }
    } else if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      if (front_diverter_set_position_value != -1 && front_diverter_current_position > (front_diverter_left_thresold + front_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(FRONT_SERVO, front_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          // add_log("Front position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }

      if (rear_diverter_set_position_value != -1 && rear_diverter_current_position < (rear_diverter_left_thresold - rear_diverter_tolerance)) {
        delay(1);
        sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
        if (sm.getLastError()) {
          // add_log("Rear position write communication failed!");
          portENTER_CRITICAL(&error_diverter_status_mux);
          error_diverter_status = true;
          portEXIT_CRITICAL(&error_diverter_status_mux);
        }
      }
    }
    prev_diverter_set_torque_flag = diverter_set_torque_flag;
    delay(1);
    front_diverter_current_position = sm.ReadPos(FRONT_SERVO);
    if (sm.getLastError()) {
      // add_log("Front position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }
    delay(1);
    rear_diverter_current_position = sm.ReadPos(REAR_SERVO);
    if (sm.getLastError()) {
      // add_log("Rear position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }

    UpdateDiverterPositionHighLevel();

    // Update motor speed if changed
    if (drive_motor_set_speed != drive_motor_previous_set_speed) {
      if (drive_motor_set_speed != 0) {
        String log = setTagetVelocity(drive_motor_set_speed);
        add_log("Set taget log: " + log);
      } else {
        String log = haltMovement();
        add_log("Halt log: " + log);
      }
      add_log("Set speed now: " + String(drive_motor_set_speed));
      drive_motor_previous_set_speed = drive_motor_set_speed;
    }

    // Monitor drive motor current speed
    drive_motor_current_speed = getVelocityActualValueAveraged();
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
    CANConfig();
    MotorConfig();
    LoadDiverterConfig();
    Beep();
    delay(1000);
    
    Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
    printFileName();
    add_log("Starting test!");
    
    // Create RTOS tasks
    xTaskCreatePinnedToCore(
        READING_SENSOR_AND_UPDATE_STATE_TASK,
        "reading_sensor_and_update_state_task",
        10000,
        NULL,
        4,
        &readingSensorAndUpdateStateTask,
        1);
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    xTaskCreatePinnedToCore(
        ACTUATION_TASK,
        "actuation_task",
        10000,
        NULL,
        5,
        &actuationTask,
        1);
    
    last_toggle_time = millis();
}

void loop() {
  // Empty loop as tasks handle functionality
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
  // add_log("C" + String(totalColumnsDetected) + "[" + detectedColumnCode + "]:" + String(totalFramesBetweenSpaces) + "f(" + String(frameCount01) + "," + String(frameCount10) + "," + String(frameCount11) + ") " + HealthToString(overallHealthWTM));
  return overallHealthWTM;
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
        String loggerUrl = HTTP_DEBUG_SERVER_URL + "?entity=bot&entity_id=" + BOT_ID;
        httpDebugger.begin(loggerUrl);
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
  // Serial.println(log);  // Also print to serial for debugging
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
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT_PULLUP);       // Traffic indicator pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT_PULLUP);      // Column indicator pin config
  pinMode(BUZZER, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
}

void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
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