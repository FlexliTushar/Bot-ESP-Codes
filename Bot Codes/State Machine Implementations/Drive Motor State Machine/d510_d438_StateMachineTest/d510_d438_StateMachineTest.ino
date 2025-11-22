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
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35

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

// Constant variables
const String CODE_ID = "d510";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";


Adafruit_MCP23X17 mcp;
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

enum DriveMotorState {
    DRIVE_MOTOR_STOP = 1,
    DRIVE_MOTOR_STOPPING = 2,
    DRIVE_MOTOR_RUNNING = 3,
    DRIVE_MOTOR_SWEEPING_COLUMN = 4,
    DRIVE_MOTOR_ERROR = 5
};

// Upcoming States (for future implementation):
// AVAILABLE, BUSY_STOP, BUSY_STOPPING, BUSY_MOVING, 
// BUSY_SWEEPING_COLUMN, MAINTENANCE

enum DriveMotorDirection {
    DRIVE_MOTOR_FORWARD = 1,
    DRIVE_MOTOR_REVERSE = -1
};

enum DriveMotorStateOutput {
    DRIVE_MOTOR_STATE_NO_CHANGE = 0,
    DRIVE_MOTOR_STATE_CHANGED = 1,
    DRIVE_MOTOR_UNKNOWN_STATE = 2
};


// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

bool traffic_permission = true;
bool global_error_permission = true;
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

Preferences prefs;

TaskHandle_t sensorReadingTask;
volatile bool firstTimeError = false;
TaskHandle_t actuationTask;

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
    }
    
    columnJustCompleted = false;
  }
}

int offCounter = 0;

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
  CANConfig();
  MotorConfig();
  LoadDiverterConfig();
  Beep();
  delay(1000);
  drive_motor_set_speed = S0_5;  // Initial speed set to 0.5 m/s
  delay(500);
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  printFileName();
  add_log("Starting the bot!");
  xTaskCreatePinnedToCore(
    SENSOR_READING_TASK,
    "sensor_reading_task",
    10000,
    NULL,
    5,
    &sensorReadingTask,
    1);
  vTaskDelay(pdMS_TO_TICKS(20));
  xTaskCreatePinnedToCore(
    ACTUATION_TASK,
    "actuation_task",
    10000,
    NULL,
    4,
    &actuationTask,
    1);
}

// Logger Function which will run as RTOS task
void SENSOR_READING_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    // Read TI/CI sensors and process column detection
    String combo = ReadSensorCombo();
    ProcessColumnDetection(combo);
    HandleColumnDecisions();

    if (UpdateDriveMotorState() == DRIVE_MOTOR_STATE_CHANGED) {
      add_log("State changed to: " + String(drive_motor_current_state) + " from " + String(drive_motor_previous_state));
      add_log("All shared variables - Traffic_Permission: " + String(traffic_permission) + ", Global_Error_Permission: " + String(global_error_permission) + ", Current_Assignment: " + String(current_assignment) + ", Permitted_Edge_Speed: " + String(permitted_edge_speed) + ", column_detected_in_sweep: " + 
                String(column_detected_in_sweep));
      ImplementDriveMotorState();
    }
  }
}

void ACTUATION_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Update motor speed if changed
    if (drive_motor_set_speed != drive_motor_previous_set_speed) {
      if (drive_motor_set_speed != 0) {
        setTagetVelocity(drive_motor_set_speed);
        add_log("Setting speed to " + String(drive_motor_set_speed));
      } else {
        haltMovement();
        add_log("halting the bot");
      }
      drive_motor_previous_set_speed = drive_motor_set_speed;
    }

    // Monitor drive motor current speed
    drive_motor_current_speed = getVelocityActualValueAveraged();
  }
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
  add_log("C" + String(totalColumnsDetected) + "[" + detectedColumnCode + "]:" + String(totalFramesBetweenSpaces) + "f(" + String(frameCount01) + "," + String(frameCount10) + "," + String(frameCount11) + ") " + HealthToString(overallHealthWTM));
  return overallHealthWTM;
}

void printFileName() {
  Serial.println(__FILE__);
}

// Function to load values from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);
  BOT_ID = prefs.getString("BotId", "Bx");
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
  // mcp.digitalWrite(PANEL_LED_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
  // mcp.digitalWrite(PANEL_LED_PIN, LOW);
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
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT_PULLUP);       // Traffic indicator pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT_PULLUP);      // Column indicator pin config
  pinMode(BUZZER, OUTPUT);
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