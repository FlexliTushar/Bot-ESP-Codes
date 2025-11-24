#define FIRMWARE_VERSION "d511"
/*
  Diverter State Machine Test Sketch
  ====================================
  Standalone test for diverter state machine control
  
  This sketch tests the diverter state machine by automatically toggling
  the demand direction between LEFT and RIGHT every 5 seconds.
*/

#include <SMS.h>
#include <Adafruit_MCP23X17.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
using namespace std;

// GPIO Pin Assignments
#define BUZZER 13
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define PANEL_LED_BUTTON 7

#define ENABLE 1
#define DISABLE 0

// Constant variables
const String CODE_ID = "d511_diverter_test";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";

Adafruit_MCP23X17 mcp;
SMS sm;
Preferences prefs;

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

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

TaskHandle_t diverterStateTask;
TaskHandle_t diverterActuationTask;
TaskHandle_t auxiliaryTask;

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
DiverterDirection diverter_track_edge_direction = DIVERTER_DIRECTION_UNKNOWN;
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
const unsigned long TOGGLE_INTERVAL = 5000;  // 5 

portMUX_TYPE current_diverter_position_high_level_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE error_diverter_status_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE diverter_set_position_value_mux = portMUX_INITIALIZER_UNLOCKED;

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// Update high-level position based on current encoder position
void UpdateDiverterPositionHighLevel() {
  if (front_diverter_current_position > front_diverter_right_thresold && 
  front_diverter_current_position < front_diverter_right_limit &&
  rear_diverter_current_position > rear_diverter_right_thresold &&
  rear_diverter_current_position < rear_diverter_right_limit) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_RIGHT;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  if (front_diverter_current_position > front_diverter_left_limit && 
  front_diverter_current_position < front_diverter_left_thresold &&
  rear_diverter_current_position > rear_diverter_left_limit &&
  rear_diverter_current_position < rear_diverter_left_thresold) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_LEFT;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  if (front_diverter_current_position > front_diverter_right_limit && 
  front_diverter_current_position < front_diverter_left_limit &&
  rear_diverter_current_position > rear_diverter_right_limit &&
  rear_diverter_current_position < rear_diverter_left_limit) {
    portENTER_CRITICAL(&current_diverter_position_high_level_mux);
    current_diverter_position_high_level = DIVERTER_POSITION_INTERIM;
    portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
  }

  portENTER_CRITICAL(&current_diverter_position_high_level_mux);
  current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;
  portEXIT_CRITICAL(&current_diverter_position_high_level_mux);
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
  
  // Update position high level
  UpdateDiverterPositionHighLevel();
  
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
    add_log("Diverter in LEFT position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_RIGHT) {
    // ===== RIGHT STATE =====
    diverter_set_torque_flag = false;
    add_log("Diverter in RIGHT position - torque disabled.");
  }
  
  else if (diverter_current_state == DIVERTER_SWITCHING) {
    // ===== SWITCHING STATE =====
    diverter_set_torque_flag = true;  // Enable torque for movement
    if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_left_thresold;
      rear_diverter_set_position_value = rear_diverter_left_thresold;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      add_log("Switching to LEFT - moving to front threshold: " + String(front_diverter_left_thresold) + " and rear threshold: " + String(rear_diverter_left_thresold));
    }
    else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
      portENTER_CRITICAL(&diverter_set_position_value_mux);
      front_diverter_set_position_value = front_diverter_right_thresold;
      rear_diverter_set_position_value = rear_diverter_right_thresold;
      portEXIT_CRITICAL(&diverter_set_position_value_mux);
      add_log("Switching to RIGHT - moving to front threshold: " + String(front_diverter_right_thresold) + " and rear threshold: " + String(rear_diverter_right_thresold));
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

// ==============================================================================
// RTOS TASKS
// ==============================================================================

// Diverter State Machine Task
void DIVERTER_STATE_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);  // 10ms cycle time
    global_error_status = error_diverter_status;
    
    DiverterStateChangeOptions stateChange = UpdateDiverterState();
    if (stateChange == DIVERTER_STATE_CHANGED) {
      add_log("State changed to: " + String(diverter_current_state) + " from " + String(diverter_previous_state));
      add_log("Variables - Global_Error: " + String(global_error_status) + ", Demand_Dir: " + String(diverter_track_edge_direction) + ", Pos_HL: " + String(current_diverter_position_high_level) + ", Error_Status: " + String(error_diverter_status));
      UpdateDiverterActuatorInputVariables();
    } else if (stateChange == DIVERTER_UNEXPECTED_CONDITION) {
      add_log("Diverter State Machine in UNEXPECTED CONDITION!");
    }

    if (!mcp.digitalRead(PANEL_LED_BUTTON) && global_error_status) {
      error_diverter_status = false;
    }
  }
}

// Diverter Actuation Task
void DIVERTER_ACTUATION_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Update motor speed if changed
    if (!diverter_set_torque_flag && prev_diverter_set_torque_flag) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      if (sm.getLastError()) {
        add_log("Front torque disabling communication failed!");
        portENTER_CRITICAL(&error_diverter_status_mux);
        error_diverter_status = true;
        portEXIT_CRITICAL(&error_diverter_status_mux);
      }
      sm.EnableTorque(REAR_SERVO, DISABLE);
      if (sm.getLastError()) {
        add_log("Rear torque disabling communication failed!");
        portENTER_CRITICAL(&error_diverter_status_mux);
        error_diverter_status = true;
        portEXIT_CRITICAL(&error_diverter_status_mux);
      }
      prev_diverter_set_torque_flag = diverter_set_torque_flag;
    }

    if (front_diverter_set_position_value != -1 && front_diverter_set_position_value != prev_front_diverter_set_position_value) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_set_position_value, 100);
      if (sm.getLastError()) {
        add_log("Front position write communication failed!");
        portENTER_CRITICAL(&error_diverter_status_mux);
        error_diverter_status = true;
        portEXIT_CRITICAL(&error_diverter_status_mux);
      }
      prev_front_diverter_set_position_value = front_diverter_set_position_value;
    }

    if (rear_diverter_set_position_value != -1 && rear_diverter_set_position_value != prev_rear_diverter_set_position_value) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_set_position_value, 100);
      if (sm.getLastError()) {
        add_log("Rear position write communication failed!");
        portENTER_CRITICAL(&error_diverter_status_mux);
        error_diverter_status = true;
        portEXIT_CRITICAL(&error_diverter_status_mux);
      }
      prev_rear_diverter_set_position_value = rear_diverter_set_position_value;
    }

    front_diverter_current_position = sm.ReadPos(FRONT_SERVO);
    if (sm.getLastError()) {
      add_log("Front position read communication failed!");
      portENTER_CRITICAL(&error_diverter_status_mux);
      error_diverter_status = true;
      portEXIT_CRITICAL(&error_diverter_status_mux);
    }
    rear_diverter_current_position = sm.ReadPos(REAR_SERVO);
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
    vTaskDelay(100 / portTICK_PERIOD_MS);  // 100ms cycle time
    
    unsigned long current_time = millis();
    
    // Check if 5 seconds have passed and no errors
    if ((current_time - last_toggle_time >= TOGGLE_INTERVAL) && 
      (error_diverter_status == false) && 
      (global_error_status == false)) {
      
      // Toggle demand direction
      if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
        diverter_track_edge_direction = DIVERTER_DIRECTION_RIGHT;
        add_log("AUX: Toggling demand direction to RIGHT");
      } 
      else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
        diverter_track_edge_direction = DIVERTER_DIRECTION_LEFT;
        add_log("AUX: Toggling demand direction to LEFT");
      }
      else if (diverter_track_edge_direction == DIVERTER_DIRECTION_UNKNOWN) {
        // Initialize to LEFT on first toggle
        diverter_track_edge_direction = DIVERTER_DIRECTION_LEFT;
        add_log("AUX: Initializing demand direction to LEFT");
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
    
    xTaskCreatePinnedToCore(
        AUXILIARY_TASK,
        "auxiliary_task",
        10000,
        NULL,
        3,
        &auxiliaryTask,
        1);
    
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
