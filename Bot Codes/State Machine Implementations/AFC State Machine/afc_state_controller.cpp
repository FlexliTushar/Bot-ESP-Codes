/*
AFC State Controller
============================
Automatic Flap Controller (AFC) State Management System

Global Variables:
- Global_Error_Status: bool - Shared
- Current_Station: Station (enum) - Shared
- Current_Edge: Edge (enum) - Shared
- Previous_Edge: Edge (enum) - Shared
- Drive_Motor_Current_State: State (enum) - Shared
- Drive_Motor_Sweeping_Duration: unsigned long - Shared

- AFC_Current_State: State (enum)
- Error_AFC_Status: bool
- AFC_Close_Flag: bool
- Infeed_Stop_Flag: bool
- AFC_Open_Timestamp: unsigned long
- AFC_Close_Timestamp: unsigned long
- Carrier_Open_Timestamp: unsigned long
- Limit_Switch_Pressed_Duration: unsigned long
- Dropping_Station_ID: int

- disableTorque(): AFC motor actuation function
- setAFCSetState(AFCSetState state): AFC motor actuation function
- setElectromagnets(bool state): Electromagnet control function (ON = true, OFF = false)
- callDropoffAPI(): API call function
- callDestinationAPI(): API call function
*/

// ==============================================================================
// STATE DEFINITIONS
// ==============================================================================

enum AFCState : uint8_t {
    AFC_CLOSING = 1,
    AFC_CLOSED_EMPTY = 2,
    AFC_CLOSED_PACKET = 3,
    AFC_STAGING = 4,
    AFC_OPENING = 5,
    AFC_OPEN = 6,
    AFC_STOP = 7,
    AFC_ERROR = 8
};

enum AFCSetState : uint8_t {
    AFC_SET_CLOSED = 1,
    AFC_SET_STAGED = 2,
    AFC_SET_OPEN = 3,
    AFC_SET_UNKNOWN = 0
};

enum AFCUpdateStateOutput : uint8_t {
    AFC_STATE_NO_CHANGE = 0,
    AFC_STATE_CHANGED = 1,
    AFC_UNKNOWN_STATE = 2
};

enum DriveMotorState : uint8_t {
    DRIVE_MOTOR_STOP = 1,
    DRIVE_MOTOR_STOPPING = 2,
    DRIVE_MOTOR_RUNNING = 3,
    DRIVE_MOTOR_SWEEPING_COLUMN = 4,
    DRIVE_MOTOR_ERROR = 5
};

enum EdgeType : uint8_t {
    EDGE_INFEED = 1,
    EDGE_GENERAL = 2,
    EDGE_UNKNOWN = 0
};

// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

// Shared variables
bool global_error_status = false;
int current_station = 0;
int dropping_station_id = 1;
EdgeType current_edge = EDGE_UNKNOWN;
EdgeType previous_edge = EDGE_UNKNOWN;
DriveMotorState drive_motor_current_state = DRIVE_MOTOR_STOP;
unsigned long drive_motor_sweeping_duration = 0;

// AFC-specific variables
AFCState afc_current_state = AFC_STOP;
AFCState afc_previous_state = AFC_STOP;
bool error_afc_status = false;
bool afc_close_flag = false;
bool infeed_stop_flag = false;
unsigned long afc_open_timestamp = 0;
unsigned long afc_close_timestamp = 0;
unsigned long carrier_open_timestamp = 0;
unsigned long limit_switch_pressed_duration = 0;
bool clear_command_received = false;

// Implementer demand variables
AFCSetState afc_set_state = AFC_SET_UNKNOWN;
bool electromagnets_state = true;  // true = ON, false = OFF
bool afc_set_torque_flag = false;

// Timing constants (in milliseconds)
const unsigned long AFC_OPEN_TIMEOUT = 1000;
const unsigned long AFC_CLOSE_DETECTION_TIME = 50;
const unsigned long SWEEPING_STATE_DURATION_THRESHOLD = 100;
const unsigned long CARRIER_OPEN_DURATION = 300;
const unsigned long AFC_CLOSING_TIMEOUT = 1000;

bool dropoff_update_api_flag = false;  // Flag to indicate dropoff API call needed

bool Log_info(String message);  // Logging function - Returns true if log successful


// ==============================================================================
// UTILITY FUNCTIONS
// ==============================================================================
void updateAFCCloseFlag() {
    // Update AFC Close Flag based on limit switch pressed duration
    if (limit_switch_pressed_duration > AFC_CLOSE_DETECTION_TIME) {
        afc_close_flag = true;
    } else {
        afc_close_flag = false;
    }
}

void updateInfeedStopFlag() {
    // Update Infeed Stop Flag based on conditions
    // If current edge is infeed edge, sweeping state duration > 100ms, and current state is RUNNING
    if (current_edge == EDGE_INFEED && 
        drive_motor_sweeping_duration > SWEEPING_STATE_DURATION_THRESHOLD && 
        drive_motor_current_state == DRIVE_MOTOR_RUNNING) {
        infeed_stop_flag = true;
    }
    
    // Reset flag when edge changes from infeed to other
    if (previous_edge == EDGE_INFEED && current_edge != EDGE_INFEED) {
        infeed_stop_flag = false;
    }
}


// ==============================================================================
// STATE UPDATE FUNCTION
// ==============================================================================

AFCUpdateStateOutput UpdateAFCState() {
    /*
    Evaluates global variables and determines if state should change
    Updates Current_State based on conditions and valid transitions
    Returns: state change status
    */
    afc_previous_state = afc_current_state;
    unsigned long currentTime = millis();
    
    
    // ----- FROM ERROR_AFC STATE -----
    if (afc_current_state == AFC_ERROR) {
        // Transition to AFC_STOP
        if (error_afc_status == false) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in ERROR_AFC
        afc_current_state = AFC_ERROR;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_STOP STATE -----
    else if (afc_current_state == AFC_STOP) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_STOP if global error permission is false
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_NO_CHANGE;
        }
        
        // Transition to AFC_CLOSING
        if (currentTime - afc_open_timestamp > AFC_OPEN_TIMEOUT && 
            afc_close_flag == false) {
            afc_current_state = AFC_CLOSING;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_CLOSED_EMPTY
        if (afc_close_flag == true && infeed_stop_flag == false) {
            afc_current_state = AFC_CLOSED_EMPTY;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_CLOSED_PACKET
        if (afc_close_flag == true && 
            current_edge == EDGE_INFEED && 
            infeed_stop_flag == true) {
            afc_current_state = AFC_CLOSED_PACKET;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_OPENING (Note: Skipping CARRIER_OPENING as requested)
        if (current_station == dropping_station_id) {
            afc_current_state = AFC_OPENING;
            carrier_open_timestamp = currentTime;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_STOP
        afc_current_state = AFC_STOP;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_CLOSING STATE -----
    else if (afc_current_state == AFC_CLOSING) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_STOP
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_CLOSED_EMPTY
        if (afc_close_flag == true && infeed_stop_flag == false) {
            afc_current_state = AFC_CLOSED_EMPTY;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_CLOSING
        afc_current_state = AFC_CLOSING;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_CLOSED_EMPTY STATE -----
    else if (afc_current_state == AFC_CLOSED_EMPTY) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_STOP
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_CLOSED_EMPTY if still closed
        if (afc_close_flag == true && infeed_stop_flag == false) {
            afc_current_state = AFC_CLOSED_EMPTY;
            return AFC_STATE_NO_CHANGE;
        }
        
        // Transition to AFC_CLOSED_PACKET
        if (afc_close_flag == true && infeed_stop_flag == true) {
            afc_current_state = AFC_CLOSED_PACKET;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_CLOSED_EMPTY
        afc_current_state = AFC_CLOSED_EMPTY;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_CLOSED_PACKET STATE -----
    else if (afc_current_state == AFC_CLOSED_PACKET) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_STOP
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_OPENING
        if (current_station == dropping_station_id) {
            afc_current_state = AFC_OPENING;
            carrier_open_timestamp = currentTime;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_CLOSED_PACKET
        afc_current_state = AFC_CLOSED_PACKET;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_STAGING STATE -----
    else if (afc_current_state == AFC_STAGING) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_STOP
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_STAGING
        afc_current_state = AFC_STAGING;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_OPENING STATE -----
    else if (afc_current_state == AFC_OPENING) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_STOP
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_OPEN
        if (currentTime - carrier_open_timestamp > CARRIER_OPEN_DURATION) {
            afc_current_state = AFC_OPEN;
            afc_open_timestamp = currentTime;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_OPENING
        afc_current_state = AFC_OPENING;
        return AFC_STATE_NO_CHANGE;
    }
    
    
    // ----- FROM AFC_OPEN STATE -----
    else if (afc_current_state == AFC_OPEN) {
        // Transition to ERROR_AFC
        if (error_afc_status == true) {
            afc_current_state = AFC_ERROR;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_STOP
        if (global_error_status == true) {
            afc_current_state = AFC_STOP;
            return AFC_STATE_CHANGED;
        }
        
        // Transition to AFC_CLOSING
        if (currentTime - carrier_open_timestamp > AFC_CLOSING_TIMEOUT) {
            afc_current_state = AFC_CLOSING;
            return AFC_STATE_CHANGED;
        }
        
        // Remain in AFC_OPEN
        afc_current_state = AFC_OPEN;
        return AFC_STATE_NO_CHANGE;
    }
    
    return AFC_UNKNOWN_STATE;
}


// ==============================================================================
// STATE IMPLEMENTER / ACTUATOR
// ==============================================================================

void UpdateImplementerDemandStateAFC() {
    /*
    Executes the actions required for the current state
    */
    
    if (afc_current_state == AFC_ERROR) {
        // ===== ERROR_AFC STATE =====
        afc_set_torque_flag = true;  // Disable torque
        electromagnets_state = true;  // Turn ON electromagnets
        Log_info("AFC error detected! Torque disabled, electromagnets ON.");
    }
    
    
    else if (afc_current_state == AFC_STOP) {
        // ===== AFC_STOP STATE =====
        afc_set_torque_flag = true;  // Disable torque
        electromagnets_state = true;  // Turn ON electromagnets
        Log_info("AFC stopped - torque disabled, electromagnets ON.");
    }
    
    
    else if (afc_current_state == AFC_CLOSING) {
        // ===== AFC_CLOSING STATE =====
        afc_set_state = AFC_SET_CLOSED;
        electromagnets_state = true;  // Turn ON electromagnets
        Log_info("AFC closing - set state set to CLOSED, electromagnets ON.");
    }
    
    
    else if (afc_current_state == AFC_CLOSED_EMPTY) {
        // ===== AFC_CLOSED_EMPTY STATE =====
        dropoff_update_api_flag = true;  // Indicate dropoff API call needed
        Log_info("AFC closed empty - calling dropoff API.");
    }
    
    
    else if (afc_current_state == AFC_CLOSED_PACKET) {
        // ===== AFC_CLOSED_PACKET STATE =====
        // Call Destination API
        Log_info("AFC closed with packet - calling destination API.");
        // callDestinationAPI();  // Uncomment when API function is available
    }
    
    
    else if (afc_current_state == AFC_STAGING) {
        // ===== AFC_STAGING STATE =====
        afc_set_state = AFC_SET_STAGED;
        Log_info("AFC staging - set state set to STAGED.");
    }
    
    
    else if (afc_current_state == AFC_OPENING) {
        // ===== AFC_OPENING STATE =====
        electromagnets_state = false;  // Turn OFF electromagnets
        Log_info("AFC opening - electromagnets turned OFF.");
    }
    
    
    else if (afc_current_state == AFC_OPEN) {
        // ===== AFC_OPEN STATE =====
        Log_info("AFC open.");
    }
}


// ==============================================================================
// MAIN CONTROL LOOP
// ==============================================================================

void main() {
    // Main control loop
    while (true) {
        // Update utility flags
        updateAFCCloseFlag();
        updateInfeedStopFlag();
        
        AFCUpdateStateOutput stateChange = UpdateAFCState();
        if (stateChange == AFC_STATE_CHANGED) {
            Log_info("AFC State changed to: " + String(afc_current_state) + 
                     " from " + String(afc_previous_state));
            Log_info("All shared variables - Global_Error_Status: " + 
                     String(global_error_status) + 
                     ", Current_Station: " + String(current_station) + 
                     ", AFC_Close_Flag: " + String(afc_close_flag) + 
                     ", Infeed_Stop_Flag: " + String(infeed_stop_flag) + 
                     ", Error_AFC_Status: " + String(error_afc_status));
            UpdateImplementerDemandStateAFC();
        } else if (stateChange == AFC_UNKNOWN_STATE) {
            Log_info("AFC State Machine in UNKNOWN STATE!");
        }
        // Add delay or other processing as needed
    }
}
