/*
Diverter State Controller
============================
Diverter Control System with State Management

Global Variables:
- Global_Error_Permission: bool - Shared
- diverter_track_edge_direction: Direction (enum) - Shared
- current_diverter_position_high_level: int - Shared

- Current_State: State (enum)
- Error_Diverter_Status: bool
- Left_Threshold: int
- Right_Threshold: int

- disableTorque(): diverter actuation function
- moveToPosition(int position): diverter actuation function
*/

// ==============================================================================
// STATE DEFINITIONS
// ==============================================================================

enum DiverterState : uint8_t{
    DIVERTER_LEFT = 1,
    DIVERTER_RIGHT = 2,
    DIVERTER_SWITCHING = 3,
    DIVERTER_ERROR = 4,
    DIVERTER_STOP = 5
};

enum DiverterDirection : uint8_t{
    DIVERTER_DIRECTION_LEFT = 1,
    DIVERTER_DIRECTION_RIGHT = 2,
    DIVERTER_DIRECTION_UNKNOWN = 0
};

enum DiverterPosition : uint8_t{
    DIVERTER_POSITION_LEFT = 1,
    DIVERTER_POSITION_RIGHT = 2,
    DIVERTER_POSITION_INTERIM = 3,
    DIVERTER_POSITION_UNKNOWN = 0
};

enum DiverterStateChangeOptions : uint8_t{
    DIVERTER_STATE_NO_CHANGE = 0,
    DIVERTER_STATE_CHANGED = 1,
    DIVERTER_UNEXPECTED_CONDITION = 2
};

// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

int left_threshold = 1000;
int right_threshold = 2000;

bool global_error_status = false;
DiverterDirection diverter_track_edge_direction = DIVERTER_DIRECTION_UNKNOWN;
DiverterPosition current_diverter_position_high_level = DIVERTER_POSITION_UNKNOWN;

DiverterState diverter_current_state = DIVERTER_STOP;
DiverterState diverter_previous_state = DIVERTER_STOP;
bool error_diverter_status = false;
bool diverter_set_torque_flag = false;
int diverter_set_position_value = -1;

bool Log_info(String message);     // Logging function - Returns true if log successful

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
            current_diverter_position_high_level == DIVERTER_DIRECTION_LEFT) {
            diverter_current_state = DIVERTER_LEFT;
            return DIVERTER_STATE_CHANGED;
        }
        
        // Transition to RIGHT
        if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT && 
            current_diverter_position_high_level == DIVERTER_DIRECTION_RIGHT) {
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
        Log_info("Diverter in LEFT position - torque disabled.");
    }
    
    
    else if (diverter_current_state == DIVERTER_RIGHT) {
        // ===== RIGHT STATE =====
        diverter_set_torque_flag = false;
        Log_info("Diverter in RIGHT position - torque disabled.");
    }
    
    
    else if (diverter_current_state == DIVERTER_SWITCHING) {
        diverter_set_torque_flag = true;
        // ===== SWITCHING STATE =====
        if (diverter_track_edge_direction == DIVERTER_DIRECTION_LEFT) {
            diverter_set_position_value = left_threshold;
            Log_info("Switching to LEFT - moving to threshold: " + String(left_threshold));
        }
        else if (diverter_track_edge_direction == DIVERTER_DIRECTION_RIGHT) {
            diverter_set_position_value = right_threshold;
            Log_info("Switching to RIGHT - moving to threshold: " + String(right_threshold));
        }
    }
    
    
    else if (diverter_current_state == DIVERTER_ERROR) {
        // ===== ERROR_DIVERTER STATE =====
        diverter_set_torque_flag = false;
        Log_info("Diverter error detected! Torque disabled.");
    }
    
    
    else if (diverter_current_state == DIVERTER_STOP) {
        // ===== STOP_DIVERTER STATE =====
        diverter_set_torque_flag = false;
        Log_info("Diverter stopped - torque disabled.");
    }
}


// ==============================================================================
// MAIN CONTROL LOOP
// ==============================================================================

void main() {
    // Main control loop
    while (true) {
        DiverterStateChangeOptions stateChange = UpdateDiverterState();
        if (stateChange == DIVERTER_STATE_CHANGED) {
            Log_info("State changed to: " + String(diverter_current_state) + 
                     " from " + String(diverter_previous_state));
            Log_info("All shared variables - Global_Error_Status: " + 
                     String(global_error_status) + 
                     ", diverter_track_edge_direction: " + String(diverter_track_edge_direction) + 
                     ", current_diverter_position_high_level: " + String(current_diverter_position_high_level) + 
                     ", Error_Diverter_Status: " + String(error_diverter_status));
            UpdateDiverterActuatorInputVariables();
        } else if (stateChange == DIVERTER_UNEXPECTED_CONDITION) {
            Log_info("Diverter State Machine in UNEXPECTED CONDITION!");
        }
        // Add delay or other processing as needed
    }
}
