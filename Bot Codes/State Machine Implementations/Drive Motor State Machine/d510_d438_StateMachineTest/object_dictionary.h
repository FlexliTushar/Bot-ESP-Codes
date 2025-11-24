// Motor Speed Levels
#define S0_05         26    //0.05m/s
#define S0_1          53    //0.1m/s
#define S0_5          267   //0.5m/s
#define S1            535   //1m/s
#define S1_5          802   //1.5m/s
#define S2            1070  //2m/s
#define S2_5          1337  //2.5m/s
#define S3            1604  //3m/s

//Modes of operation

#define CAN_DELAY               5  //10ms delay to prevent CAN Rx queue overflow

#define OPERATION_MODE          0x6060

#define PROFILE_POSITION_MODE   1
#define PROFILE_VELOCITY_MODE   3
#define HOMING_MODE             6

//Set Parameters
#define FOLLOWING_ERROR_WINDOW  0x6065
#define MAX_PROFILE_VELOCITY    0x607F
#define PROFILE_VELOCITY        0x6081
#define PROFILE_ACCELERATION    0x6083
#define PROFILE_DECELERATION    0x6084
#define QUICK_STOP_DECELERATION 0x6085
#define MOTION_PROFILE_TYPE     0x6086

#define CONTROLWORD             0x6040

#define TARGET_POSITION         0x607A
#define TARGET_VELOCITY         0x60FF

#define VELOCITY_ACTUAL_VALUE   0x606C
#define VELOCITY_ACTUAL_VALUE_AVG 0x30D3
#define CURRENT_ACTUAL_VALUE_AVG  0x30D1 // Sun Index 0x01
#define CURRENT_ACTUAL_VALUE      0x30D1 // Sun Index 0x02

#define ABSOLUTE_POSITION       31
#define ABSOLUTE_POSITION_START_IMMEDIATELY   63
#define RELATIVE_POSITION_START_IMMEDIATELY   127
#define RELATIVE_POSITION       95

#define ABSOLUTE 1
#define RELATIVE 0