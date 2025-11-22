#include <Ticker.h>
#include <TMCStepper.h>

// ─── Pin Definitions ───
#define EMG_PIN 12

#define EN_PIN 18
#define STEP_PIN 19
#define DIR_PIN 5
#define RX_PIN 16  // TMC2209 PDN_UART → ESP32 RX2
#define TX_PIN 17  // TMC2209 PDN_UART ← ESP32 TX2
#define READ_PIN_1 15
#define READ_PIN_2 2

#define R_SENSE 0.075f
#define DRIVER_ADDR 0b00

#define STALL_THRESHOLD 325  // Adjust as needed

const String CODE_ID = "d365";

HardwareSerial MySerial(2);
TMC2209Stepper driver(&MySerial, R_SENSE, DRIVER_ADDR);

Ticker stepTicker;

bool dir = true;
volatile bool stepState = false;
uint32_t step_interval_us = 70;
bool motorRunning = false;

bool ignoreSG = true;
unsigned long startTime;

// Step counting variables
volatile long stepCount = 0;
volatile long targetSteps = 0;
bool stepCountMode = false;
unsigned 

// Movement states
enum MovementState {
  IDLE,
  OPENING,
  CLOSING,
  STEP_COUNTING
};

MovementState currentState = IDLE;

// ─── ISR: Pulse STEP pin ───
void IRAM_ATTR onStepTicker() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(STEP_PIN, LOW);

  // Increment step count if in step counting mode
  if (stepCountMode) {
    stepCount++;
    if (stepCount >= targetSteps) {
      stepTicker.detach();
      digitalWrite(EN_PIN, HIGH);  // Disable driver
      motorRunning = false;
      stepCountMode = false;
      currentState = IDLE;
      Serial.println("Step count completed!");
    }
  }
}

void stopMotorDueToStall() {
  stepTicker.detach();
  digitalWrite(EN_PIN, HIGH);  // Disable driver
  motorRunning = false;

  // switch (currentState) {
  //   case OPENING:
  //     // Serial.println("Stalled while OPENING → switching to CLOSING...");
  //     currentState = CLOSING;
  //     startMotor(false);  // Switch direction
  //     break;

  //   case CLOSING:
  //     // Serial.println("Stalled while CLOSING → switching to OPENING...");
  //     currentState = OPENING;
  //     startMotor(true);  // Switch direction
  //     break;

  //   case STEP_COUNTING:
  //     Serial.println("Motor stalled during step counting!");
  //     stepCountMode = false;
  //     currentState = IDLE;
  //     break;

  //   default:
  //     // Serial.println("Unexpected state. Resetting to IDLE.");
  //     currentState = IDLE;
  //     break;
  // }
}

void startMotor(bool direction) {
  dir = direction;
  digitalWrite(EN_PIN, LOW);  // Enable driver
  driver.shaft(dir);
  // delay(30);

  stepTicker.attach_us(step_interval_us, onStepTicker);
  motorRunning = true;

  // Reset stall detection timer
  ignoreSG = true;
  startTime = millis();
}

// New function: Move specific number of steps in given direction
void moveSteps(bool direction, long steps) {
  if (motorRunning) {
    Serial.println("Motor already running! Stop current operation first.");
    return;
  }

  if (steps <= 0) {
    Serial.println("Invalid step count! Must be greater than 0.");
    return;
  }

  Serial.print("Moving ");
  Serial.print(steps);
  Serial.print(" steps in ");
  Serial.println(direction ? "CW" : "CCW");

  // Initialize step counting variables
  stepCount = 0;
  targetSteps = steps;
  stepCountMode = true;
  currentState = STEP_COUNTING;

  startMotor(direction);
}

void stage() {
  moveSteps(1, 80000); //60000
}

void open() {
  if (motorRunning) {
    // Serial.println("Motor already running! Stop current operation first.");
    return;
  }

  // Serial.println("Opening - rotating CW until stall...");
  stepCountMode = false;
  currentState = OPENING;
  startMotor(true);  // CW direction
}

void close() {
  if (motorRunning) {
    // Serial.println("Motor already running! Stop current operation first.");
    return;
  }

  // Serial.println("Closing - rotating CCW until stall...");
  stepCountMode = false;
  currentState = CLOSING;
  startMotor(false);  // CCW direction
}

void stopMotor() {
  if (motorRunning) {
    stepTicker.detach();
    digitalWrite(EN_PIN, HIGH);  // Disable driver
    motorRunning = false;
    stepCountMode = false;
    currentState = IDLE;
    Serial.println("Motor stopped manually.");
  }
}

bool allowedToRead = true;
unsigned long allowedToReadStartTime;
String input = "00";
String prevInput = "00";
bool firstTime = true;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting TMC2209 SG-based stall detection...");

  pinMode(READ_PIN_1, INPUT);
  pinMode(READ_PIN_2, INPUT);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // Disable driver initially

  MySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(1500);
  driver.microsteps(16);
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  // driver.TCOOLTHRS(0xFFFFF);  // Enable SG at all speeds

  Serial.println("Driver initialized.");
  Serial.println("Code ID: " + CODE_ID);
  Serial.println(__FILE__);
  // Serial.println("Commands: 'o' = open, 'c' = close, 's' = stop");
  // Serial.println("Step mode: 'm<direction><steps>' - e.g., 'm1200' (CW 200 steps), 'm0100' (CCW 100 steps)");
  // open();
}

void loop() {
  // Handle stall detection ignore period
  // if (ignoreSG && motorRunning && millis() - startTime > 1000) {
  //   ignoreSG = false;
  //   // Serial.println("Stall detection enabled.");
  // }

  // Monitor SG_RESULT periodically (only for stall-based modes, not step counting)
  static unsigned long lastCheck = 0;
  // if (!ignoreSG && motorRunning && !stepCountMode && millis() - lastCheck > 200) {
  //   lastCheck = millis();
  //   uint16_t sg = driver.SG_RESULT();
  //   Serial.print("SG_RESULT = ");
  //   Serial.println(sg);

  //   // if (sg < STALL_THRESHOLD) {
  //   //   stopMotorDueToStall();
  //   // }
  // }

  input = String(digitalRead(READ_PIN_1)) + String(digitalRead(READ_PIN_2));
  if (prevInput != input) {
    Serial.println(input);
    if (firstTime) {
      if (prevInput == "11") {
        if (input == "01") {
          Serial.println("Asked to close!");
          close();
          firstTime = false;
        }
      }
    } else {
      if (input == "01") {
        Serial.println("Asked to close!");
        close();
      } else if (input == "10") {
        Serial.println("Asked to stage!");
        stage();
      } else if (input == "11") {
        Serial.println("Asked to stop the motor!");
        delay(200);
        stopMotorDueToStall();
      }
    }
    prevInput = input;
  }
}