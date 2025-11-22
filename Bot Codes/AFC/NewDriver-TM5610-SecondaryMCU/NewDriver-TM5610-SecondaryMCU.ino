#include <TMCStepper.h>

// GPIO communication pins with master ESP32
#define INPUT1_PIN 15  // Command input 1 from master
#define INPUT2_PIN 2   // Command input 2 from master

#define EN_PIN           18 // Enable
#define DIR_PIN          4  // Direction
#define STEP_PIN         19 // Step
#define CS_PIN           16 // Chip select
#define SW_MOSI          23 // Software MOSI
#define SW_MISO          17 // Software MISO
#define SW_SCK           5  // Software SCK
#define LIMIT_SWITCH_PIN 14 // Limit switch input

#define R_SENSE 0.11f 

#define STAGE_STEPS 60000

const String CODE_ID = "AFC-SecondaryMCU";

// Select driver
TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

bool shaft = true;
String prevInput = "00";
bool firstTime = true;
String state = "DEFAULT";
int stageIteration = 0;

void stepMotor(uint16_t steps, uint16_t delay_us = 70) {
  for (uint16_t i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delay_us);
  }
}

void SetDirection(bool direction) {
  driver.shaft(direction);
}

// Close until limit switch is pressed
void closeUnit() {
  if (state == "CLOSING") { // run until switch pressed
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(70);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(70);
  }
}

// Open for fixed steps
void openUnit() {
  if (stageIteration < STAGE_STEPS && state == "STAGING") {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(70);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(70);
    stageIteration++;
  } else if (state == "STAGING" && stageIteration == STAGE_STEPS){
    stageIteration = 0;
    state = "DEFAULT";
  }
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial);
  PinConfig();
  MotorConfig();
  delay(5000);
  Serial.println("Code ID: " + CODE_ID);
  Serial.println(__FILE__);
}

void loop() {
  String input = String(digitalRead(INPUT1_PIN)) + String(digitalRead(INPUT2_PIN));
  if (prevInput != input) {
    Serial.println(input);
    // if (firstTime) {
    //   if (prevInput == "11") {
    //     Serial.println("Starting to listen");
    //     if (input == "01") {
    //       Serial.println("Asked to close!");
    //       state = "CLOSING";
    //       Serial.println("Closing...");
    //       SetDirection(true);
    //       firstTime = false;
    //     }
    //   }
    // } else {
    //   if (input == "01") {
    //     Serial.println("Asked to close!");
    //     state = "CLOSING";
    //     Serial.println("Closing...");
    //     SetDirection(true);
    //   } else if (input == "10") {
    //     Serial.println("Asked to stage!");
    //     state = "STAGING";
    //     Serial.println("Staging...");
    //     SetDirection(false);
    //   } else if (input == "11") {
    //     Serial.println("Asked to stop motor!");
    //     state = "DEFAULT";
    //     Serial.println("Stopping...");
    //   }
    // }
  }
  prevInput = input;
  // closeUnit();
  // openUnit();
}

void PinConfig() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
}

void MotorConfig() {
  driver.begin();             // Init driver
  driver.rms_current(600);    // Set motor RMS current
  driver.en_pwm_mode(1);      // Enable stealthChop
  driver.microsteps(16);      // 1/16 microstepping
  digitalWrite(EN_PIN, LOW);  // Enable driver
  Serial.print("DRV_STATUS=0b");
  Serial.println(driver.DRV_STATUS(), BIN);
  driver.shaft(shaft);
}