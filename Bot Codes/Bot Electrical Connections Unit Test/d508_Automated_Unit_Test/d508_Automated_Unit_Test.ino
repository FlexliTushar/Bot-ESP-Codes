#include <Adafruit_MCP23X17.h>
#include <esp32_can.h>
#include <can_common.h>
#include <object_dictionary.h>
#include <maxon.h>
#include <SMS.h>
#include <Preferences.h>

Adafruit_MCP23X17 mcp;
SMS sm;
Preferences prefs;
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define ENABLE 1
#define DISABLE 0
#define CLK 27
#define DT 26
#define PIN_AFC_OUT 18
#define PIN_AFC_IN 19

// Test state machine
int currentTest = 1;

// Drive motor target
const int targetRPM = 26;        // Dummy mapping for 0.05 m/s → RPM (replace with real formula)
const int rpmTolerance = 5;      // ±5 RPM

// Assign MCP pin for button
const uint8_t PANEL_BUTTON_PIN = 7;
const uint8_t PANEL_LED_PIN = 8;

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

// Station Reader Diodes
const uint8_t DIODE1_PIN = 36;
const uint8_t DIODE2_PIN = 39;
const uint8_t DIODE3_PIN = 34;
const int lowerThreshold = 400;
const int upperThreshold = 600;
const static int MAX_REATTEMPTS_FOR_SERVO_COM = 3;
const int DIVERTER_POSITION_ERROR_LIMIT = 30;
bool enableTesting = false;

// === Define pins for WTM ===
const uint8_t COLUMN_PIN  = 33;  // Example ADC pin, adjust as per wiring
const uint8_t TRAFFIC_PIN = 35;

const uint8_t PIN_ELECTROMAGNET = 12;

int lastA = 0, lastB = 0;
volatile int pulseCount = 0;

void IRAM_ATTR EncoderISR() {
  int currentA = digitalRead(CLK);
  int currentB = digitalRead(DT);
  if (currentA != lastA) {
    if (currentA != currentB) {
    pulseCount++; // Clockwise rotation (Phase A and B differ)
    } else {
    pulseCount--; // Counter-clockwise rotation (Phase A and B same)
    }
  }
  lastA = currentA;
  lastB = currentB;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!mcp.begin_I2C()) { // defaults to 0x20 addr
    Serial.println("MCP23017 not found. Check wiring.");
    while (1);
  }
  attachInterrupt(digitalPinToInterrupt(CLK), EncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT), EncoderISR, CHANGE);

  // Configure button pin as input with pullup
  mcp.pinMode(PANEL_BUTTON_PIN, INPUT_PULLUP);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  mcp.digitalWrite(PANEL_LED_PIN, HIGH);
  pinMode(COLUMN_PIN, INPUT);
  pinMode(TRAFFIC_PIN, INPUT);
  pinMode(PIN_ELECTROMAGNET, OUTPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(PIN_AFC_OUT, OUTPUT);
  pinMode(PIN_AFC_IN, INPUT);
  CANConfig();
  MotorConfig();
  ServoConfig();
  LoadDiverterConfig();
  Serial.println(__FILE__);

  Serial.println("=== Starting Bot Connection Test ===");
  Serial.println("Test 1: Panel Button");
}

void loop() {

  if (enableTesting && mcp.digitalRead(PANEL_BUTTON_PIN)) { return; }
  Serial.println(mcp.digitalRead(PANEL_BUTTON_PIN));

  switch (currentTest) {
    case 1: {
      // Button test
      int state = mcp.digitalRead(PANEL_BUTTON_PIN);
      if (state == LOW) { // active LOW
        Serial.println("Test 1 Passed: Button detected");
        delay(500);
        currentTest++;
        Serial.println("Test 2: Button LED (observe blinking)");
        enableTesting = true;
      } else {
        Serial.println("Test 1 failed or button not pressed yet");
        delay(1000);
      }
      break;
    }

    case 2: {
      // Button LED blink test
      for (int i = 1; i <= 20; i++) {
        mcp.digitalWrite(PANEL_LED_PIN, HIGH);
        delay(500);
        mcp.digitalWrite(PANEL_LED_PIN, LOW);
        delay(500);
        Serial.print("Blink ");
        Serial.print(i);
        Serial.println("/20");
      }
      Serial.println("Test 2 complete: If operator saw blinking → PASSED");
      Serial.println("If operator saw steady ON or OFF → FAILED");
      delay(1000);
      currentTest++;
      Serial.println("Test 3: Drive Motor");
      break;
    }

    case 3: {
      // Drive motor test
      Serial.print("Actuating drive motor at S0_05 = ");
      Serial.print(S0_05);
      Serial.println(" m/s");
      enable();
      delay(1000);
      setTagetVelocity(targetRPM); // Dummy function to command motor
      delay(1000);                 // Wait for motor to stabilize

      int withinRange = 0;
      for (int i = 0; i < 5; i++) {
        int feedback = getVelocityActualValueAveraged(); // Dummy function to read feedback
        Serial.print("Feedback ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(feedback);

        if (abs(feedback - targetRPM) <= rpmTolerance) {
          withinRange++;
        }
        delay(200); // spacing between samples
      }

      haltMovement();
      disable();

      if (withinRange >= 3) {
        Serial.println("Test 3 Passed: Drive motor feedback within tolerance");
      } else {
        Serial.println("Test 3 Failed: It will affect test 5 and 6");
      }

      delay(1000);
      currentTest++;
      Serial.println("Test 4: Diverter Motor");
      break;
    }

    case 4: {
      // Diverter motor test
      bool leftOK = false;
      bool rightOK = false;

      Serial.println("Moving diverter to LEFT...");
      SetDiverter(false);
      delay(5000);
      if (CheckDiverter(false)) {
        leftOK = true;
        Serial.println("LEFT position confirmed");
      } else {
        Serial.println("LEFT position mismatch");
      }

      Serial.println("Moving diverter to RIGHT...");
      SetDiverter(true);
      delay(5000);
      if (CheckDiverter(true)) {
        rightOK = true;
        Serial.println("RIGHT position confirmed");
      } else {
        Serial.println("RIGHT position mismatch");
      }

      if (leftOK && rightOK) {
        Serial.println("Test 4 Passed: Diverter motor moved correctly");
      } else {
        Serial.println("Test 4 Failed: Diverter motor did not reach expected positions");
      }

      delay(1000);
      currentTest++;
      Serial.println("Test 5: Station Photo Diode");
      break;
    }

    // Station Reader Diode Test
    case 5: {
      Serial.println("Running bot at S0_05 to check station reader diodes...");
      enable();
      delay(1000);
      setTagetVelocity(targetRPM);
      unsigned long startTime = millis();
      bool allHighOnce = false;

      while (millis() - startTime < 5000) {
        int d1 = analogRead(DIODE1_PIN) / 4; // scale 0–4096 → 0–1024
        int d2 = analogRead(DIODE2_PIN) / 4;
        int d3 = analogRead(DIODE3_PIN) / 4;

        bool d1High = (d1 > upperThreshold);
        bool d2High = (d2 > upperThreshold);
        bool d3High = (d3 > upperThreshold);

        bool d1Low = (d1 < lowerThreshold);
        bool d2Low = (d2 < lowerThreshold);
        bool d3Low = (d3 < lowerThreshold);

        Serial.print("D1: "); Serial.print(d1);
        Serial.print(" D2: "); Serial.print(d2);
        Serial.print(" D3: "); Serial.println(d3);

        if (d1High && d2High && d3High) {
          allHighOnce = true;
          break;
        }

        // Optional: log when all low (debugging)
        if (d1Low && d2Low && d3Low) {
          Serial.println("All LOW state observed");
        }

        delay(100);
      }
      haltMovement();
      disable();

      if (allHighOnce) {
        Serial.println("Test 5 Passed: All diodes HIGH together observed");
      } else {
        Serial.println("Test 5 Failed: Never observed all diodes HIGH simultaneously");
      }

      delay(1000);
      currentTest++;
      Serial.println("Test 6: WTM Readers");
      break;
    }

    case 6: {
      Serial.println("Running WTM test (Column + Traffic indicators)...");
      enable();
      delay(1000);
      setTagetVelocity(targetRPM);
      unsigned long startTime = millis();

      // Tracking states
      int prevColumnState = -1;
      int prevTrafficState = -1;

      int colTransitions = 0;
      int trafTransitions = 0;

      bool colLowHigh = false;
      bool colHighLow = false;
      bool trafLowHigh = false;
      bool trafHighLow = false;

      while (millis() - startTime < 5000) {
        int colRaw = analogRead(COLUMN_PIN) / 4;
        int trafRaw = analogRead(TRAFFIC_PIN) / 4;

        int colState = -1;  // -1=undefined, 0=LOW, 1=HIGH
        int trafState = -1;

        if (colRaw < lowerThreshold) colState = 0;
        else if (colRaw > upperThreshold) colState = 1;

        if (trafRaw < lowerThreshold) trafState = 0;
        else if (trafRaw > upperThreshold) trafState = 1;

        Serial.print("Column: "); Serial.print(colRaw);
        Serial.print(" (state "); Serial.print(colState); Serial.print(") ");
        Serial.print("Traffic: "); Serial.print(trafRaw);
        Serial.print(" (state "); Serial.print(trafState); Serial.println(")");

        // Column transitions
        if (prevColumnState != -1 && colState != -1 && colState != prevColumnState) {
          colTransitions++;
          if (prevColumnState == 0 && colState == 1) colLowHigh = true;
          if (prevColumnState == 1 && colState == 0) colHighLow = true;
        }
        if (colState != -1) prevColumnState = colState;

        // Traffic transitions
        if (prevTrafficState != -1 && trafState != -1 && trafState != prevTrafficState) {
          trafTransitions++;
          if (prevTrafficState == 0 && trafState == 1) trafLowHigh = true;
          if (prevTrafficState == 1 && trafState == 0) trafHighLow = true;
        }
        if (trafState != -1) prevTrafficState = trafState;

        delay(100);
      }
      haltMovement();
      disable();
      // Evaluation
      bool pass = true;
      if (!(colLowHigh && colHighLow)) {
        Serial.println("Column indicator missing required transitions");
        pass = false;
      }
      if (!(trafLowHigh && trafHighLow)) {
        Serial.println("Traffic indicator missing required transitions");
        pass = false;
      }
      if (colTransitions != trafTransitions) {
        Serial.println("Transition count mismatch between column and traffic");
        pass = false;
      }

      if (pass) {
        Serial.println("Test 6 Passed: WTM transitions valid and matched");
      } else {
        Serial.println("Test 6 Failed");
      }

      delay(1000);
      currentTest++;
      Serial.println("Test 7: (Dummy)");
      break;
    }

    case 7:
      testElectromagnetAndFlap();
      break;

    case 8:
      testEncoder();
      break;

    // case 9:
    //   testSecondaryAFC();
    //   break;

    default:
      Serial.println("All tests finished.");
      while (1);
  }
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

// Function to configure the CAN protocol
void CANConfig() {
  CAN0.setCANPins(GPIO_NUM_15, GPIO_NUM_14);  // rx tx
  CAN0.begin(500000);
  Serial.println("CAN Ready!");
}

// Define the configuration for the servo object
void ServoConfig() {
  sm.begin(&Serial2, 115200);
  sm.setTimeOut(100);
  Serial2.begin(115200, SERIAL_8N1, 25, 32);
  sm.IncreaseLimit(2, 5000);
}

// Function to load diverter position values from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);

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

// Diverter function - For Left
void DivertLeft() {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    Serial.println("Servo comm. failed - front!");
    return;
  }
  delay(1);
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    Serial.println("Servo comm. failed - rear!");
    return;
  }
  delay(1);
  if (current_position_front_servo < front_diverter_left_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - front!");
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_left_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - front!");
      return;
    }
    delay(10);
  }
  
  if (current_position_rear_servo > rear_diverter_left_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(REAR_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - rear!");
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_left_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - rear!");
      return;
    }
    delay(1);
  }
}

// Diverter function - For Right
void DivertRight() {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    Serial.println("Servo comm. failed - front!");
    return;
  }
  delay(1);
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    Serial.println("Servo comm. failed - rear!");
    return;
  }
  delay(1);
  if (current_position_front_servo > front_diverter_right_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(FRONT_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - front!");
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(FRONT_SERVO, front_diverter_right_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - front!");
      return;
    }
    delay(10);
  }
  
  if (current_position_rear_servo < rear_diverter_right_thresold) {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.EnableTorque(REAR_SERVO, DISABLE);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - rear!");
      return;
    }
    delay(1);
  } else {
    code = 1;
    count = 0;
    while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
      sm.WritePosEx(REAR_SERVO, rear_diverter_right_thresold, 100);
      code = sm.getLastError();
      count++;
    }
    if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
      Serial.println("Servo comm. failed - rear!");
      return;
    }
    delay(1);
  }
}

// Diverter function - Setting the direction
void SetDiverter(bool direction) {
  if (direction) {
    DivertRight();
  } else {
    DivertLeft();
  }
}

bool CheckDiverter(bool direction) {
  int code = 1;
  int count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_front_servo = sm.ReadPos(FRONT_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    Serial.println("Servo comm. failed - front!");
    return false;
  }
  delay(1);
  code = 1;
  count = 0;
  while(code && count < MAX_REATTEMPTS_FOR_SERVO_COM) {
    current_position_rear_servo = sm.ReadPos(REAR_SERVO);
    code = sm.getLastError();
    count++;
  }
  if ((count == MAX_REATTEMPTS_FOR_SERVO_COM && code == 1) || (count > MAX_REATTEMPTS_FOR_SERVO_COM)) {
    Serial.println("Servo comm. failed - rear!");
    return false;
  }
  delay(1);
  if (direction) {
    if (current_position_front_servo > front_diverter_right_thresold - DIVERTER_POSITION_ERROR_LIMIT && current_position_rear_servo < rear_diverter_right_thresold + DIVERTER_POSITION_ERROR_LIMIT) {
      // Correct diversion
      return true;
    } else {
      return false;
    }
  } else {
    if (current_position_front_servo < front_diverter_left_thresold + DIVERTER_POSITION_ERROR_LIMIT && current_position_rear_servo > rear_diverter_left_thresold - DIVERTER_POSITION_ERROR_LIMIT) {
      // Correct diversion
      return true;
    } else {
      return false;
    }
  }
  return false;
}

void testElectromagnetAndFlap() {
  Serial.println("Test 7: Electromagnet + Flap close");

  digitalWrite(PIN_ELECTROMAGNET, HIGH);  // Power ON
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    digitalWrite(PANEL_LED_PIN, HIGH);
    delay(500);
    digitalWrite(PANEL_LED_PIN, LOW);
    delay(500);
  }

  Serial.println("Test 7 PASSED: If flap latched else FAILED");

  digitalWrite(PIN_ELECTROMAGNET, LOW);   // Switch OFF
  delay(1000);
  currentTest++;
}

void testEncoder() {
  Serial.println("Test 8: Encoder");

  SetEncoder(0); // Reset encoder
  enable();
  delay(1000);
  setTagetVelocity(S0_05);  // Run bot @ 0.05 m/s
  delay(5000);
  haltMovement();
  disable()
;
  int pulse = GetCurrentPulse(); // <-- your function
  if (pulse > 0) {
    Serial.println("Test 8 PASSED: Encoder detected pulses");
  } else {
    Serial.println("Test 8 FAILED: Encoder did not detect pulses");
  }
  delay(1000);
  currentTest++;
}

int GetCurrentPulse() {
  return pulseCount;
}

void SetEncoder(const int& value) {
  pulseCount = value;
}

void testSecondaryAFC() {
  Serial.println("Test 9: Secondary MCU AFC Comm");

  const int sendPin = PIN_AFC_OUT;
  const int recvPin = PIN_AFC_IN;
  pinMode(sendPin, OUTPUT);
  pinMode(recvPin, INPUT);

  int pattern[8] = {1,0,1,1,0,0,1,1};  // example pattern
  bool receivedOK = false;

  unsigned long startTime = millis();
  while (millis() - startTime < 30000) {
    // Send pattern
    for (int i = 0; i < 8; i++) {
      digitalWrite(sendPin, pattern[i]);
      delay(100);
    }

    // Read back
    int readPattern[8];
    while(!digitalRead(recvPin)) {}
    for (int i = 0; i < 8; i++) {
      readPattern[i] = digitalRead(recvPin);
      delay(100);
    }

    // Compare
    bool match = true;
    for (int i = 0; i < 8; i++) {
      if (pattern[i] != readPattern[i]) {
        match = false;
        break;
      }
    }
    if (match) {
      receivedOK = true;
      break;
    }
  }

  if (receivedOK) {
    Serial.println("Test 9 PASSED: Pattern echoed back correctly");
  } else {
    Serial.println("Test 9 FAILED: No valid response in 30s");
  }
  delay(1000);
  currentTest++;
}