#include <SPI.h>
#include <HighPowerStepperDriver.h>

const uint8_t DirPin = 14;
const uint8_t StepPin = 21;
const uint8_t CSPin  = 10;

// GPIO command inputs from Primary
#define AFC_IO_OPEN  4
#define AFC_IO_CLOSE 5
#define DIR_OPEN  1
#define DIR_CLOSE 0
#define AFC_TOP_LIM_SWITCH 48
#define AFC_BOTTOM_LIM_SWITCH 47

const uint16_t StepPeriodUs = 100;

HighPowerStepperDriver sd;
String prevCmd = "00";
bool motorOnForClose = false;
bool motorOnForStage = false;

void CloseFlap() {
  if (motorOnForClose) {
    setDirection(DIR_CLOSE);
    step();
  }
}

void StageFlap() {
  if (motorOnForStage) {
    setDirection(DIR_OPEN);
    step();
  }
}

int counter = 0;

bool canListen() {
  bool openCmd  = digitalRead(AFC_IO_OPEN);
  bool closeCmd = digitalRead(AFC_IO_CLOSE);

  String cmd = String(openCmd) + String(closeCmd);
  // Serial.println(cmd);
  // Serial.println("In the loop...");
  if ((cmd == "01" || cmd == "00" || cmd == "11" || cmd == "10") && prevCmd == cmd) {
    counter++;
    if (counter == 10) {
      return true;
    }
  } else {
    counter = 0;
  }
  prevCmd = cmd;
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(AFC_IO_OPEN, INPUT_PULLUP);
  pinMode(AFC_IO_CLOSE, INPUT_PULLUP);
  pinMode(AFC_TOP_LIM_SWITCH, INPUT_PULLUP);
  pinMode(AFC_BOTTOM_LIM_SWITCH, INPUT_PULLUP);

  SPI.begin();
  sd.setChipSelectPin(CSPin);
  pinMode(StepPin, OUTPUT);
  pinMode(DirPin, OUTPUT);

  sd.resetSettings();
  sd.clearStatus();
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  sd.setCurrentMilliamps36v4(1200);
  sd.setStepMode(HPSDStepMode::MicroStep8);
  sd.enableDriver();

  Serial.println("Secondary ready.");
  while (!canListen()) {}
  Serial.println("Out of while loop....");
}

void loop() {
  bool openCmd  = digitalRead(AFC_IO_OPEN);
  bool closeCmd = digitalRead(AFC_IO_CLOSE);

  String cmd = String(openCmd) + String(closeCmd);
  //Serial.println(cmd);
  if (cmd != prevCmd) {
    Serial.println(cmd);
    if (cmd == "10") {
      motorOnForStage = true;
    } else if (cmd == "01") {
      motorOnForClose = true;
    }
  }

  if (motorOnForClose && !digitalRead(AFC_TOP_LIM_SWITCH)) {
    motorOnForClose = false;
  }

  if (motorOnForStage && !digitalRead(AFC_BOTTOM_LIM_SWITCH))  {
    motorOnForStage = false;
  }

  CloseFlap();

  StageFlap();

  prevCmd = cmd;

  delayMicroseconds(StepPeriodUs);
}

void step() {
  digitalWrite(StepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(StepPin, LOW);
  delayMicroseconds(3);
}

void setDirection(bool dir) {
  delayMicroseconds(1);
  digitalWrite(DirPin, dir);
  delayMicroseconds(1);
}