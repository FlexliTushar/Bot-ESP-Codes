#define PHASE_A 27
#define PHASE_B 26

volatile int count = 0;
int lastA = 0;
int lastB = 0;

void IRAM_ATTR pulseCounter() {
  int currentA = digitalRead(PHASE_A);
  int currentB = digitalRead(PHASE_B);

  // Check for state change on Phase A
  if (currentA != lastA) {
    // Check the state of Phase B to determine direction
    if (currentA != currentB) {
      count++; // Clockwise rotation (Phase A and B differ)
    } else {
      count--; // Counter-clockwise rotation (Phase A and B same)
    }
  }

  lastA = currentA;
  lastB = currentB;
}

void setup() {

  pinMode(PHASE_A, INPUT_PULLUP);
  pinMode(PHASE_B, INPUT_PULLUP);

  // Use interrupt service routine for efficient pulse counting
  attachInterrupt(digitalPinToInterrupt(PHASE_A), pulseCounter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PHASE_B), pulseCounter, CHANGE);

  Serial.begin(115200);
}

void loop() {
  Serial.println(count);
  delay(10);
}
