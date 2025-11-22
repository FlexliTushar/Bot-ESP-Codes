#include <Adafruit_MCP23X17.h>

// GPIO Pin Assignments
#define PANEL_LED_PIN 8
#define COLUMN_INDICATOR_SENSOR 33
#define TRAFFIC_INDICATOR_SENSOR 35
#define BOT_INDICATOR_SENSOR 19

// Constant variables
const String CODE_ID = "d360-WTMStandaloneTest";
const int THRESHOLD_INTENSITY = 250;

// Bot Variables
Adafruit_MCP23X17 mcp;

// Function which tells if traffic indicator is detected or not
bool TrafficIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int TI_rawValue = map(analogRead(TRAFFIC_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then traffic detected else not detected
  if (TI_rawValue < THRESHOLD_INTENSITY) {
    return true;
  }
  return false;
}

// Function which tells if column indicator is detected or not
bool ColumnIndicatorDetection() {
  // Analog read column detection sensor value and map to 10 bit
  int CI_rawValue = map(analogRead(COLUMN_INDICATOR_SENSOR), 0, 4096, 0, 1024);
  // Case: If condition true then column detected else not detected
  if (CI_rawValue < THRESHOLD_INTENSITY) {
    return true;
  }
  return false;
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  MCPConfig();
  PinConfig();
}

void loop() {
  bool columnDetected = ColumnIndicatorDetection();
  bool trafficDetected = TrafficIndicatorDetection();
  if (columnDetected) {
    if (trafficDetected) {
      mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    } else {
      mcp.digitalWrite(PANEL_LED_PIN, LOW);
    }
  } else {
    mcp.digitalWrite(PANEL_LED_PIN, LOW);
  }
  Serial.println("CI: " + String(columnDetected) + " TI: " + String(trafficDetected));
}

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }
}

// Setup the pin configuration
void PinConfig() {
  pinMode(COLUMN_INDICATOR_SENSOR, INPUT);       // Traffic indicator pin config
  pinMode(BOT_INDICATOR_SENSOR, OUTPUT);         // Bot detection sensor pin config
  pinMode(TRAFFIC_INDICATOR_SENSOR, INPUT);      // Column indicator pin config
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);

  digitalWrite(BOT_INDICATOR_SENSOR, LOW);
}