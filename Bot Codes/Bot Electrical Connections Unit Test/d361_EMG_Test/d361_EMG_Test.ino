#include <esp32_can.h>
#include <Adafruit_MCP23X17.h>
using namespace std;

// GPIO Pin Assignments
#define EMG_PIN 12
#define PANEL_LED_PIN 8
#define PANEL_LED_BUTTON 7 

// Static Variables
const String CODE_ID = "d361-EmgStandaloneTest";

// Bot Variables
Adafruit_MCP23X17 mcp;
bool state = false;

void setup() {
  delay(1000);
  Serial.begin(115200);
  MCPConfig();
  PinConfig();
  Serial.println("Bot: BB CodeID: " + CODE_ID);
  Serial.println(__FILE__);
}

void loop() {
  if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
    state = !state;
    digitalWrite(EMG_PIN, state);
    if (state) {
      Serial.println("EMG ON");
      mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    } else {
      Serial.println("EMG OFF");
      mcp.digitalWrite(PANEL_LED_PIN, LOW);
    }
    delay(2000);
  }
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
  pinMode(EMG_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
}