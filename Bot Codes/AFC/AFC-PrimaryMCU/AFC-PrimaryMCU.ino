/* AFC Values
  ---------------------------
  | Pin1 + Pin2 | Operation |
  |     01      |   CLOSE   |
  |     10      |   STAGE   |
  |     11      |   STOP    |
  ---------------------------
*/

#include <Adafruit_MCP23X17.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

// GPIO Pin Assignments
#define EMG_PIN 12
#define PANEL_LED_PIN 8
#define AFC_PIN_1 18
#define AFC_PIN_2 19
#define LIM_SWITCH 16

// Constant variables
const String CODE_ID = "AFC-Test-PrimaryMCU";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";

// Logger variables
String BOT_ID = "B";
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Bot Variables
Adafruit_MCP23X17 mcp;
Preferences prefs;
bool afcPinState = false;
unsigned long afcStartTime;
bool closingFlap = false;
unsigned long startClosingTime;
int i = 0;
bool allowedToClick = true;
unsigned long lastActionTime = millis();
bool doStaging = false;
bool doOpen = false;
bool doStop = false;

void CloseFlap() {
  digitalWrite(EMG_PIN, HIGH);    // Turn ON the electromagnets
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, HIGH);
  afcPinState = true;
  afcStartTime = millis();
}

void OpenFlap() {
  digitalWrite(EMG_PIN, LOW);
  allowedToClick = true;
}

void StageFlap() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, LOW);
  afcPinState = true;
  afcStartTime = millis();
}

void StartListening() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, HIGH);
  // afcPinState = true;
  // afcStartTime = millis();
}

void setup() {
  PinConfig();
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, HIGH);

  delay(1000);
  Serial.begin(115200);
  MCPConfig();
 
  LoadDiverterConfig();
  WiFiConfig();
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    10000,
    NULL,
    1,
    &httpDebugLog,
    0);
  StartListening();
  Serial.println("Bot: " + String(BOT_ID) + " Code: " + CODE_ID);
  Serial.println(__FILE__);
  lastActionTime = millis();
  // delay(5000);
  // add_log("Closing");
  // CloseFlap();
  closingFlap = true;
  // doOpen = true;
}

void loop() {
  if (closingFlap && millis() - lastActionTime >= 5000) {
    add_log("Closing");
    CloseFlap();
    closingFlap = false;
    doStop = true;
    lastActionTime = millis();
  }

  if (doStop && millis() - lastActionTime >= 5000) {
    add_log("Limited switch detected");
    digitalWrite(AFC_PIN_1, HIGH);
    digitalWrite(AFC_PIN_2, HIGH);
    mcp.digitalWrite(PANEL_LED_PIN, HIGH);
    doStop = false;
    doStaging = true;
    lastActionTime = millis();
  }

  if (doStaging && millis() - lastActionTime >= 5000) {
    add_log("Staging!");
    StageFlap();
    doStaging = false;
    doOpen = true;
    lastActionTime = millis();
  }

  if (doOpen && millis() - lastActionTime >= 5000) {
    add_log("Staging!");
    OpenFlap();
    doOpen = false;
    closingFlap = true;
    lastActionTime = millis();
  }

  // // Start closing the flaps after threshold time of detecting the drop-off
  // // if (closingFlap && !digitalRead(LIM_SWITCH)) {
  // // if (closingFlap && )
  // //   add_log("Limited switch detected");
  // //   digitalWrite(AFC_PIN_1, HIGH);
  // //   digitalWrite(AFC_PIN_2, HIGH);
  // //   mcp.digitalWrite(PANEL_LED_PIN, HIGH);
  // //   closingFlap = false;
  // //   afcPinState = true;
  // //   afcStartTime = millis();
  // //   doStaging = true;
  // // }

  // Reset the AFC pins state to 00 after threshold time
  if (afcPinState && millis() - afcStartTime >= 1000) {
    add_log("Default");
    digitalWrite(AFC_PIN_1, LOW);
    digitalWrite(AFC_PIN_2, LOW);
    afcPinState = false;
    allowedToClick = true;
  }

  // if (doStaging && millis() - afcStartTime >= 5000) {
  //   add_log("Staging");
  //   StageFlap();
  //   mcp.digitalWrite(PANEL_LED_PIN, LOW);
  //   // i++;
  //   allowedToClick = false;
  //   lastActionTime = millis();
  //   doStaging = false;
  //   doOpen = true;
  // }

  // if (doOpen && millis() - lastActionTime >= 5000) {
  //   add_log("Opening");
  //   OpenFlap();
  //   doOpen = false;
  //   delay(5000);
  //   add_log("Closing");
  //   CloseFlap();
  //   closingFlap = true;
  // }
}

// Function to load values from NVM
void LoadDiverterConfig() {
  prefs.begin("diverter", true);
  BOT_ID = prefs.getString("BotId", "Bx");
}

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    // while (1);
  }
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (debugLoggingString != BOT_ID + " " + CODE_ID + ": ") {
      // Serial.println("Sending logs!");
      // Serial.println(String(WiFi.status()));
      int httpCode = -1;
      if (loggerFlag) {
        httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
        httpCode = httpDebugger.POST(debugLoggingString);
        // Serial.println("Response: " + httpDebugger.getString());
      } else {
        httpCode = HTTP_CODE_OK;
      }
      if (httpCode == HTTP_CODE_OK) {
        debugLoggingString = BOT_ID + " " + CODE_ID + ": ";
      }
    }
  }
}

// Function to add logs into the log string
void add_log(String log) {
  debugLoggingString += " | " + log;
}

// Function to connect controller to network
void WiFiConfig() {
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  Serial.println("Connected to WiFi!");
}

// Setup the pin configuration
void PinConfig() {
  pinMode(EMG_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_PIN, OUTPUT);
  pinMode(LIM_SWITCH, INPUT_PULLUP);

  digitalWrite(EMG_PIN, HIGH);

  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);
}