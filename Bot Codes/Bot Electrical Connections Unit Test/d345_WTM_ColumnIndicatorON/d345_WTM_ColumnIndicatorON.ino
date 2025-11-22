#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <Update.h>
using namespace std;

#define NODE_COUNT 20

// GPIO Pin Assignments
#define ZONE_MATRIX_PIN 12
#define BOOT_BUTTON_PIN 0

const int botIndicatorPins[] = { 36, 39, 34, 35, 32, 33, 25, 26, 27, 14,
                         13, 23, 2, 19, 18, 5, 17, 16, 4, 15 };

// Constant variables
const String CODE_ID = "d345-WTMv2.2.36v2";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String OTA_URL = "http://192.168.2.109:5000/download_WTM";

// Logger variables
volatile bool otaFlag = false;

// Zone Variables
Preferences preferences;
Adafruit_NeoPixel zoneMatrix(20, ZONE_MATRIX_PIN, NEO_RGB + NEO_KHZ800);
bool virtualZoneMatrix[3][20] = { 0 };
uint8_t currentZoneID;
int columnCount;
String zoneType;
int prevPosition = -1;

// Interrupt service routine for boot button press
void IRAM_ATTR ButtonISR() {
  otaFlag = true;
}

// Set the values of LED column
// (r -> Bot Indicator, g -> Traffic Indicator, b -> Column Indicator)
void SetVirtualColumnValue(int i, uint8_t r, uint8_t g, uint8_t b) {
  virtualZoneMatrix[0][i] = (255 == r);
  virtualZoneMatrix[1][i] = (255 == g);
  virtualZoneMatrix[2][i] = (255 == b);
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOOT_BUTTON_PIN), ButtonISR, FALLING);
  BoardConfig();
  NeoPixelConfig();
  for(int i = 0 ; i < 20 ; i++) {
    pinMode(botIndicatorPins[i], INPUT);
  }

  // Turn ON column indicators
  for (int i = 0; i < columnCount ; i++) {
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(255, 0, 255));
    SetVirtualColumnValue(i, 255, 0, 255);
  }
  zoneMatrix.show();
  Serial.println("Zone: Z: " + String(currentZoneID) + " Code: " + CODE_ID);
  Serial.println(__FILE__);
  // ---------------------------------------------------------------------------------------------------------------------

  // -------------------------------------------- Virtual Bot Initialization ---------------------------------------------
  // ---------------------------------------------------------------------------------------------------------------------
}

void loop() {
  // ---------------------------------------------------------------------- OTA Check -----------------------------------------------------------------------
  if (otaFlag) {
    Serial.println("Taking OTA!");
    WiFiConfig();
    CheckForOTAUpdate();
  }
  for (int i = 0 ; i < columnCount ; i++) {
    if (!digitalRead(botIndicatorPins[i]) && i != prevPosition) {
      if (prevPosition != -1) {
        zoneMatrix.setPixelColor(prevPosition, zoneMatrix.Color(255, 0, 255));
        SetVirtualColumnValue(prevPosition, 255, 0, 255);
      }
      zoneMatrix.setPixelColor(i, zoneMatrix.Color(255, 255, 255));
      SetVirtualColumnValue(i, 255, 255, 255);
      zoneMatrix.show();
      prevPosition = i;
    }
    if (digitalRead(botIndicatorPins[i]) && prevPosition == columnCount - 1 && i == columnCount - 1) {
      zoneMatrix.setPixelColor(prevPosition, zoneMatrix.Color(255, 0, 255));
      SetVirtualColumnValue(prevPosition, 255, 0, 255);
      zoneMatrix.show();
      prevPosition = -1;
    } 
  }

  // --------------------------------------------------------------------------------------------------------------------------------------------------------
}

// Configuration function to read non-volatile memory of EEPROM
void BoardConfig() {
  preferences.begin("board-config", false);
  zoneType = preferences.getString("ZoneType", "regular");      // Default to "regular" if not set
  currentZoneID = preferences.getInt("CurrentZoneID", -1);      // Default to -1 if not set
  columnCount = preferences.getInt("ColumnCount", 0);           // Default to 0 if not set

  if (currentZoneID == -1) {
    Serial.println("Current Zone ID is not set!");
    while (true);
  }

  Serial.println("Board Type: " + zoneType + ", Column Count: " + String(columnCount) + ", Current Zone ID: Z" + String(currentZoneID));
  
  // Update strip with the correct columnCount
  zoneMatrix.updateLength(columnCount);
  zoneMatrix.begin();
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

// Function for OTA firmare update success
void UpdateSuccess() {
  for (int i = 0; i < NODE_COUNT; i++) {
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 255, 0));
    zoneMatrix.show();
    delay(500);
  }
  for (int i = 0; i < NODE_COUNT; i++) {  // Turn off all LED if update has succeeded
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 0, 0));
    zoneMatrix.show();
  }
}

// Function for OTA firmare update failure
void UpdateFailed() {
  for (int i = 0; i < NODE_COUNT; i++) {
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 255, 0));
    zoneMatrix.show();
  }
}

// Function to take firmware OTA if boot button is pressed.
void CheckForOTAUpdate() {
  for (int i = 0; i < NODE_COUNT; i++) {  // Turn on all LED so that update has started
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 255, 0));
    zoneMatrix.show();
  }
  
  HTTPClient http;
  String firmwareUrl = String(OTA_URL);
  Serial.println("Starting OTA update...");
  for (int i = 0; i < NODE_COUNT; i++) {  // Turn OFF all LED so that Update has started
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 0, 0));
    zoneMatrix.show();
  }
  http.begin(firmwareUrl);

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    for (int i = 0; i < NODE_COUNT; i++) {
      zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 255, 0));
      zoneMatrix.show();
      delay(500);
    }
    int contentLength = http.getSize();
    WiFiClient* stream = http.getStreamPtr();
    if (Update.begin(contentLength)) {
      size_t written = Update.writeStream(*stream);
      if (written == contentLength && Update.end()) {
        if (Update.isFinished()) {
          Serial.println("OTA update complete!");
          UpdateSuccess();
          ESP.restart();
        } else {
          Serial.println("OTA update failed!");
          UpdateFailed();
        }
      } else {
        Serial.println("Failed to write the firmware!");
        UpdateFailed();
      }
    } else {
      Serial.println("Not enough space for OTA update!");
      UpdateFailed();
    }
  } else {
    Serial.println("Failed to fetch firmware: HTTP error");
    UpdateFailed();
  }
  http.end();
}

// Configuration function for LED strip
void NeoPixelConfig() {
  zoneMatrix.begin();             // INITIALIZE NeoPixel strip object (REQUIRED)
  zoneMatrix.show();              // Turn OFF all pixels ASAP
  zoneMatrix.setBrightness(255);  // Set BRIGHTNESS to about 100% (max = 255)
}