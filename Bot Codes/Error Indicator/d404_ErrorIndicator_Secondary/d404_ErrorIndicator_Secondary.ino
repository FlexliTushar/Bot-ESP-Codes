#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>

// ================= LCD Configuration =================
#define LCD_ADDRESS 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

#define SDA_PIN 8
#define SCL_PIN 9

// ================= UART Configuration =================
#define UART_BAUD 115200
#define RX_PIN 11
#define TX_PIN 12

// ================= NeoPixel Configuration =================
#define NEO_PIN 2         // GPIO pin for NeoPixel DIN
#define NEO_COUNT 52       // Number of LEDs in your strip
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

// ================= Objects =================
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
String receivedCommand = "";

// ================= Setup =================
void setup() {
  Serial.begin(115200);

  // LCD init
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD Ready");

  // UART init
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // NeoPixel init
  strip.begin();
  strip.show(); // Initialize all to 'off'

  Serial.println("Secondary ESP32 initialized");
}

// ================= Loop =================
void loop() {
  if (Serial1.available()) {
    receivedCommand = Serial1.readStringUntil('\n');
    receivedCommand.trim();

    if (receivedCommand.length() > 0) {
      processCommand(receivedCommand);
      Serial.println("Processed: " + receivedCommand);
    }
  }
}

// ================= Helpers =================
void setNeoColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEO_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void showOnLCD(String msg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
}

// Unified response handler
void sendResponseToIndicators(String response) {
  if (response.startsWith("ERROR:")) {
    String msg = response.substring(6);
    setNeoColor(255, 0, 0);     // Red
    showOnLCD("ERROR: " + msg);
    Serial.println("ERROR: " + msg);
  }
  else if (response.startsWith("SUCCESS:")) {
    String msg = response.substring(8);

    if (msg.startsWith("FULL-")) {
      String disp = msg.substring(5);
      setNeoColor(0, 255, 0);   // Green
      showOnLCD("FULL-" + disp);
      Serial.println("SUCCESS: FULL-" + disp);
    } else {
      setNeoColor(0, 0, 255);   // Blue
      showOnLCD(msg);
      Serial.println("SUCCESS: " + msg);
    }
  }
  else if (response.startsWith("WARNING:")) {
    String msg = response.substring(8);
    setNeoColor(255, 255, 0);   // Yellow
    showOnLCD("WARNING: " + msg);
    Serial.println("WARNING: " + msg);
  }
  else if (response == "CLEAR") {
    lcd.clear();
    Serial.println("CLEARED");
  }
  else {
    // Fallback
    Serial.println(response);
  }
}

// ================= Command Processor =================
void processCommand(String command) {
  if (command == "INIT") {
    initializeLCD();
  }
  else if (command == "CLEAR") {
    sendResponseToIndicators("CLEAR");
  }
  else {
    sendResponseToIndicators(command);
  }
}

// ================= Helpers =================
void initializeLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(500);
  lcd.clear();
}

void clearLine(int line) {
  lcd.setCursor(0, line);
  for (int i = 0; i < LCD_COLS; i++) lcd.print(" ");
  lcd.setCursor(0, line);
}
