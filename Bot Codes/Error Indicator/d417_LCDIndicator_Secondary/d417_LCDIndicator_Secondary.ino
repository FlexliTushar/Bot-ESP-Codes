#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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

// ================= Objects =================
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
String receivedCommand = "";

// ================== Global Variables =================
const String CODE_ID = "d417";

String entries[5];       // store up to 5 entries
int entryCount = 0;      // total entries stored (max 5)
int nextIndex = 0;       // index to write next entry
unsigned long lastUpdate = 0; // track 1s cycle
int displayIndex = 0;    // index to display next on LCD

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

  Serial.println("CODE ID: " + String(CODE_ID));
  Serial.println(__FILE__);
  Serial.println("Secondary ESP32 initialized");
}

// ================= Loop =================
void loop() {
  // Receive new command
  if (Serial1.available()) {
    receivedCommand = Serial1.readStringUntil('\n');
    receivedCommand.trim();

    if (receivedCommand.length() > 0) {
      addEntry(receivedCommand);
      Serial.println("Added entry: " + receivedCommand);
      displayIndex = 0; // start showing from newest
      lastUpdate = millis(); // reset timer to show immediately
    }
  }

  // Display entries in rolling fashion every 1 second
  if (entryCount > 0 && millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    showOnLCD(displayIndex);
    displayIndex = (displayIndex + 1) % entryCount;
  }
}

// ================= Helpers =================
void addEntry(String cmd) {
  // Shift existing entries down by 1
  for (int i = min(entryCount, 4); i > 0; i--) {
    entries[i] = entries[i - 1];
  }

  // Add new command at index 0
  entries[0] = cmd;

  // Update entry count (max 5)
  if (entryCount < 5) entryCount++;
}

// Display entry with numbering
void showOnLCD(int index) {
  lcd.clear();

  String msg = entries[index];

  // Split at first space
  int spacePos = msg.indexOf('E');
  String line1, line2;

  if (spacePos != -1) {
    line1 = msg.substring(0, spacePos - 1);           // first part (e.g., "S: 2")
    line2 = "   " + msg.substring(spacePos);         // rest (e.g., "EP: 16")
  } else {
    line1 = msg;                                 // entire string if no space
    line2 = "";
  }

  // Add numbering to line1
  int displayNumber = index + 1;
  String numberedLine1 = String(displayNumber) + ". " + line1;

  // Ensure both lines fit LCD width
  numberedLine1 = numberedLine1.substring(0, LCD_COLS);
  line2 = line2.substring(0, LCD_COLS);

  // Print to LCD
  lcd.setCursor(0, 0);
  lcd.print(numberedLine1);
  lcd.setCursor(0, 1);
  lcd.print(line2);

  Serial.println("Display:\n" + numberedLine1 + "\n" + line2);
}