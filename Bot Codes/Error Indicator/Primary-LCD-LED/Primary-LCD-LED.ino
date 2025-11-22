/*
 * Primary ESP32 - UART Sender
 * Sends display commands to Secondary ESP32 via UART
 * Uses Serial1 for LCD communication (Serial2 reserved for servo)
 * 
 * Note: Add your ServoConfig() function to setup()
 */

// UART Configuration for LCD communication
#define UART_BAUD 115200
#define RX_PIN 16  // Connect to TX of secondary ESP32
#define TX_PIN 17  // Connect to RX of secondary ESP32

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
 
  // Initialize UART communication with secondary ESP32 (using Serial1)
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("Primary ESP32 initialized");
  delay(1000);
  
  // Initialize LCD on secondary ESP32
  sendCommand("S: 0 EP: 16");
  delay(100);
}

void loop() {
  // displayErrorCode("ERROR: 42");
  // delay(2000);
  
  // // Clear display
  // clearDisplay();
  // delay(1000);
  
  // Display on specific line and position
  // displayAt(Line, Position, "Message");
  
  // displayAt(0, 0, "ERROR: 42 Tadaaa");
  // delay(2000);
  // displayAt(1, 0, "-x-x-x-x-x-x-x-x");
  // delay(500);
  // clearDisplay();
  
  // unsigned long startTime = micros();
  // // displayAt(1, 0, "x-x-x-x-x-x-x-x-");
  // sendCommand("NEO:FILL:255,0,0");  // Set all LEDs red
  // unsigned long endTime  = micros();
  
  // sendCommand("NEO:SHOW");          // Apply
  // delay(1000);

  // sendCommand("NEO:CLEAR");         // Turn off all LEDs
  // sendCommand("NEO:SHOW");
  // delay(1000);

  // sendCommand("NEO:FILL:0,0,255"); // Set LED 3 to blue
  // sendCommand("NEO:SHOW");
  // delay(500);

  // unsigned long duration = endTime - startTime;

  // Serial.print("Took ");
  // Serial.print(duration);
  // Serial.println(" microseconds");
}

// Fast error code display function - minimal overhead
void displayErrorCode(String errorCode) {
  sendCommand("PRINT:" + errorCode);
}

// Clear display
void clearDisplay() {
  sendCommand("CLEAR");
}

// Display text at specific position
void displayAt(int line, int position, String text) {
  sendCommand("AT:" + String(line) + "," + String(position) + ":" + text);
}

// Set cursor position
void setCursor(int line, int position) {
  sendCommand("CURSOR:" + String(line) + "," + String(position));
}

// Turn display backlight on/off
void setBacklight(bool state) {
  sendCommand(state ? "BACKLIGHT:ON" : "BACKLIGHT:OFF");
}

// Send command to secondary ESP32
void sendCommand(String command) {
  Serial1.println(command);
  // Serial.println("Sent: " + command); // Debug output
}
