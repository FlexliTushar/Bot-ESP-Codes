#include <Preferences.h>

// Constant variables
const String CODE_ID = "Bot_config";

// Bot Variables
Preferences preferences;
String BOT_ID = "B10";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Code ID: " + CODE_ID);
  Serial.println(__FILE__);
  preferences.begin("diverter", false);
  delay(100);
  preferences.putString("BotId", BOT_ID);
  delay(100);
  preferences.end();
}

void loop() {
  preferences.begin("diverter", false);
  Serial.println("Bot ID: " + preferences.getString("BotId", "Bx"));
}