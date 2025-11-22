#include <Adafruit_MCP23X17.h>

// GPIO Pin Assignments
#define EMG_PIN 12
#define PANEL_LED_BUTTON 7 
#define AFC_BOTTOM_LSW_PIN 3
#define AFC_TOP_LSW_PIN 1
#define AFC_PIN_1 19
#define AFC_PIN_2 18

// Bot Variables
Adafruit_MCP23X17 mcp;
bool openFlap = false;
bool closeFlap = false;
bool stageFlap = false;
bool stopWhileStaging = false;
bool stopWhileClosing = false;
bool prevAckButtonState = false;

void StopFlap() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, HIGH);
}

void CloseFlap() {
  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, HIGH);
  stopWhileClosing = true;
}

void OpenFlap() {
  digitalWrite(EMG_PIN, LOW);
}

void StageFlap() {
  digitalWrite(AFC_PIN_1, HIGH);
  digitalWrite(AFC_PIN_2, LOW);
  stopWhileStaging = true;
}

TaskHandle_t limSwitchTask;

// Logger Function which will run as RTOS task
void LIM_SWITCH_TASK(void* pvParameters) {
  while (true) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    Serial.println(mcp.digitalRead(AFC_TOP_LSW_PIN));
    if (stopWhileClosing && !mcp.digitalRead(AFC_TOP_LSW_PIN)) {
      Serial.println("Stop Closing");
      StopFlap();
      stopWhileClosing = false;
    }
    if (stopWhileStaging && !mcp.digitalRead(AFC_BOTTOM_LSW_PIN))  {
      Serial.println("Stop Staging");
      StopFlap();
      stopWhileStaging = false;
    }
  }
}

void setup() {
  MCPConfig();
  PinConfig();
  // xTaskCreatePinnedToCore(
  //   LIM_SWITCH_TASK,
  //   "lim-switch_task",
  //   8192,
  //   NULL,
  //   1,
  //   &limSwitchTask,
  //   0);
  delay(1000);
  Serial.begin(115200);
  closeFlap = true;
}

void loop() {
  // bool currentAckButtonState = !mcp.digitalRead(PANEL_LED_BUTTON);
  if (!mcp.digitalRead(PANEL_LED_BUTTON)) {
    if (openFlap) {
      Serial.println("Open Flap");
      OpenFlap();
      openFlap = false;
      closeFlap = true;
    } else if (closeFlap) {
      Serial.println("Close Flap");
      CloseFlap();
      closeFlap = false;
      stageFlap = true;
    } else if (stageFlap) {
      Serial.println("Stage Flap");
      StageFlap();
      stageFlap = false;
      openFlap = true;
    }
    delay(1000);
  }
  if (stopWhileClosing && !mcp.digitalRead(AFC_TOP_LSW_PIN)) {
    Serial.println("Stop Closing");
    StopFlap();
    stopWhileClosing = false;
  }
  if (stopWhileStaging && !mcp.digitalRead(AFC_BOTTOM_LSW_PIN))  {
    Serial.println("Stop Staging");
    StopFlap();
    stopWhileStaging = false;
  }
  // prevAckButtonState = currentAckButtonState;
}

// Setup the MCP.
void MCPConfig() {
  if (!mcp.begin_I2C()) Serial.println("Error.");
}

// Setup the pin configuration
void PinConfig() {
  pinMode(EMG_PIN, OUTPUT);
  mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
  mcp.pinMode(AFC_TOP_LSW_PIN, INPUT_PULLUP);
  mcp.pinMode(AFC_BOTTOM_LSW_PIN, INPUT_PULLUP);

  // GPIO command outputs to Secondary
  pinMode(AFC_PIN_1, OUTPUT);
  pinMode(AFC_PIN_2, OUTPUT);

  digitalWrite(EMG_PIN, HIGH);
  digitalWrite(AFC_PIN_1, LOW);
  digitalWrite(AFC_PIN_2, LOW);
}