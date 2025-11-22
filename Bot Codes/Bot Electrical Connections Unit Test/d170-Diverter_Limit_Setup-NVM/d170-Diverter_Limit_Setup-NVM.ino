// -------------------- MCP Related Start
#include <Adafruit_MCP23X17.h>
#include <Preferences.h>
#include <SMS.h>

Adafruit_MCP23X17 mcp;
Preferences prefs;

#define PANEL_LED_PIN 8
#define PANEL_LED_BUTTON 7

// -------------------- MCP Related End

// -------------------- Servo Related Start
#define FRONT_SERVO 2
#define REAR_SERVO 3
#define ENABLE 1
#define DISABLE 0
#define SERVO_UART_DELAY 50

int front_diverter_left_limit = 0;
int front_diverter_right_limit = 0;
int front_diverter_tolerance = 0;
int front_diverter_left_thresold = 0;
int front_diverter_right_thresold = 0;

int rear_diverter_left_limit = 0;
int rear_diverter_right_limit = 0;
int rear_diverter_tolerance = 0;
int rear_diverter_left_thresold = 0;
int rear_diverter_right_thresold = 0;

int current_position_front_servo = 0;
int current_position_rear_servo = 0;

SMS sm;

// -------------------- Servo Related End

void setup() {
    Serial.begin(115200);
    mcp_config();
    pinConfig();
    servo_setup();

    Serial.println("Select an option:");
    Serial.println("1. Do servo position configuration.");
    Serial.println("2. Load saved diverter configuration.");
    Serial.print("Enter choice: ");

    while (!Serial.available()); // Wait for user input
    char choice = Serial.read();

    if (choice == '1') {
        servo_config();
    } else {
        loadDiverterConfig();
    }

    printDiverterConfig(); // Print the loaded/saved values

    div_left();
    delay(1000);
    div_right();
    delay(1000);
}

void loop() {
}

void saveDiverterConfig() {
    prefs.begin("diverter", false);
    prefs.putInt("front_left_lim", front_diverter_left_limit);
    prefs.putInt("front_right_lim", front_diverter_right_limit);
    prefs.putInt("front_tol", front_diverter_tolerance);
    prefs.putInt("front_left_thr", front_diverter_left_thresold);
    prefs.putInt("front_right_thr", front_diverter_right_thresold);
    
    prefs.putInt("rear_left_lim", rear_diverter_left_limit);
    prefs.putInt("rear_right_lim", rear_diverter_right_limit);
    prefs.putInt("rear_tol", rear_diverter_tolerance);
    prefs.putInt("rear_left_thr", rear_diverter_left_thresold);
    prefs.putInt("rear_right_thr", rear_diverter_right_thresold);
    prefs.end();
    Serial.println("Diverter config saved to NVM.");
}

// Function to load values from NVM
void loadDiverterConfig() {
    prefs.begin("diverter", true);
    front_diverter_left_limit = prefs.getInt("front_left_lim", 0);
    front_diverter_right_limit = prefs.getInt("front_right_lim", 0);
    front_diverter_tolerance = prefs.getInt("front_tol", 106);
    front_diverter_left_thresold = prefs.getInt("front_left_thr", 0);
    front_diverter_right_thresold = prefs.getInt("front_right_thr", 0);
    
    rear_diverter_left_limit = prefs.getInt("rear_left_lim", 0);
    rear_diverter_right_limit = prefs.getInt("rear_right_lim", 0);
    rear_diverter_tolerance = prefs.getInt("rear_tol", 0);
    rear_diverter_left_thresold = prefs.getInt("rear_left_thr", 0);
    rear_diverter_right_thresold = prefs.getInt("rear_right_thr", 0);
    prefs.end();
    Serial.println("Diverter config loaded from NVM.");
}

// Function to print the stored values
void printDiverterConfig() {
    prefs.begin("diverter", true);
    
    Serial.println("\n--- Stored Diverter Config ---");
    Serial.print("Front Diverter Left Limit: "); Serial.println(prefs.getInt("front_left_lim", 0));
    Serial.print("Front Diverter Right Limit: "); Serial.println(prefs.getInt("front_right_lim", 0));
    Serial.print("Front Diverter Tolerance: "); Serial.println(prefs.getInt("front_tol", 0));
    Serial.print("Front Diverter Left Threshold: "); Serial.println(prefs.getInt("front_left_thr", 0));
    Serial.print("Front Diverter Right Threshold: "); Serial.println(prefs.getInt("front_right_thr", 0));

    Serial.print("Rear Diverter Left Limit: "); Serial.println(prefs.getInt("rear_left_lim", 0));
    Serial.print("Rear Diverter Right Limit: "); Serial.println(prefs.getInt("rear_right_lim", 0));
    Serial.print("Rear Diverter Tolerance: "); Serial.println(prefs.getInt("rear_tol", 0));
    Serial.print("Rear Diverter Left Threshold: "); Serial.println(prefs.getInt("rear_left_thr", 0));
    Serial.print("Rear Diverter Right Threshold: "); Serial.println(prefs.getInt("rear_right_thr", 0));

    Serial.println("------------------------------\n");
    
    prefs.end();
}

// Servo communication setup.
void servo_setup() {
  sm.begin(&Serial2, 115200);
  delay(50);
  sm.setTimeOut(100);
  delay(50);

  Serial2.begin(115200, SERIAL_8N1, 25, 32);
}

// Servo position Configuration and Calibration
void servo_config() {

    sm.EnableTorque(FRONT_SERVO, DISABLE);
    delay(50);
    sm.EnableTorque(REAR_SERVO, DISABLE);
    delay(50);
     
    // loadDiverterConfig();  // Load stored values first

    front_diverter_left_limit = sm.ReadPos(FRONT_SERVO);
    delay(50);
    front_diverter_right_limit = sm.ReadPos(FRONT_SERVO);
    delay(50);

    rear_diverter_left_limit = sm.ReadPos(REAR_SERVO);
    delay(50);
    rear_diverter_right_limit = sm.ReadPos(REAR_SERVO);
    delay(50);

    while (mcp.digitalRead(PANEL_LED_BUTTON)) {
        Serial.println("Move both diverters to extreme ends 5-6 times and then Press ACK Button.");
        
        mcp.digitalWrite(PANEL_LED_PIN, HIGH);
        delay(200);
        mcp.digitalWrite(PANEL_LED_PIN, LOW);
        delay(200);

        current_position_front_servo = sm.ReadPos(FRONT_SERVO);
        delay(50);
        current_position_rear_servo = sm.ReadPos(REAR_SERVO);
        delay(50);

        if (current_position_front_servo < front_diverter_left_limit ) front_diverter_left_limit = current_position_front_servo; // front diverter left limit is lowest value
        if (current_position_front_servo > front_diverter_right_limit ) front_diverter_right_limit = current_position_front_servo; // front diverter right limit is higesh value

        if (current_position_rear_servo > rear_diverter_left_limit ) rear_diverter_left_limit = current_position_rear_servo; // diverter left limit is lowest value
        if (current_position_rear_servo < rear_diverter_right_limit ) rear_diverter_right_limit = current_position_rear_servo; // diverter right limit is higesh value
        
    }

    front_diverter_tolerance = (front_diverter_right_limit - front_diverter_left_limit)/10;  //5
    front_diverter_left_thresold = front_diverter_left_limit + front_diverter_tolerance;
    front_diverter_right_thresold = front_diverter_right_limit - front_diverter_tolerance;

    rear_diverter_tolerance = (rear_diverter_left_limit - rear_diverter_right_limit)/10;     //5
    rear_diverter_left_thresold = rear_diverter_left_limit - rear_diverter_tolerance;
    rear_diverter_right_thresold = rear_diverter_right_limit + rear_diverter_tolerance;

    Serial.println ("");
    Serial.print ("front diverter left limit: "); Serial.print (front_diverter_left_limit);Serial.print (" | ");
    Serial.print ("front diverter right limit: "); Serial.print (front_diverter_right_limit);Serial.print (" | ");
    Serial.print ("front diverter tolerance: "); Serial.print (front_diverter_tolerance);Serial.print(" | ");
    Serial.print ("front diverter left thresold: "); Serial.print (front_diverter_left_thresold);Serial.print (" | ");
    Serial.print ("front diverter right thresold: "); Serial.print (front_diverter_right_thresold);Serial.println(" | ");

    Serial.print ("rear diverter left limit: "); Serial.print (rear_diverter_left_limit);Serial.print (" | ");
    Serial.print ("rear diverter right limit: "); Serial.print (rear_diverter_right_limit);Serial.print (" | ");
    Serial.print ("rear diverter tolerance: "); Serial.print (rear_diverter_tolerance);Serial.print(" | ");
    Serial.print ("rear diverter left thresold: "); Serial.print (rear_diverter_left_thresold);Serial.print (" | ");
    Serial.print ("rear diverter right thresold: "); Serial.print (rear_diverter_right_thresold);Serial.println(" | ");

    saveDiverterConfig();  // Save values after calibration

    Serial.println("-----------------> Servo Configured");
}

// MCP and PIN Configuration
void mcp_config() {
    if (!mcp.begin_I2C()) {
        Serial.println("Error.");
        while (1);
    }
}

void pinConfig() {
    mcp.pinMode(PANEL_LED_PIN, OUTPUT);
    mcp.pinMode(PANEL_LED_BUTTON, INPUT_PULLUP);
    mcp.digitalWrite(PANEL_LED_PIN, LOW);
    Serial.println("-----------------> Pins configured");
}

// Move Diverter to Left Position
void div_left() {


  current_position_front_servo = sm.ReadPos(FRONT_SERVO);
  delay(50);
  current_position_rear_servo = sm.ReadPos(REAR_SERVO);
  delay(50);


  if (current_position_front_servo < front_diverter_left_thresold )
  {
    sm.EnableTorque(FRONT_SERVO, DISABLE);
    delay(50);
  }
  else
  {
    sm.WritePosEx(FRONT_SERVO, front_diverter_left_thresold, 100);
    // Serial.println("Diverter Left");
    delay(50);
  }



    if (current_position_rear_servo > rear_diverter_left_thresold )
  {
    sm.EnableTorque(REAR_SERVO, DISABLE);
    delay(50);
  }
  else
  {
    sm.WritePosEx(REAR_SERVO, rear_diverter_left_thresold, 100);
    delay(50);
    // Serial.println("Diverter Left");
  }


}

// Move Diverter to Right Position
void div_right() {


  current_position_front_servo = sm.ReadPos(FRONT_SERVO);
  delay(50);
  current_position_rear_servo = sm.ReadPos(REAR_SERVO);
  delay(50);


  if (current_position_front_servo > front_diverter_right_thresold )
  {
    sm.EnableTorque(FRONT_SERVO, DISABLE);
     delay(50);
  }
  else
  {
    sm.WritePosEx(FRONT_SERVO, front_diverter_right_thresold, 100);
     delay(50);
    // Serial.println("Diverter Left");
  }



    if (current_position_rear_servo < rear_diverter_right_thresold )
  {
    sm.EnableTorque(REAR_SERVO, DISABLE);
     delay(50);
  }
  else
  {
    sm.WritePosEx(REAR_SERVO, rear_diverter_right_thresold, 100);
     delay(50);
    // Serial.println("Diverter Left");
  }


}
