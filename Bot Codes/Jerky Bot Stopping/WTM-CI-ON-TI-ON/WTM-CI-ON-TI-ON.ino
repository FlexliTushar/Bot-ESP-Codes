#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <Update.h>
#include <CAN.h>
using namespace std;

#define CAN_SPEED 1000E3
#define NODE_COUNT 20

#define BOOT_BUTTON_PIN 0
#define ZONE_MATRIX_PIN 12
#define CAN_RX_PIN 21
#define CAN_TX_PIN 22
const int botIndicatorPins[20] = { 36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 13, 23, 2, 19, 18, 5, 17, 16, 4, 15 };

// Structs
struct RequestToYourZoneModel {
  uint8_t requestId = -1;
  uint8_t receiverId = 0;
  uint8_t originalSenderId = 0;
  uint8_t requestType = 0;
  String CommunicatorStatus = "PENDING";
  String CANCommunicatorStatus = "PENDING";
  float data1 = 0.0;
  float data2 = 0.0;
  float data3 = 0.0;
  float data4 = 0.0;
  float data5 = 0.0;
};

struct RequestFromYourZoneModel {
  uint8_t requestId = -1;
  uint8_t senderId = 0;
  uint8_t originalSenderId = 0;
  uint8_t requestType = 0;
  String CommunicatorStatus = "PENDING";
  String CANCommunicatorStatus = "PENDING";
  float data1 = 0.0;
  float data2 = 0.0;
  float data3 = 0.0;
  float data4 = 0.0;
  float data5 = 0.0;
};

const unsigned long THRESHOLD_TIME_FOR_HIGH_AT_LOWEST_POSSIBLE_SPEED = 400;
const unsigned long THRESHOLD_TIME_FOR_LOW_AT_HIGHEST_POSSIBLE_SPEED = 1000;
const unsigned long THRESHOLD_TIME_TO_CLEAR_BOT_FROM_MY_ZONE = 500;
const String CODE_ID = "d383-WTM-OptimisedRetry";
const float INTER_COLUMN_PITCH = 0.04;
const float ACCELERATION = 1.87;
const unsigned long THRESHOLD_TIME_OF_WAIT_FOR_ACK = 10;
const int THRESHOLD_RETRIES = 3;

// WiFi credentials
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";

// Server URL
const char* serverUrl = "http://192.168.2.109:5050/upload";
const String OTA_URL = "http://192.168.2.109:5000/download_WTM";

// Monitoring variables
const unsigned long maxLoopTime = 1000;
unsigned long loopStartTime;
size_t maxMemoryUsed = 0;

// Sampling parameters
int zeroCount[20] = { 0 };

// Logger variables
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;

// Zone Variables
Preferences preferences;
Adafruit_NeoPixel zoneMatrix(20, ZONE_MATRIX_PIN, NEO_RGB + NEO_KHZ800);
bool virtualZoneMatrix[3][100] = { 0 };
uint8_t currentZoneID;
int columnCount;
String zoneType;
int myZoneBotPosition = -1;
bool highDetectedStatus[20] = { false };
unsigned long lowStartTime[20] = { 0 };
unsigned long highStartTime[20] = { 0 };
bool botIsPresent[20] = { false };
bool botDetectedFirstTime = false;
bool expectedStatus[20] = { true };
bool highFrequencyNoiseNotReported[20] = { true };
bool lowFrequencyNoiseNotReported[20] = { true };
int expectedSafetyRange = 5;
float minimumSafetyGap = 3;
int myZoneTailLength = 0;
float myZoneBotSpeed = 0;
float yourZoneBotSpeed = 0;
int myZoneTailActiveLength = 0;
int yourZoneTailLength = 0;
int yourZoneBotMyZoneTailActiveLength_A = 0;
int yourZoneBotMyZoneTailActiveLength_B = 0;
int previousYourZoneTailLength = 0;
int myZoneMirrorBotPosition = -1;
int myZoneMirrorTailActiveLength = 0;
RequestToYourZoneModel requestToYourZoneTable[20];
RequestFromYourZoneModel requestFromYourZoneTable[20];
int requestFromTableIndex = 0;
int requestToTableIndex = 0;
int requestId = 0;
uint8_t previousZoneID_A;
uint8_t previousZoneID_B;
uint8_t nextZoneID_A;
uint8_t nextZoneID_B;
SemaphoreHandle_t xMyZoneBotPosition;
int myZoneBotPositionForCore0 = -1;
SemaphoreHandle_t xBotClearingFlag;
bool clearTheBot = false;
volatile bool otaFlag = false;
TaskHandle_t tailAndCommTask;
SemaphoreHandle_t xUpdateExpectedStatusFlag;
bool needToUpdateExpectedStatus = false;
int valueForUpdateExpectedStatus = -1;
TaskHandle_t recieveRequestTask;
portMUX_TYPE requestTableMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t slaveZoneID;
SemaphoreHandle_t xZeroCount;
int maxExpectedRangeSent = 0;
SemaphoreHandle_t xintimateLoopTimeIncrease;
bool intimateLoopTimeIncrease = false;
SemaphoreHandle_t xOTAFlag;
unsigned long requestToYourZoneTimeArray[20];
int retriesCounterArray[20] = { 0 };
int botDetectedFirstTimeRequestId = -1;
int yourZoneExpectedValue = 0;

// Interrupt service routine for boot button press
void IRAM_ATTR ButtonISR() {
   if (xSemaphoreTake(xOTAFlag, portMAX_DELAY) == pdTRUE) {
    otaFlag = true;
    xSemaphoreGive(xOTAFlag);
  }
}

// Function for filter based bot detection - High frequency noise, low frequency noise and expected status based filters
void ReadTransition(int index) {
  int sensorValue = digitalRead(botIndicatorPins[index]);
  // Case: If detected the bot on indicator pin and high status is false then capture the start of high-time and toggle high status to true
  if (!sensorValue && !highDetectedStatus[index]) {
    highStartTime[index] = millis();
    highDetectedStatus[index] = true;
    if (xSemaphoreTake(xZeroCount, portMAX_DELAY) == pdTRUE) {
      zeroCount[index] = 1;
      xSemaphoreGive(xZeroCount);
    }
  }

  if (!sensorValue && highDetectedStatus[index] && expectedStatus[index]) {
    if (xSemaphoreTake(xZeroCount, portMAX_DELAY) == pdTRUE) {
      zeroCount[index]++;
      xSemaphoreGive(xZeroCount);
    }
  }

  // Case: If high status is true then check whether there is a potential for low frequency noise
  if (highDetectedStatus[index]) {
    // Case: If bot is expected on specific column then potentially either bot has stopped over there or there is a low frequency noise just ahead of the bot else only low frequency noise
    if (expectedStatus[index]) {
      // Case: If high time is greater than threshold then there is certainly either bot has stopped over the column or low frequency noise just ahead of the bot and need to toggle the high status to false
      if (millis() - highStartTime[index] > THRESHOLD_TIME_FOR_HIGH_AT_LOWEST_POSSIBLE_SPEED && lowFrequencyNoiseNotReported[index]) {
        highDetectedStatus[index] = false;
      }
    } else {
      // Case: If high time is greater than threshold then there is certainly low frequency noise and need to toggle the high status to false
      if (millis() - highStartTime[index] > THRESHOLD_TIME_FOR_HIGH_AT_LOWEST_POSSIBLE_SPEED && lowFrequencyNoiseNotReported[index]) {
        highDetectedStatus[index] = false;
      }
    }
  }

  // Case: If not detecting anything on indicator pina and high status is true then potentially bot present or some error and need to capture the start of the low time and toggle high status to false
  if (sensorValue && highDetectedStatus[index]) {
    // Case: If low time is greater than the threshold, high time smaller than the threshold and expected status is true then bot should be present
    // [Note]: Not robust against bot mimicking signal that might be appearing just ahead of the bot
    if (highStartTime[index] - lowStartTime[index] > THRESHOLD_TIME_FOR_LOW_AT_HIGHEST_POSSIBLE_SPEED && millis() - highStartTime[index] < THRESHOLD_TIME_FOR_HIGH_AT_LOWEST_POSSIBLE_SPEED) {
      if (expectedStatus[index]) {
        botIsPresent[index] = true;
      }
    }
    lowStartTime[index] = millis();
    highDetectedStatus[index] = false;
  }
}

// Function to update the expected status of the system based on the MyZoneBotPosition
void UpdateExpectedStatus(int index) {
  for (int i = 0; i <= index; i++) {
    expectedStatus[i] = false;
  }
  // Case: If expected safety range columns are exhausted in the current zone then transfer the remaining column's expected status to the next zone
  if (columnCount - index - 1 < expectedSafetyRange) {
    for (int i = index + 1; i < columnCount; i++) {
      expectedStatus[i] = true;
    }
  } else {
    for (int i = index + 1; i < index + 1 + expectedSafetyRange; i++) {
      expectedStatus[i] = true;
    }
  }
}

// Function responsible for the detection of the bot, updating the MyZoneBotSpeed and MyZoneBotPosition
bool BotDetection() {
  // Iterating over all the bot detection pins
  for (int i = 0; i < columnCount; i++) {
    ReadTransition(i);
    // Case: If condition true then potentially a bot is detected else not
    if (botIsPresent[i]) {
      // add_log("Bot detected @ " + String(i));
      // Case: If condition true then bot detected at new column
      if (myZoneBotPosition != i) {
        unsigned long currentTime = millis();
        // Case: If condition true then turn OFF the bot LED at previous MyZoneBotPosition
        if (myZoneBotPosition != -1) {
          botIsPresent[myZoneBotPosition] = false;
        }

        // Update MyZoneBotPosition and turn ON the bot LED at new MyZoneBotPosition
        myZoneBotPosition = i;
        UpdateExpectedStatus(i);
        return true;
      }
    }
  }

  if (xSemaphoreTake(xBotClearingFlag, portMAX_DELAY) == pdTRUE) {
    if (clearTheBot) {
      // add_log("Bot removed!");
      botIsPresent[myZoneBotPosition] = false;
      myZoneBotPosition = -1;
      for (int i = 0; i < columnCount; i++) {
        botIsPresent[i] = false;
        if (xSemaphoreTake(xZeroCount, portMAX_DELAY) == pdTRUE) {
          zeroCount[i] = 0;
          xSemaphoreGive(xZeroCount);
        }
        expectedStatus[i] = false;
        highFrequencyNoiseNotReported[i] = true;
        lowFrequencyNoiseNotReported[i] = true;
      }
      clearTheBot = false;
    }
    xSemaphoreGive(xBotClearingFlag);
  }
  return false;
}

// Set the values of LED column
// (r -> Bot Indicator, g -> Traffic Indicator, b -> Column Indicator)
void SetVirtualColumnValue(int i, uint8_t r, uint8_t g, uint8_t b) {
  virtualZoneMatrix[0][i] = (255 == r);
  virtualZoneMatrix[1][i] = (255 == g);
  virtualZoneMatrix[2][i] = (255 == b);
}

// Generates length of tail because of bot in MyZone.
int TailRegulator(int myZoneBotPosition, float myZoneBotSpeed, float yourZoneBotSpeed) {
  // Case handling, if bot is not present in the zone then return 0 length
  if (myZoneBotPosition == -1) {
    return 0;
  }

  float relativeSpeed = myZoneBotSpeed - yourZoneBotSpeed;
  float relativeDistance = (relativeSpeed * relativeSpeed) / (2 * ACCELERATION);
  float dynamicSafetyGap = minimumSafetyGap;

  // Case handling, if the bot behind is moving faster than extend the tail
  if (relativeSpeed < 0) {
    dynamicSafetyGap += relativeDistance;
  }

  int tailLength = ceil(minimumSafetyGap / INTER_COLUMN_PITCH);
  return tailLength;
}

// Generates length of active tail in MyZone because of bot in MyZone and length of tail for the zone behind because of bot in MyZone and incoming tails from ahead zones
pair<int, int> TailSegmentor(int myZoneTailLength, int myZoneBotPosition, int yourZoneBotMyZoneTailActiveLength_A, int yourZoneBotMyZoneTailActiveLength_B) {
  pair<int, int> myPair = { 0, 0 };
  int yourZoneTailLength = 0;
  int myZoneTailActiveLength = 0;
  if (myZoneBotPosition != -1) {
    yourZoneTailLength = max(myZoneTailLength - myZoneBotPosition, yourZoneTailLength);
    myZoneTailActiveLength = min(myZoneBotPosition, myZoneTailLength);
  }

  if (zoneType == "diverge") {
    yourZoneBotMyZoneTailActiveLength_A = max(yourZoneBotMyZoneTailActiveLength_A, yourZoneBotMyZoneTailActiveLength_B);
  }

  int remainingYourZoneBotTailLength = yourZoneBotMyZoneTailActiveLength_A - columnCount;

  // If remainingYourZoneBotTailLength is a positive number only then it means that the tail from YourZone is exceeding even MyZone
  if (remainingYourZoneBotTailLength > 0) {
    yourZoneTailLength = max(yourZoneTailLength, remainingYourZoneBotTailLength);
  }

  myPair.first = myZoneTailActiveLength;
  myPair.second = yourZoneTailLength;
  return myPair;
}

// Turns ON/OFF the relevant amount of traffic indicators
void TailSuperimposer(int myZoneBotPosition, int myZoneTailActiveLength, int myZoneMirrorBotPosition, int myZoneMirrorTailActiveLength, int yourZoneBotMyZoneTailActiveLength_A, int yourZoneBotMyZoneTailActiveLength_B) {
  // 'superimposedActiveZone' is the union of all the input active tail lengths and generates a snapshot of what pattern traffic indicators must turn ON and OFF in MyZone
  bool superimposedActiveZone[columnCount] = { 0 };

  if (myZoneBotPosition != -1) {
    for (int i = myZoneBotPosition - 1; i > myZoneBotPosition - 1 - myZoneTailActiveLength; i--) {
      superimposedActiveZone[i] = 1;
    }
  }

  if (zoneType == "mergeLane") {
    if (myZoneMirrorBotPosition != -1) {
      for (int i = myZoneMirrorBotPosition - 1; i > myZoneMirrorBotPosition - 1 - myZoneMirrorTailActiveLength; i--) {
        superimposedActiveZone[i] = 1;
      }
    }
  }

  if (zoneType == "diverge") {
    yourZoneBotMyZoneTailActiveLength_A = max(yourZoneBotMyZoneTailActiveLength_A, yourZoneBotMyZoneTailActiveLength_B);
  }

  if (yourZoneBotMyZoneTailActiveLength_A > columnCount) {
    for (int i = 0; i < columnCount; i++) {
      superimposedActiveZone[i] = 1;
    }
  } else {
    for (int i = columnCount - 1; i > columnCount - 1 - yourZoneBotMyZoneTailActiveLength_A; i--) {
      superimposedActiveZone[i] = 1;
    }
  }

  for (int i = 0; i < columnCount; i++) {
    if (superimposedActiveZone[i]) {
      if (!virtualZoneMatrix[0][i]) {
        SetVirtualColumnValue(i, 255, virtualZoneMatrix[1][i] * 255, virtualZoneMatrix[2][i] * 255);
        zoneMatrix.setPixelColor(i, zoneMatrix.Color(255, virtualZoneMatrix[1][i] * 255, virtualZoneMatrix[2][i] * 255));
      }
    } else {
      if (virtualZoneMatrix[0][i]) {
        SetVirtualColumnValue(i, 0, virtualZoneMatrix[1][i] * 255, virtualZoneMatrix[2][i] * 255);
        zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, virtualZoneMatrix[1][i] * 255, virtualZoneMatrix[2][i] * 255));
      }
    }
  }
  zoneMatrix.show();
}

// CANCommunicator: Sending CAN request
void SendRequest(RequestToYourZoneModel request) {
  // Begin the CAN frame
  CAN.beginPacket(currentZoneID);
  // Write relevant information - Consumes 4 bytes right now
  CAN.write(request.receiverId);
  CAN.write(request.originalSenderId);
  CAN.write(request.requestType);
  CAN.write(request.requestId);
  // Case: If condition true then write 1st part of payload as crude value else write mapped value (1 byte)
  if (request.requestType == 0x3 || request.requestType == 0x5 || request.requestType == 0x7 || request.requestType == 0x8 || request.requestType == 0xB || request.requestType == 0xC || request.requestType == 0xD || request.requestType == 0xE || request.requestType == 0xF || request.requestType == 0x11 || request.requestType == 0x12) {
    CAN.write((uint8_t)request.data1);
  } else if (request.requestType != 0x0 && request.requestType != 0x11) {
    CAN.write((uint8_t)round((request.data1 * 100.0 * 255.0) / 512.0));
  }
  // Case: If condition true then write 2nd and 3rd part of the payload (1 byte each - total 2 bytes)
  if (request.requestType == 0x1 || request.requestType == 0x2 || request.requestType == 0x5) {
    CAN.write((uint8_t)round((request.data2 * 100.0 * 255.0) / 512.0));
    CAN.write((uint8_t)round((request.data3 * 100.0 * 255.0) / 512.0));
  } else if (request.requestType == 0xB || request.requestType == 0xF || request.requestType == 0x11 || request.requestType == 0x12) {
    CAN.write((uint8_t)request.data2);
  }
  // Case: If condition true then write 4th part of the payload (1 byte)
  if (request.requestType == 0x2) {
    CAN.write((uint8_t)round((request.data4 * 100.0 * 255.0) / 512.0));
  }
  // End the CAN frame
  CAN.endPacket();
  // add_log("Sending request of type " + String(request.requestType) + " with data " + String(request.data1) + " and request ID " + String(request.requestId));
}

// Function to add an incoming request from YourZone in the RequestFromYourZoneTable
void AddRequestFromYourZoneTable(RequestFromYourZoneModel request) {
  if (requestFromTableIndex >= 20) {
    // add_log("OVERFLOW_ERROR: 'requestFromTableIndex' is out of range! Setting index back to '0'.");
    requestFromTableIndex = 0;
  }
  portENTER_CRITICAL(&requestTableMux);
  requestFromYourZoneTable[requestFromTableIndex] = request;
  portEXIT_CRITICAL(&requestTableMux);
  requestFromTableIndex++;
}

// CANCommunicator: Receive the income CAN request
void RecieveRequest() {
  unsigned long latency = micros();
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    RequestFromYourZoneModel recievedRequest;
    recievedRequest.senderId = CAN.packetId();
    if (!CAN.packetRtr()) {
      uint8_t receiverId = CAN.read();
      if (receiverId == currentZoneID) {
        while (CAN.available()) {
          recievedRequest.originalSenderId = CAN.read();
          recievedRequest.requestType = CAN.read();
          recievedRequest.requestId = CAN.read();
          if (recievedRequest.requestType == 0x3 || recievedRequest.requestType == 0x5 || recievedRequest.requestType == 0x7 || recievedRequest.requestType == 0x8) {
            recievedRequest.data1 = CAN.read();
          } else if (recievedRequest.requestType != 0x0 && recievedRequest.requestType != 0x15) {
            recievedRequest.data1 = ((float)CAN.read() * 512.0 / 255.0) / 100.0;
          }
          if (recievedRequest.requestType == 0x1 || recievedRequest.requestType == 0x2 || recievedRequest.requestType == 0x5) {
            recievedRequest.data2 = ((float)CAN.read() * 512.0 / 255.0) / 100.0;
            recievedRequest.data3 = ((float)CAN.read() * 512.0 / 255.0) / 100.0;
          }
          if (recievedRequest.requestType == 0x2) {
            recievedRequest.data4 = ((float)CAN.read() * 512.0 / 255.0) / 100.0;
          }

          recievedRequest.CANCommunicatorStatus = "COMPLETE";
          AddRequestFromYourZoneTable(recievedRequest);
          // add_log("Recieved request ID " + String(recievedRequest.requestId) + " of type " + String(recievedRequest.requestType) + " with data " + String(recievedRequest.data1));
        }
      }
    }
  }
}

// Function to add an outgoing request to YourZone in the RequestToYourZoneTable
void AddRequestToYourZoneTable(RequestToYourZoneModel request) {
  if (requestToTableIndex >= 20) {
    // add_log("OVERFLOW_ERROR: 'requestToTableIndex' is out of range! Setting index back to '0'.");
    requestToTableIndex = 0;
  }
  requestToYourZoneTable[requestToTableIndex] = request;
  requestToTableIndex++;
}

//Function to resolve the requests in RequestToYourZoneTable
void ResolveRequestToYourZoneTable() {
  for (int i = 0; i < 20; i++) {
    RequestToYourZoneModel request = requestToYourZoneTable[i];
    if (request.requestId == 255) continue;
    if (request.CommunicatorStatus == "COMPLETE" && request.CANCommunicatorStatus == "COMPLETE") {
      for (int j = i; j < 19; j++) {
        requestToYourZoneTable[j] = requestToYourZoneTable[j + 1];
        retriesCounterArray[j] = retriesCounterArray[j + 1];
        requestToYourZoneTimeArray[j] = requestToYourZoneTimeArray[j + 1];
      }
      requestToYourZoneTable[19] = RequestToYourZoneModel();
      retriesCounterArray[19] = 0;
      requestToYourZoneTimeArray[19] = 0;
      requestToTableIndex--;
      i--;
      continue;
    }

    if (request.CANCommunicatorStatus == "PENDING") {
      SendRequest(request);
      request.CANCommunicatorStatus = "COMPLETE";
      requestToYourZoneTable[i] = request;
      requestToYourZoneTimeArray[i] = millis();
      retriesCounterArray[i] = 1;
    }

    if (request.CANCommunicatorStatus == "COMPLETE" && request.CommunicatorStatus == "PENDING" && millis() - requestToYourZoneTimeArray[i] > THRESHOLD_TIME_OF_WAIT_FOR_ACK) {
      if ((request.requestType == 0x3 || request.requestType == 0x7) && yourZoneTailLength >= (int) request.data1) {
        // add_log("Need to resend the request of type " + String(request.requestType));
        RequestToYourZoneModel errorRequest = IntimateRetryCommunication(retriesCounterArray[i], request.requestType);
        AddRequestToYourZoneTable(errorRequest);
        if (retriesCounterArray[i] < THRESHOLD_RETRIES) {
          SendRequest(request);
          requestToYourZoneTimeArray[i] = millis();
          retriesCounterArray[i]++;
        }
      } else if (request.requestType == 0x8 && yourZoneExpectedValue <= (int) request.data1) {
        // add_log("Need to resend the request of type " + String(request.requestType));
        RequestToYourZoneModel errorRequest = IntimateRetryCommunication(retriesCounterArray[i], request.requestType);
        AddRequestToYourZoneTable(errorRequest);
        if (retriesCounterArray[i] < THRESHOLD_RETRIES) {
          SendRequest(request);
          requestToYourZoneTimeArray[i] = millis();
          retriesCounterArray[i]++;
        }
      } else {
        for (int j = i; j < 19; j++) {
          requestToYourZoneTable[j] = requestToYourZoneTable[j + 1];
          retriesCounterArray[j] = retriesCounterArray[j + 1];
          requestToYourZoneTimeArray[j] = requestToYourZoneTimeArray[j + 1];
        }
        requestToYourZoneTable[19] = RequestToYourZoneModel();
        retriesCounterArray[19] = 0;
        requestToYourZoneTimeArray[19] = 0;
        requestToTableIndex--;
        i--;
        continue;
      }
    }
  }
}

//Function to resolve the requests in RequestFromYourZoneTable
void ResolveRequestFromYourZoneTable() {
  for (int i = 0; i < 20; i++) {
    RequestFromYourZoneModel request = requestFromYourZoneTable[i];
    if (request.requestId == 255) continue;
    if (request.CommunicatorStatus == "COMPLETE" && request.CANCommunicatorStatus == "COMPLETE") {
      for (int j = i; j < 19; j++) {
        requestFromYourZoneTable[j] = requestFromYourZoneTable[j + 1];
      }
      requestFromYourZoneTable[19] = RequestFromYourZoneModel();
      requestFromTableIndex--;
      i--;
      continue;
    }

    if (request.CommunicatorStatus == "PENDING") {
      if (request.requestType == 0x0) {
        for (int j = 0; j < 20; j++) {
          RequestToYourZoneModel myZoneRequest = requestToYourZoneTable[i];
          if (myZoneRequest.requestId == request.requestId) {
            myZoneRequest.CommunicatorStatus = "COMPLETE";
            requestToYourZoneTable[i] = myZoneRequest;
            break;
          }
        }
        if (request.requestType == botDetectedFirstTimeRequestId) {
          HandleAcknowledgementForTailBotClearanceFromPreviousZone();
        }
      } else if (request.requestType == 0x1 || request.requestType == 0x2 || request.requestType == 0x5) {
        // Not using right now
      } else if (request.requestType == 0x3 || request.requestType == 0x7) {
        HandleTailTransfer(request);
      } else if (request.requestType == 0x8) {
        HandleExpectedBotRequest(request);
      } else if (request.requestType == 0x4) {
        // Not using right now
      } else if (request.requestType == 0x15) {
        HandleOTARequestViaBridge();
      }

      request.CommunicatorStatus = "COMPLETE";
      requestFromYourZoneTable[i] = request;
    }
  }
}

// Handle the incoming OTA request from the bridge
void HandleOTARequestViaBridge() {
  if (xSemaphoreTake(xOTAFlag, portMAX_DELAY) == pdTRUE) {
    otaFlag = true;
    xSemaphoreGive(xOTAFlag);
  }
}

// Fabricate tail transfer request
RequestToYourZoneModel IntimationTailTransfer(int yourZoneTailLength, uint8_t previousZoneID) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = previousZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0x3;
  // Case: If bot detected for the first time then send the tail transfer request of type 0x7 - Clear the bot from previous zone
  if (botDetectedFirstTime) {
    requestToYourZone.requestType = 0x7;
    botDetectedFirstTimeRequestId = requestToYourZone.requestId;
  }
  requestToYourZone.data1 = yourZoneTailLength;

  return requestToYourZone;
}

void HandleAcknowledgementForTailBotClearanceFromPreviousZone() {
  botDetectedFirstTime = false;
  botDetectedFirstTimeRequestId = -1;
}

// Handle the incoming tail transfer request
void HandleTailTransfer(RequestFromYourZoneModel request) {
  if (request.requestType == 0x7 && myZoneBotPositionForCore0 != -1) {
    if (xSemaphoreTake(xBotClearingFlag, portMAX_DELAY) == pdTRUE) {
      // add_log("Asked to remove the bot!");
      clearTheBot = true;
      xSemaphoreGive(xBotClearingFlag);
    }
    // sendDataToServer();
  }

  if (request.senderId == nextZoneID_A) {
    yourZoneBotMyZoneTailActiveLength_A = (int)request.data1;
  } else {
    yourZoneBotMyZoneTailActiveLength_B = (int)request.data1;
  }

  RequestToYourZoneModel ackRequest;
  ackRequest.requestId = request.requestId;
  ackRequest.receiverId = request.senderId;
  ackRequest.originalSenderId = currentZoneID;
  ackRequest.requestType = 0x0;
  ackRequest.CommunicatorStatus = "COMPLETE";

  AddRequestToYourZoneTable(ackRequest);
}

// Fabricate bot expected request
RequestToYourZoneModel IntimateExpectedBotRequest(int expectedPlaces) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = nextZoneID_A;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0x8;
  requestToYourZone.data1 = expectedPlaces;
  return requestToYourZone;
}

// Handle the incoming bot expected request
void HandleExpectedBotRequest(RequestFromYourZoneModel request) {
  int expected = (int)request.data1;

  if (xSemaphoreTake(xUpdateExpectedStatusFlag, portMAX_DELAY) == pdTRUE) {
    needToUpdateExpectedStatus = true;
    valueForUpdateExpectedStatus = expected;
    xSemaphoreGive(xUpdateExpectedStatusFlag);
  }

  RequestToYourZoneModel ackRequest;
  ackRequest.requestId = request.requestId;
  ackRequest.receiverId = request.senderId;
  ackRequest.originalSenderId = currentZoneID;
  ackRequest.requestType = 0x0;
  ackRequest.CommunicatorStatus = "COMPLETE";
  AddRequestToYourZoneTable(ackRequest);
}

// Fabricate skipped columns error request
RequestToYourZoneModel IntimateSkippedColumnError(int skippedColumns, int currentPosition) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0xB;
  requestToYourZone.data1 = skippedColumns;
  requestToYourZone.data2 = currentPosition;

  return requestToYourZone;
}

// Fabricate bot expected request
RequestToYourZoneModel IntimateFrames(int position) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0xF;
  requestToYourZone.data1 = position;
  if (xSemaphoreTake(xZeroCount, portMAX_DELAY) == pdTRUE) {
    requestToYourZone.data2 = (zeroCount[position] * 255) / 16383;
    xSemaphoreGive(xZeroCount);
  }
  return requestToYourZone;
}

// Fabricate bot expected request
RequestToYourZoneModel IntimateLoopTimeIncrease() {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0x11;
  return requestToYourZone;
}

// Fabricate CAN comm. failure request
RequestToYourZoneModel IntimateRetryCommunication(int retry, int requestType) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0x12;
  requestToYourZone.data1 = retry;
  requestToYourZone.data2 = requestType;
  
  return requestToYourZone;
}

void TAIL_AND_COMM_TASK(void* parameters) {
  while (true) {
    vTaskDelay(2 / portTICK_PERIOD_MS);
    bool changeInPosition = false;
    int prevPosition = myZoneBotPositionForCore0;
    if (xSemaphoreTake(xMyZoneBotPosition, portMAX_DELAY) == pdTRUE) {
      if (myZoneBotPositionForCore0 != myZoneBotPosition) {
        // add_log("Bot Position: " + String(myZoneBotPosition));
        myZoneBotPositionForCore0 = myZoneBotPosition;
        changeInPosition = true;
      }
      xSemaphoreGive(xMyZoneBotPosition);
    }

    if (changeInPosition) {
      // Case: If previously MyZoneBotPosition was -1 then bot is getting detected in the zone for the first time
      if (myZoneBotPositionForCore0 != -1 && prevPosition == -1) {
        botDetectedFirstTime = true;
      }

      if (myZoneBotPositionForCore0 != -1) {
        SetVirtualColumnValue(myZoneBotPositionForCore0, virtualZoneMatrix[0][myZoneBotPositionForCore0] * 255, 255, virtualZoneMatrix[2][myZoneBotPositionForCore0] * 255);
        zoneMatrix.setPixelColor(myZoneBotPositionForCore0, zoneMatrix.Color(virtualZoneMatrix[0][myZoneBotPositionForCore0] * 255, 255, virtualZoneMatrix[2][myZoneBotPositionForCore0] * 255));

        // add_log("Bot detected @ " + String(myZoneBotPositionForCore0));
        if (columnCount - myZoneBotPositionForCore0 - 1 < expectedSafetyRange) {
          maxExpectedRangeSent = expectedSafetyRange - (columnCount - myZoneBotPositionForCore0 - 1);
          yourZoneExpectedValue = expectedSafetyRange - (columnCount - myZoneBotPositionForCore0 - 1);
          RequestToYourZoneModel request = IntimateExpectedBotRequest(yourZoneExpectedValue);
          AddRequestToYourZoneTable(request);
        }
        // RequestToYourZoneModel request = IntimateFrames(myZoneBotPositionForCore0);
        // AddRequestToYourZoneTable(request);
      }

      if (prevPosition != -1) {
        SetVirtualColumnValue(prevPosition, virtualZoneMatrix[0][prevPosition] * 255, 0, virtualZoneMatrix[2][prevPosition] * 255);
        zoneMatrix.setPixelColor(prevPosition, zoneMatrix.Color(virtualZoneMatrix[0][prevPosition] * 255, 0, virtualZoneMatrix[2][prevPosition] * 255));
        if (myZoneBotPositionForCore0 == -1 && maxExpectedRangeSent < expectedSafetyRange && columnCount - 1 - prevPosition > 0) {
          RequestToYourZoneModel errorRequest = IntimateSkippedColumnError(columnCount - 1 - prevPosition, prevPosition);
          AddRequestToYourZoneTable(errorRequest);
        }

        if (myZoneBotPositionForCore0 != -1 && myZoneBotPositionForCore0 - prevPosition > 1) {
          RequestToYourZoneModel errorRequest = IntimateSkippedColumnError(myZoneBotPositionForCore0 - prevPosition - 1, myZoneBotPositionForCore0);
          AddRequestToYourZoneTable(errorRequest);
        }
      }
      changeInPosition = false;
    }

    myZoneTailLength = TailRegulator(myZoneBotPositionForCore0, 0, 0);
    pair<int, int> segmentorResult = TailSegmentor(myZoneTailLength, myZoneBotPositionForCore0, yourZoneBotMyZoneTailActiveLength_A, yourZoneBotMyZoneTailActiveLength_B);
    myZoneTailActiveLength = segmentorResult.first;
    yourZoneTailLength = segmentorResult.second;
    // TailSuperimposer(myZoneBotPositionForCore0, myZoneTailActiveLength, myZoneMirrorBotPosition, myZoneMirrorTailActiveLength, yourZoneBotMyZoneTailActiveLength_A, yourZoneBotMyZoneTailActiveLength_B);

    if (previousYourZoneTailLength != yourZoneTailLength) {
      RequestToYourZoneModel tailRequest = IntimationTailTransfer(yourZoneTailLength, previousZoneID_A);
      AddRequestToYourZoneTable(tailRequest);
      previousYourZoneTailLength = yourZoneTailLength;
    }

    bool intimateLoopTimeIncrease0 = false;
    if (xSemaphoreTake(xintimateLoopTimeIncrease, portMAX_DELAY) == pdTRUE) {
      intimateLoopTimeIncrease0 = intimateLoopTimeIncrease;
      intimateLoopTimeIncrease = false;
      xSemaphoreGive(xintimateLoopTimeIncrease);
    }
    if (intimateLoopTimeIncrease0) {
      RequestToYourZoneModel request = IntimateLoopTimeIncrease();
      AddRequestToYourZoneTable(request);
    }

    ResolveRequestToYourZoneTable();
    ResolveRequestFromYourZoneTable();
  }
}

void CAN_RECEIVE_TASK(void* parameters) {
  while (true) {
    RecieveRequest();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  // WiFiConfig();
  xMyZoneBotPosition = xSemaphoreCreateBinary();
  xSemaphoreGive(xMyZoneBotPosition);
  xBotClearingFlag = xSemaphoreCreateBinary();
  xSemaphoreGive(xBotClearingFlag);
  xUpdateExpectedStatusFlag = xSemaphoreCreateBinary();
  xSemaphoreGive(xUpdateExpectedStatusFlag);
  xZeroCount = xSemaphoreCreateBinary();
  xSemaphoreGive(xZeroCount);
  xintimateLoopTimeIncrease = xSemaphoreCreateBinary();
  xSemaphoreGive(xintimateLoopTimeIncrease);
  xOTAFlag = xSemaphoreCreateBinary();
  xSemaphoreGive(xOTAFlag);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOOT_BUTTON_PIN), ButtonISR, FALLING);
  BoardConfig();
  debugLoggingString = "Z: " + String(currentZoneID) + " " + CODE_ID + ": ";
  // xTaskCreatePinnedToCore(
  //   HTTP_DEBUG_LOGGER,
  //   "debug_logging",
  //   10000,
  //   NULL,
  //   1,
  //   &httpDebugLog,
  //   0);
  NeoPixelConfig();
  CANconfig();
  for (int i = 0; i < 20; i++) {
    pinMode(botIndicatorPins[i], INPUT);
  }

  // Turn ON column indicators
  for (int i = 0; i < columnCount; i++) {
    SetVirtualColumnValue(i, 255, 0, 255);
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(255, 0, 255));
  }
  zoneMatrix.show();
  Serial.println("Zone: Z: " + String(currentZoneID) + " Code: " + CODE_ID);
  Serial.println(__FILE__);
  xTaskCreatePinnedToCore(
    TAIL_AND_COMM_TASK,
    "tail_and_comm_task",
    14000,
    NULL,
    2,
    &tailAndCommTask,
    0);
  vTaskDelay(pdMS_TO_TICKS(20));
  xTaskCreatePinnedToCore(
    CAN_RECEIVE_TASK,
    "CAN_RECEIVE_TASK",
    2048,
    NULL,
    3,
    &recieveRequestTask,
    0);
}

void loop() {
  loopStartTime = micros();
  if (xSemaphoreTake(xOTAFlag, portMAX_DELAY) == pdTRUE) {
    if (otaFlag) {
      Serial.println("Taking OTA!");
      if (tailAndCommTask != NULL) {
        vTaskDelete(tailAndCommTask);
        tailAndCommTask = NULL;
      }
      if (recieveRequestTask != NULL) {
        vTaskDelete(recieveRequestTask);
        recieveRequestTask = NULL;
      }
      delay(100);
      WiFiConfig();
      CheckForOTAUpdate();
    }
    xSemaphoreGive(xOTAFlag);
  }

  // if (xSemaphoreTake(xMyZoneBotPosition, portMAX_DELAY) == pdTRUE) {
  //   bool botDetectedOnNewColumn = BotDetection();
  //   xSemaphoreGive(xMyZoneBotPosition);
  // }

  // if (xSemaphoreTake(xUpdateExpectedStatusFlag, portMAX_DELAY) == pdTRUE) {
  //   if (needToUpdateExpectedStatus) {
  //     if (valueForUpdateExpectedStatus > columnCount) {
  //       for (int i = 0; i < columnCount; i++) {
  //         expectedStatus[i] = true;
  //       }
  //     } else {
  //       for (int i = 0; i < valueForUpdateExpectedStatus; i++) {
  //         expectedStatus[i] = true;
  //       }
  //     }
  //     needToUpdateExpectedStatus = false;
  //     valueForUpdateExpectedStatus = -1;
  //   }
  //   xSemaphoreGive(xUpdateExpectedStatusFlag);
  // }

  PerformanceMonitor();
}

void NeoPixelConfig() {
  zoneMatrix.begin();             // INITIALIZE NeoPixel zoneMatrix object (REQUIRED)
  zoneMatrix.show();              // Turn OFF all pixels ASAP
  zoneMatrix.setBrightness(255);  // Set BRIGHTNESS to about 1/5 (max = 255)
}

void sendDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");

  // Get unique ESP32 ID
  // Start JSON with esp_id field
  String json = "{";
  json += "\"esp_id\":\"" + String(currentZoneID) + "\",";
  json += "\"channels\":{";

  for (int ch = 0; ch < columnCount; ch++) {
    json += "\"" + String(ch) + "\":" + String(zeroCount[ch]);
    if (ch < columnCount - 1) json += ",";
  }

  json += "}}";  // Close columnCount and root

  int code = http.POST(json);
  if (code > 0) {
    Serial.printf("Data sent → HTTP %d\n", code);
  } else {
    Serial.printf("Error sending data: %s\n", http.errorToString(code).c_str());
  }

  http.end();
}

// Configuration function to read non-volatile memory of EEPROM
void BoardConfig() {
  preferences.begin("board-config", false);
  currentZoneID = preferences.getInt("CurrentZoneID", -1);
  zoneType = preferences.getString("ZoneType", "regular");
  columnCount = preferences.getInt("ColumnCount", 0);
  minimumSafetyGap = preferences.getFloat("MinSafGap", 3);
  previousZoneID_A = preferences.getInt("PrevZoneID_A", -1);
  previousZoneID_B = preferences.getInt("PrevZoneID_B", -1);
  nextZoneID_A = preferences.getInt("NextZoneID_A", -1);
  nextZoneID_B = preferences.getInt("NextZoneID_B", -1);
  slaveZoneID = preferences.getInt("SlaveId", -1);
  expectedSafetyRange = preferences.getInt("ExpGap", -1);
  
  Serial.println("Board Type: " + zoneType + ", Column Count: " + String(columnCount) + ", Current Zone ID: Z" + String(currentZoneID) + ", Previous Zone ID: Z" + String(previousZoneID_A) + ", Next Zone ID: Z" + String(nextZoneID_A) + ", SlaveId: Z" + String(slaveZoneID) + ", Minimum Safety Gap (m): " + String(minimumSafetyGap, 3) + " and Expected Status Gap: " + String(expectedSafetyRange));
}

// Configuration of the CAN communication
void CANconfig() {
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  // Start the CAN bus
  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
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

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    if (debugLoggingString != "Z: " + String(currentZoneID) + " " + CODE_ID + ": ") {
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
        debugLoggingString = "Z: " + String(currentZoneID) + " " + CODE_ID + ": ";
      }
    }
  }
}

// Function to add logs into the log string
void add_log(String log) {
  debugLoggingString += " | " + log;
}

// Function to montior the loop time and memory usage of the controller
void PerformanceMonitor() {
  // RAM monitoring
  size_t freeHeap = ESP.getFreeHeap();   // Free heap memory
  size_t totalHeap = ESP.getHeapSize();  // Total heap memory
  float ramUsage = 100.0 - ((float)freeHeap / totalHeap * 100.0);
  if (ramUsage > 70) {
    // add_log("Ram usage has exceeded 70%!");
    // add_log("Total ram available (b): " + String(totalHeap));
    // add_log("RAM used (b): " + String(totalHeap - freeHeap));
    // add_log("Free space left (b): " + String(freeHeap));
    // add_log("RAM used (%): " + String(ramUsage) + " %");
  }

  // Loop time monitoring
  unsigned long loopTime = micros() - loopStartTime;
  if (loopTime > maxLoopTime) {
    if (xSemaphoreTake(xintimateLoopTimeIncrease, portMAX_DELAY) == pdTRUE) {
      intimateLoopTimeIncrease = true;
      xSemaphoreGive(xintimateLoopTimeIncrease);
    }
  }
  if (totalHeap - freeHeap > maxMemoryUsed) {
    maxMemoryUsed = totalHeap - freeHeap;
    // add_log("New Max. Memory Usage (b): " + String(totalHeap - freeHeap));
  }
}