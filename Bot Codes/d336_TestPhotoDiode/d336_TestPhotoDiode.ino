#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <Update.h>
#include <CAN.h>
using namespace std;

#define CAN_SPEED 1000E3
#define NODE_COUNT 20

// GPIO Pin Assignments
#define ZONE_MATRIX_PIN 12
#define BOOT_BUTTON_PIN 0
#define CAN_RX_PIN 21
#define CAN_TX_PIN 22
const int botIndicatorPins[] = { 36, 39, 34, 35, 32, 33, 25, 26, 27, 14,
                         13, 23, 2, 19, 18, 5, 17, 16, 4, 15 };

// Structs
struct RequestToYourZoneModel {
  uint8_t requestId = -1;
  uint8_t receiverId = 0;
  uint8_t originalSenderId = 0;
  uint8_t requestType = -1;
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
  uint8_t requestType = -1;
  String CommunicatorStatus = "PENDING";
  String CANCommunicatorStatus = "PENDING";
  float data1 = 0.0;
  float data2 = 0.0;
  float data3 = 0.0;
  float data4 = 0.0;
  float data5 = 0.0;
};

// Constant variables
const String CODE_ID = "d336-WTMv2.2.36v1";
const char* SSID = "Server_PC";
const char* PASSWORD = "msort@flexli";
const String HTTP_DEBUG_SERVER_URL = "http://192.168.2.109:5000/data";
const String OTA_URL = "http://192.168.2.109:5000/download_WTM";
const float INTER_COLUMN_PITCH = 0.04;
const float ACCELERATION = 1.87;
const float NET_DISTANCE_TO_MERGE_POINT = 3;
const float MAX_SPEED = 3;
const float THRESHOLD_DISTANCE_FOR_MESSAGE_HOPING = 3.5;

// Monitoring variables
unsigned long maxLoopTime;
unsigned long loopStartTime;
int numsOfIteration = 0;
unsigned long maxLatency = 0;
size_t maxMemoryUsed = 0;

// Logger variables
String debugLoggingString = "";
HTTPClient httpDebugger;
TaskHandle_t httpDebugLog;
bool loggerFlag = true;
volatile bool otaFlag = false;

// Zone Variables
Preferences preferences;
Adafruit_NeoPixel zoneMatrix(20, ZONE_MATRIX_PIN, NEO_RGB + NEO_KHZ800);
bool botIndicatorVirtualPins[20] = {0};
bool virtualZoneMatrix[3][100] = {0};
uint8_t currentZoneID;
uint8_t previousZoneID_A;
uint8_t previousZoneID_B;
uint8_t nextZoneID_A;
uint8_t nextZoneID_B;
uint8_t mirrorZoneID;
int columnCount;
String zoneType;
int myZoneBotPosition = -1;
float myZoneBotSpeed = 0;
float yourZoneBotSpeed = 0;
float upcomingMyZoneBotSpeed = 0;
bool yourZoneBotAcceleration = true;
float yourZoneBotDistance = 0;
int myZoneTailLength = 0;
int myZoneTailActiveLength = 0;
int yourZoneTailLength = 0;
int yourZoneBotMyZoneTailActiveLength_A = 0;
int yourZoneBotMyZoneTailActiveLength_B = 0;
int myZoneMirrorBotPosition = -1;
float myZoneMirrorBotSpeed = 0;
bool myZoneMirrorBotAcceleration = false;
int myZoneMirrorTailActiveLength = 0;
bool intimationTailTest = false;
RequestToYourZoneModel requestToYourZoneTable[20];
RequestFromYourZoneModel requestFromYourZoneTable[20];
RequestToYourZoneModel tempRequestToYourZoneTable[20];
RequestFromYourZoneModel tempRequestFromYourZoneTable[20];
int requestFromTableIndex = 0;
int requestToTableIndex = 0;
bool lastBotPreference = false;
int requestId = 0;
unsigned long realYourZoneBotUpdateTimeStamp = 0;
unsigned long realMirrorZoneBotUpdateTimeStamp = 0;
unsigned long lastColumnBotDetectionTime = 0;
int previousYourZoneTailLength = 0;
bool highDetectedStatus[20] = { false };
unsigned long lowStartTime[20] = { 0 };
unsigned long highStartTime[20] = { 0 };
bool botIsPresent[20] = { false };
bool botDetectedFirstTime = false;
bool expectedStatus[20] = { true };
int startStationColumn;
int endStationColumn;
bool errorMode = false;
int errorCode = -1;
bool highFrequencyNoiseNotReported[20] = { true };
bool lowFrequencyNoiseNotReported[20] = { true };
int cycleNumber = 0;
uint8_t slaveZoneID;
float minimumSafetyGap;
int expectedSafetyRange = 5;
SemaphoreHandle_t requestToTableMutex;
SemaphoreHandle_t requestFromTableMutex;
TaskHandle_t resolveRequestToTask;

// Interrupt service routine for boot button press
void IRAM_ATTR ButtonISR() {
  otaFlag = true;
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
  if (request.requestType == 0x0 || request.requestType == 0x3 || request.requestType == 0x5 || request.requestType == 0x7 || request.requestType == 0x8 || request.requestType == 0xB || request.requestType == 0xC || request.requestType == 0xD || request.requestType == 0xE) {
    CAN.write((uint8_t) request.data1);
  } else {
    CAN.write((uint8_t) round((request.data1 * 100.0 * 255.0) / 512.0));
  }
  // Case: If condition true then write 2nd and 3rd part of the payload (1 byte each - total 2 bytes)
  if (request.requestType == 0x1 || request.requestType == 0x2 || request.requestType == 0x5) {
    CAN.write((uint8_t) round((request.data2 * 100.0 * 255.0) / 512.0));
    CAN.write((uint8_t) round((request.data3 * 100.0 * 255.0) / 512.0));
  } else if (request.requestType == 0xB) {
    CAN.write((uint8_t) request.data2);
  }
  // Case: If condition true then write 4th part of the payload (1 byte)
  if (request.requestType == 0x2) {
    CAN.write((uint8_t) round((request.data4 * 100.0 * 255.0) / 512.0));
  }
  // End the CAN frame
  CAN.endPacket();
}

// Function to add an incoming request from YourZone in the RequestFromYourZoneTable
void AddRequestFromYourZoneTable(RequestFromYourZoneModel request) {
  if (xSemaphoreTake(requestFromTableMutex, portMAX_DELAY) == pdTRUE) {
    if (requestFromTableIndex >= 20) {
      // add_log("OVERFLOW_ERROR: 'requestFromTableIndex' is out of range! Setting index back to '0'.");
      requestFromTableIndex = 0;
    }
    requestFromYourZoneTable[requestFromTableIndex] = request;
    requestFromTableIndex++;
    xSemaphoreGive(requestFromTableMutex);
  }
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
          if (recievedRequest.requestType == 0x0 || recievedRequest.requestType == 0x3 || recievedRequest.requestType == 0x5 || recievedRequest.requestType == 0x7 || recievedRequest.requestType == 0x8) {
            recievedRequest.data1 = CAN.read();
          } else {
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
        }
      }
    }
  }
}

// Function to add an outgoing request to YourZone in the RequestToYourZoneTable
void AddRequestToYourZoneTable(RequestToYourZoneModel request) {
  if (xSemaphoreTake(requestToTableMutex, portMAX_DELAY) == pdTRUE) {
    if (requestToTableIndex >= 20) {
      // add_log("OVERFLOW_ERROR: 'requestToTableIndex' is out of range! Setting index back to '0'.");
      requestToTableIndex = 0;
    }
    requestToYourZoneTable[requestToTableIndex] = request;
    requestToTableIndex++;
    xSemaphoreGive(requestToTableMutex);
  }
}

// Copy new entries to temp table
void CopyNewRequestsToTemp() {
  if (xSemaphoreTake(requestToTableMutex, portMAX_DELAY) == pdTRUE) {
    // Clear the temp table
    for (int i = 0; i < 20; i++) {
      tempRequestToYourZoneTable[i] = RequestToYourZoneModel();
    }
    // Copy all entries from main to temp table
    for (int i = 0; i < 20; i++) {
      tempRequestToYourZoneTable[i] = requestToYourZoneTable[i];
    }
    xSemaphoreGive(requestToTableMutex);
  }
}

void UpdateRealListFromTemp() {
  if (xSemaphoreTake(requestToTableMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 20; i++) {
      int tempId = tempRequestToYourZoneTable[i].requestId;
      int tempType = tempRequestToYourZoneTable[i].requestType;

      // Skip empty/invalid temp entries
      if (tempId == -1 && tempType == -1) {
        continue;
      }

      // Search in real list
      for (int j = 0; j < 20; j++) {
        if (requestToYourZoneTable[j].requestId == tempId &&
            requestToYourZoneTable[j].requestType == tempType) {
          // Update real list entry with temp data
          requestToYourZoneTable[j] = tempRequestToYourZoneTable[i];
          if (tempRequestToYourZoneTable[i].CommunicatorStatus == "COMPLETE" && tempRequestToYourZoneTable[i].CANCommunicatorStatus == "COMPLETE") {
            for (int k = j; k < 19; k++) {
              requestToYourZoneTable[k] = requestToYourZoneTable[k + 1];
            }
            requestToYourZoneTable[19] = RequestToYourZoneModel();
            requestToTableIndex--;
          }
          break;
        }
      }
    }
    xSemaphoreGive(requestToTableMutex);
  }
}

// Copy new entries to temp table
void CopyNewRequestsToFromTemp() {
  if (xSemaphoreTake(requestFromTableMutex, portMAX_DELAY) == pdTRUE) {
    // Clear the temp table
    for (int i = 0; i < 20; i++) {
      tempRequestFromYourZoneTable[i] = RequestFromYourZoneModel();
    }
    // Copy all entries from main to temp table
    for (int i = 0; i < 20; i++) {
      tempRequestFromYourZoneTable[i] = requestFromYourZoneTable[i];
    }
    xSemaphoreGive(requestFromTableMutex);
  }
}

void UpdateRealListFromFromTemp() {
  if (xSemaphoreTake(requestFromTableMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 20; i++) {
      int tempId = tempRequestFromYourZoneTable[i].requestId;
      int tempType = tempRequestFromYourZoneTable[i].requestType;

      // Skip empty/invalid temp entries
      if (tempId == -1 && tempType == -1) {
        continue;
      }

      // Search in real list
      for (int j = 0; j < 20; j++) {
        if (requestFromYourZoneTable[j].requestId == tempId &&
            requestFromYourZoneTable[j].requestType == tempType) {
          // Update real list entry with temp data
          requestFromYourZoneTable[j] = tempRequestFromYourZoneTable[i];
          if (tempRequestFromYourZoneTable[i].CommunicatorStatus == "COMPLETE" && tempRequestFromYourZoneTable[i].CANCommunicatorStatus == "COMPLETE") {
            for (int k = j; k < 19; k++) {
              requestFromYourZoneTable[k] = requestFromYourZoneTable[k + 1];
            }
            requestFromYourZoneTable[19] = RequestFromYourZoneModel();
            requestFromTableIndex--;
          }
          break;
        }
      }
    }
    xSemaphoreGive(requestFromTableMutex);
  }
}

//Function to resolve the requests in RequestToYourZoneTable
void ResolveRequestToYourZoneTable() {
  for(int i = 0 ; i < 20 ; i++) {
    RequestToYourZoneModel request = requestToYourZoneTable[i];
    if (request.requestId == 255) continue;
    if (request.CommunicatorStatus == "COMPLETE" && request.CANCommunicatorStatus == "COMPLETE") {
      for (int j = i; j < 19; j++) {
        requestToYourZoneTable[j] = requestToYourZoneTable[j + 1];
      }
      requestToYourZoneTable[19] = RequestToYourZoneModel();
      requestToTableIndex--;
      i--;
      continue;
    }

    if (request.CANCommunicatorStatus == "PENDING") {
      // PrintRequestToYourZone(request);
      SendRequest(request);
      request.CANCommunicatorStatus = "COMPLETE";
      requestToYourZoneTable[i] = request;
    }
  }
}

//Function to resolve the requests in RequestToYourZoneTable
void ResolveRequestToYourZoneTempTable() {
  for(int i = 0 ; i < 20 ; i++) {
    RequestToYourZoneModel request = tempRequestToYourZoneTable[i];
    if (request.requestId == 255) continue;

    if (request.CANCommunicatorStatus == "PENDING") {
      // PrintRequestToYourZone(request);
      SendRequest(request);
      request.CANCommunicatorStatus = "COMPLETE";
      tempRequestToYourZoneTable[i] = request;
    }
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
  if (botDetectedFirstTime) {
    requestToYourZone.requestType = 0x7;
    botDetectedFirstTime = false;
  }
  requestToYourZone.data1 = yourZoneTailLength;
  
  return requestToYourZone;
}

// Handle the incoming tail transfer request
void HandleTailTransfer(RequestFromYourZoneModel request) {
  if (request.requestType == 0x7 && myZoneBotPosition != -1) {
    SetVirtualColumnValue(myZoneBotPosition, virtualZoneMatrix[0][myZoneBotPosition] * 255, 0, virtualZoneMatrix[2][myZoneBotPosition] * 255);
    zoneMatrix.setPixelColor(myZoneBotPosition, zoneMatrix.Color(virtualZoneMatrix[0][myZoneBotPosition] * 255, 0, virtualZoneMatrix[2][myZoneBotPosition] * 255));
    zoneMatrix.show();
    int skippedColumns = columnCount - myZoneBotPosition - 1;
    if (skippedColumns > 0) {
      // add_log("Number of skipped columns: " + String(skippedColumns));
      // add_log("Last detected on column: " + String(myZoneBotPosition));
      RequestToYourZoneModel errorRequest = IntimateSkippedColumnError(skippedColumns, myZoneBotPosition);
      AddRequestToYourZoneTable(errorRequest);
    }
    myZoneBotPosition = -1;
    myZoneBotSpeed = 0;
    for(int i = 0 ; i < columnCount ; i++) {
      expectedStatus[i] = false;
      highFrequencyNoiseNotReported[i] = true;
      lowFrequencyNoiseNotReported[i] = true;
    }
    cycleNumber++;
  }
  
  if (request.senderId  == nextZoneID_A) {
    yourZoneBotMyZoneTailActiveLength_A = (int) request.data1;
    intimationTailTest = true;
    // add_log("Updated yourZoneBotMyZoneTailActiveLength_A: " + String(yourZoneBotMyZoneTailActiveLength_A));
  } else {
    yourZoneBotMyZoneTailActiveLength_B = (int) request.data1;
  }

  RequestToYourZoneModel ackRequest;
  ackRequest.requestId = request.requestId;
  ackRequest.receiverId = request.senderId;
  ackRequest.originalSenderId = currentZoneID;
  ackRequest.requestType = 0x0;
  ackRequest.CommunicatorStatus = "COMPLETE";
  
  AddRequestToYourZoneTable(ackRequest);
}

// Fabricate bot ETA request
RequestToYourZoneModel YourZoneBotETA (int myZoneBotPosition, float myZoneBotSpeed, bool myZoneBotAcceleration) {
  // add_log("YOUR_ZONE_BOT_ETA");
  // add_log("MyZoneBotPosition: " + String(myZoneBotPosition) + " MyZoneBotSpeed: " + String(myZoneBotSpeed) + " MyZoneBotAcceleration: " + String(myZoneBotAcceleration));
  
  // Compute exact distance till merge point and then compute ETA
  float distanceToMergePoint = NET_DISTANCE_TO_MERGE_POINT - myZoneBotPosition * INTER_COLUMN_PITCH;
  float ETA = FindTimeToReachAtGivenDistanceAccordingToGivenSpeed(distanceToMergePoint, myZoneBotSpeed, !myZoneBotAcceleration);
  
  // Create a new request and assign ETA to the request payload
  RequestToYourZoneModel request;
  request.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  request.receiverId = mirrorZoneID;
  request.originalSenderId = currentZoneID;
  request.requestType = 0x4;
  request.data1 = ETA;
  return request;
}

// Handle the incoming bot ETA request
void HandleBotETARequest(int myZoneBotPosition, float myZoneBotSpeed, bool myZoneBotAcceleration, float yourZoneBotSpeed, bool yourZoneBotAcceleration, float yourZoneBotDistance, RequestFromYourZoneModel request) {
  // add_log("HANDLE_BOT_ETA_REQUEST");
  // add_log("MyZoneBotPosition: " + String(myZoneBotPosition) + " MyZoneBotSpeed: " + String(myZoneBotSpeed) + " MyZoneBotAcceleration: " + String(myZoneBotAcceleration));
  // add_log("YourZoneBotSpeed: " + String(myZoneBotSpeed) + " YourZoneBotDistance: " + String(yourZoneBotDistance) + " YourZoneBotAcceleration: " + String(yourZoneBotAcceleration));
  // add_log ("Received request: ");
  // PrintRequestFromYourZone(request);
  // Create and initiate acknowledgment request
  RequestToYourZoneModel ackRequest;
  ackRequest.requestId = request.requestId;
  ackRequest.receiverId = request.senderId;
  ackRequest.originalSenderId = currentZoneID;
  ackRequest.requestType = 0x0;
  ackRequest.CommunicatorStatus = "COMPLETE";

  // Case: If condition true then calculate ETA of the bot which is in the zone else calculate ETA of the bot coming from behind
  if (myZoneBotPosition != -1) {
    float distanceToMergePoint = NET_DISTANCE_TO_MERGE_POINT - myZoneBotPosition * INTER_COLUMN_PITCH;
    float ETA = FindTimeToReachAtGivenDistanceAccordingToGivenSpeed(distanceToMergePoint, myZoneBotSpeed, !myZoneBotAcceleration);
    // Case: If calculated ETA is less than the received ETA then preference is MyBot(1) else YourBot(0)
    if (ETA < request.data1) {
      lastBotPreference = true;
      ackRequest.data1 = 1;
    } else {
      lastBotPreference = false;
      ackRequest.data1 = 0;
    }
  } else {
    float distanceToMergePoint = NET_DISTANCE_TO_MERGE_POINT + yourZoneBotDistance;
    float ETA = FindTimeToReachAtGivenDistanceAccordingToGivenSpeed(distanceToMergePoint, yourZoneBotSpeed, !yourZoneBotAcceleration);
    // Case: If calculated ETA is less than the received ETA then preference is MyBot(1) else YourBot(0)
    if (ETA < request.data1) {
      lastBotPreference = true;
      ackRequest.data1 = 1;
    } else {
      lastBotPreference = false;
      ackRequest.data1 = 0;
    }
  }
  
  // Add acknowledgement request to RequestToYourZoneTable
  AddRequestToYourZoneTable(ackRequest);
}

// Fabricate the bot transfer request for the next zone
RequestToYourZoneModel IntimationBotTransfer(int myZoneBotPosition, float myZoneBotSpeed, bool myZoneBotAcceleration, float yourZoneBotSpeed, bool yourZoneBotAcceleration, float yourZoneBotDistance, uint8_t senderId, bool mirrorFlag, bool divergeCase) {
  // add_log("INTIMATION_BOT_TRANSFER");
  // add_log("MyZoneBotPosition: " + String(myZoneBotPosition) + " MyZoneBotSpeed: " + String(myZoneBotSpeed) + " MyZoneBotAcceleration: " + String(myZoneBotAcceleration));
  // add_log("YourZoneBotSpeed: " + String(myZoneBotSpeed) + " YourZoneBotDistance: " + String(yourZoneBotDistance) + " YourZoneBotAcceleration: " + String(yourZoneBotAcceleration));
  // add_log("Sender ID: " + String(senderId) + " MirrorFlag: " + String(mirrorFlag) + " DivergeCase: " + String(divergeCase));
  // Create a new request
  RequestToYourZoneModel request;
  request.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  request.originalSenderId = senderId;
  // Case: If condition true then send bot parameters for bot mirroring else either of column change, zone transfer or message hoping
  if (mirrorFlag) {
    request.receiverId = mirrorZoneID;
    request.requestType = 0x5;
    request.data1 = myZoneBotPosition;
    request.data2 = myZoneBotSpeed;
    request.data3 = myZoneBotAcceleration;
    return request;
  } else {
    request.receiverId = nextZoneID_A;
    // Case: If condition true then set rceieverId to zoneId of another diverging lane
    if (divergeCase) {
      request.receiverId = nextZoneID_B;
    }
    // Case: If condition true then zone transfer else either of column change or message hoping
    if (myZoneBotPosition == columnCount - 1) {
      request.requestType = 0x2;
      request.data1 = myZoneBotSpeed;
      request.data2 = yourZoneBotSpeed;
      request.data3 = yourZoneBotAcceleration;
      request.data4 = yourZoneBotDistance;
      return request;
    }
    request.requestType = 0x1;
    // Case: If condition true then message hoping else column change
    if (myZoneBotPosition == -1) {
      request.data1 = yourZoneBotSpeed;
      request.data2 = yourZoneBotAcceleration;
      request.data3 = yourZoneBotDistance + columnCount * INTER_COLUMN_PITCH;     // Message hoping hence add current zone length in the yourZoneBotDistance
      return request;
    } else {
      request.data1 = myZoneBotSpeed;
      request.data2 = myZoneBotAcceleration;
      request.data3 = (columnCount - myZoneBotPosition) * INTER_COLUMN_PITCH;     // Column change hence new distance of the bot from the next zone
      return request;
    }
  }
}

// Handle the incoming bot transfer request
void HandleBotTransferRequest(int myZoneBotPosition, RequestFromYourZoneModel request) {
  // add_log("HANDLE_BOT_TRANSFER_REQUEST");
  // add_log("MyZoneBotPosition: " + String(myZoneBotPosition));
  // add_log ("Received request: ");
  // PrintRequestFromYourZone(request);
  // Case: If condition true then column change or message hoping
  if (request.requestType == 0x1) {
    // Case: If condition true then message hoping else column change
    if (myZoneBotPosition == -1) {
      // Case: If condition true then message hoping termination else creation of a hop request
      if (request.data3 > THRESHOLD_DISTANCE_FOR_MESSAGE_HOPING) {
        yourZoneBotSpeed = 0;
        yourZoneBotAcceleration = 0; 
        yourZoneBotDistance = THRESHOLD_DISTANCE_FOR_MESSAGE_HOPING;
      } else {
        yourZoneBotSpeed = request.data1;
        yourZoneBotAcceleration = ((int) request.data2 >= 1);
        yourZoneBotDistance = request.data3;
        realYourZoneBotUpdateTimeStamp = millis();
        RequestToYourZoneModel hopRequest = IntimationBotTransfer(myZoneBotPosition, myZoneBotSpeed, 0, yourZoneBotSpeed, yourZoneBotDistance, yourZoneBotAcceleration, request.originalSenderId, false, false);
        AddRequestToYourZoneTable(hopRequest);
      }
    } else {
      yourZoneBotSpeed = request.data1;
      yourZoneBotAcceleration = ((int) request.data2 >= 1);
      yourZoneBotDistance = request.data3;
      realYourZoneBotUpdateTimeStamp = millis();
    }
  } 
  // Case: If condition true then zone transfer request
  else if (request.requestType == 0x2) {
    upcomingMyZoneBotSpeed = request.data1;
    yourZoneBotSpeed = request.data2;
    yourZoneBotAcceleration = ((int) request.data3 >= 1);
    yourZoneBotDistance = request.data4;
    realYourZoneBotUpdateTimeStamp = millis();
  }
  // Case: If condition true then zone transfer request
  else if (request.requestType == 0x5) {
    myZoneMirrorBotPosition = (int) request.data1;
    myZoneMirrorBotSpeed = request.data2;
    myZoneMirrorBotAcceleration = ((int) request.data3 >= 1);
    realMirrorZoneBotUpdateTimeStamp = millis();
  }
  
  // Create an ackowledgement request and add it to RequestToYourZoneTable
  RequestToYourZoneModel ackRequest;
  ackRequest.requestId = request.requestId;
  ackRequest.receiverId = request.senderId;
  ackRequest.originalSenderId = currentZoneID;
  ackRequest.requestType = 0x0;
  ackRequest.CommunicatorStatus = "COMPLETE";
  AddRequestToYourZoneTable(ackRequest);
}

// Function to compute expected parameters or safe expected parameters (speed, distance or psoition) for the bot in your zone or mirror bot in my zone
pair<float, float> BotExpectedParameterCalculator(float speed, bool slowDown, float distanceOrPosition, float elapsedTime, bool mirrorFlag, bool safeFlag) {
  // add_log("BOT_EXPECTED_PARAMETER_CALCULATOR");
  // add_log("Speed: " + String(speed) + " Decelaration: " + String(slowDown) + " DistanceOrPosition: " + String(distanceOrPosition) + " ElapsedTime: " + String(elapsedTime) + " MirrorFlag: " + String(mirrorFlag) + " SafeFlag: " + String(safeFlag));
  // Case: If condition true then calculate safe expected parameters else expected parameters
  if (safeFlag) {
    // Case: If condition true then mirror bot calculations else your zone bot calculations
    if (mirrorFlag) {
      float expectedSpeed = speed - ACCELERATION * elapsedTime;
      expectedSpeed = clamp(expectedSpeed, 0.0f, MAX_SPEED);
      // Case: If condition true then calculations according to the fact that bot only decelerated for a part of elapsed time else bot has decelerated the whole time
      if (speed - ACCELERATION * elapsedTime < 0) {
        float timeToDec = speed/ACCELERATION;
        float decDist = (speed*speed)/(2*ACCELERATION);
        int positionJumps = decDist/INTER_COLUMN_PITCH;
        int finalPosition = (int) distanceOrPosition + positionJumps;
        // Case: If condition true that means bot might have crossed the zone
        if (finalPosition >= columnCount) {
          finalPosition = -1;
        }
        pair<float, float> ans = {expectedSpeed, finalPosition};
        return ans;
      } else {
        float decDist = fabs(expectedSpeed*expectedSpeed - speed*speed)/(2*ACCELERATION);
        int positionJumps = decDist/INTER_COLUMN_PITCH;
        int finalPosition = (int) distanceOrPosition + positionJumps;
        // Case: If condition true that means bot might have crossed the zone
        if (finalPosition >= columnCount) {
          finalPosition = -1;
        }
        pair<float, float> ans = {expectedSpeed, finalPosition};
        return ans;
      }
    } else {
      float expectedSpeed = speed + ACCELERATION * elapsedTime;
      expectedSpeed = clamp(expectedSpeed, 0.0f, MAX_SPEED);
      // Case: If condition true then calculations according to the fact that bot only accelerated for a part of elapsed time else bot has accelerated the whole time
      if (speed + ACCELERATION * elapsedTime > MAX_SPEED) {
        float timeToAcc = (MAX_SPEED - speed)/ACCELERATION;
        float timeOfConst = elapsedTime - timeToAcc;
        float accDist = (MAX_SPEED*MAX_SPEED - speed*speed)/(2*ACCELERATION);
        float constDist = MAX_SPEED*timeOfConst;
        float finalDistance = distanceOrPosition - accDist - constDist;
        pair<float, float> ans = {expectedSpeed, finalDistance};
        return ans;
      } else {
        float accDist = fabs(expectedSpeed*expectedSpeed - speed*speed)/(2*ACCELERATION);
        float finalDistance = distanceOrPosition - accDist;
        pair<float, float> ans = {expectedSpeed, finalDistance};
        return ans;
      }
    }
  } else {
    // Case: If condition true then calculations as per decelerated motion else accelerated motion
    if (slowDown) {
      float expectedSpeed = speed - ACCELERATION * elapsedTime;
      expectedSpeed = clamp(expectedSpeed, 0.0f, MAX_SPEED);
      // Case: If condition true then mirror bot calculations else your zone bot calculations
      if (mirrorFlag) {
        // Case: If condition true then calculations according to the fact that bot only decelerated for a part of elapsed time else bot has decelerated the whole time
        if (speed - ACCELERATION * elapsedTime < 0) {
          float timeToDec = speed/ACCELERATION;
          float decDist = (speed*speed)/(2*ACCELERATION);
          int positionJumps = decDist/INTER_COLUMN_PITCH;
          int finalPosition = (int) distanceOrPosition + positionJumps;
          // Case: If condition true that means bot might have crossed the zone
          if (finalPosition >= columnCount) {
            finalPosition = -1;
          }
          pair<float, float> ans = {expectedSpeed, finalPosition};
          return ans;
        } else {
          float decDist = fabs(expectedSpeed*expectedSpeed - speed*speed)/(2*ACCELERATION);
          int positionJumps = decDist/INTER_COLUMN_PITCH;
          int finalPosition = (int) distanceOrPosition + positionJumps;
          // Case: If condition true that means bot might have crossed the zone
          if (finalPosition >= columnCount) {
            finalPosition = -1;
          }
          pair<float, float> ans = {expectedSpeed, finalPosition};
          return ans;
        }
      } else {
        // Case: If condition true then calculations according to the fact that bot only decelerated for a part of elapsed time else bot has decelerated the whole time
        if (speed - ACCELERATION * elapsedTime < 0) {
          float timeToDec = speed/ACCELERATION;
          float decDist = (speed*speed)/(2*ACCELERATION);
          float finalDistance = distanceOrPosition - decDist;
          pair<float, float> ans = {expectedSpeed, finalDistance};
          return ans;
        } else {
          float decDist = fabs(expectedSpeed*expectedSpeed - speed*speed)/(2*ACCELERATION);
          float finalDistance = distanceOrPosition - decDist;
          pair<float, float> ans = {expectedSpeed, finalDistance};
          return ans;
        }
      }
    } else {
      float expectedSpeed = speed + ACCELERATION * elapsedTime;
      expectedSpeed = clamp(expectedSpeed, 0.0f, MAX_SPEED);
      // Case: If condition true then mirror bot calculations else your zone bot calculations
      if (mirrorFlag) {
        // Case: If condition true then calculations according to the fact that bot only accelerated for a part of elapsed time else bot has accelerated the whole time
        if (speed + ACCELERATION * elapsedTime > MAX_SPEED) {
          float timeToAcc = (MAX_SPEED - speed)/ACCELERATION;
          float timeOfConst = elapsedTime - timeToAcc;
          float accDist = (MAX_SPEED*MAX_SPEED - speed*speed)/(2*ACCELERATION);
          float constDist = MAX_SPEED*timeOfConst;
          float finalDistance = accDist + constDist;
          int positionJumps = finalDistance/INTER_COLUMN_PITCH;
          int finalPosition = (int) distanceOrPosition + positionJumps;
          // Case: If condition true that means bot might have crossed the zone
          if (finalPosition >= columnCount) {
            finalPosition = -1;
          }
          pair<float, float> ans = {expectedSpeed, finalPosition};
          return ans;
        } else {
          float accDist = fabs(expectedSpeed*expectedSpeed - speed*speed)/(2*ACCELERATION);
          int positionJumps = accDist/INTER_COLUMN_PITCH;
          int finalPosition = (int) distanceOrPosition + positionJumps;
          // Case: If condition true that means bot might have crossed the zone
          if (finalPosition >= columnCount) {
            finalPosition = -1;
          }
          pair<float, float> ans = {expectedSpeed, finalPosition};
          return ans;
        }
      } else {
        // Case: If condition true then calculations according to the fact that bot only accelerated for a part of elapsed time else bot has accelerated the whole time
        if (speed + ACCELERATION * elapsedTime > MAX_SPEED) {
          float timeToAcc = (MAX_SPEED - speed)/ACCELERATION;
          float timeOfConst = elapsedTime - timeToAcc;
          float accDist = (MAX_SPEED*MAX_SPEED - speed*speed)/(2*ACCELERATION);
          float constDist = MAX_SPEED*timeOfConst;
          float finalDistance = distanceOrPosition - accDist - constDist;
          pair<float, float> ans = {expectedSpeed, finalDistance};
          return ans;
        } else {
          float accDist = fabs(expectedSpeed*expectedSpeed - speed*speed)/(2*ACCELERATION);
          float finalDistance = distanceOrPosition - accDist;
          pair<float, float> ans = {expectedSpeed, finalDistance};
          return ans;
        }
      }
    }
  }
}

//Function to resolve the requests in RequestFromYourZoneTable
void ResolveRequestFromYourZoneTable() {
  for(int i = 0 ; i < 20 ; i++) {
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
        for(int j = 0 ; j < 20 ; j++) {
          RequestToYourZoneModel myZoneRequest = requestToYourZoneTable[i];
          if (myZoneRequest.requestId == request.requestId) {
            if (myZoneRequest.requestType == 0x4) {
              lastBotPreference = myZoneRequest.data1 == 0;
            }
            myZoneRequest.CommunicatorStatus = "COMPLETE";
            requestToYourZoneTable[i] = myZoneRequest;
            break;
          }
        }
      } else if (request.requestType == 0x1 || request.requestType == 0x2 || request.requestType == 0x5) {
        HandleBotTransferRequest(myZoneBotPosition, request);
      } else if (request.requestType == 0x3 || request.requestType == 0x7) {
        HandleTailTransfer(request);
      } else if (request.requestType == 0x8) {
        HandleExpectedBotRequest(request);
      } else if (request.requestType == 0x4) {
        bool myZoneBotAcceleration = 1;
        if (myZoneBotPosition != -1) myZoneBotAcceleration = !virtualZoneMatrix[0][myZoneBotPosition];
        HandleBotETARequest(myZoneBotPosition, myZoneBotSpeed, myZoneBotAcceleration, yourZoneBotSpeed, yourZoneBotAcceleration, yourZoneBotDistance, request);
      }

      request.CommunicatorStatus = "COMPLETE";
      requestFromYourZoneTable[i] = request;
    }
  }
}

//Function to resolve the requests in RequestFromYourZoneTable
void ResolveRequestFromYourZoneTempTable() {
  for(int i = 0 ; i < 20 ; i++) {
    RequestFromYourZoneModel request = tempRequestFromYourZoneTable[i];
    if (request.requestId == 255) continue;
  
    if (request.CommunicatorStatus == "PENDING") {
      if (request.requestType == 0x0) {
        for(int j = 0 ; j < 20 ; j++) {
          RequestToYourZoneModel myZoneRequest = tempRequestToYourZoneTable[i];
          if (myZoneRequest.requestId == request.requestId) {
            if (myZoneRequest.requestType == 0x4) {
              lastBotPreference = myZoneRequest.data1 == 0;
            }
            myZoneRequest.CommunicatorStatus = "COMPLETE";
            tempRequestToYourZoneTable[i] = myZoneRequest;
            break;
          }
        }
      } else if (request.requestType == 0x1 || request.requestType == 0x2 || request.requestType == 0x5) {
        HandleBotTransferRequest(myZoneBotPosition, request);
      } else if (request.requestType == 0x3 || request.requestType == 0x7) {
        HandleTailTransfer(request);
      } else if (request.requestType == 0x8) {
        HandleExpectedBotRequest(request);
      } else if (request.requestType == 0x4) {
        bool myZoneBotAcceleration = 1;
        if (myZoneBotPosition != -1) myZoneBotAcceleration = !virtualZoneMatrix[0][myZoneBotPosition];
        HandleBotETARequest(myZoneBotPosition, myZoneBotSpeed, myZoneBotAcceleration, yourZoneBotSpeed, yourZoneBotAcceleration, yourZoneBotDistance, request);
      }

      request.CommunicatorStatus = "COMPLETE";
      tempRequestFromYourZoneTable[i] = request;
    }
  }
}

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

// Function responsible for the detection of the bot, updating the MyZoneBotSpeed and MyZoneBotPosition
bool BotDetection() {
  // Iterating over all the bot detection pins
  for (int i = 0 ; i < columnCount ; i++) {
    // Case: If condition true then a bot is detected else not
    if (!digitalRead(botIndicatorPins[i])) {
      // Case: If condition true then bot detected at new column
      if (myZoneBotPosition != i) {
        unsigned long currentTime = millis();
        // Case: If condition true then turn OFF the bot LED at previous MyZoneBotPosition
        if (myZoneBotPosition != -1) {
          SetVirtualColumnValue(myZoneBotPosition, virtualZoneMatrix[0][myZoneBotPosition] * 255, 0, virtualZoneMatrix[2][myZoneBotPosition] * 255);
          zoneMatrix.setPixelColor(myZoneBotPosition, zoneMatrix.Color(virtualZoneMatrix[0][myZoneBotPosition] * 255, 0, virtualZoneMatrix[2][myZoneBotPosition] * 255));
        }

        if (myZoneBotPosition == -1) {
          botDetectedFirstTime = true;
        }
        // Update MyZoneBotPosition and tunr ON the bot LED at new MyZoneBotPosition
        myZoneBotPosition = i;
        // add_log("New MyZoneBotPosition: " + String(myZoneBotPosition));
        SetVirtualColumnValue(myZoneBotPosition, virtualZoneMatrix[0][myZoneBotPosition] * 255, 255, virtualZoneMatrix[2][myZoneBotPosition] * 255);
        zoneMatrix.setPixelColor(myZoneBotPosition, zoneMatrix.Color(virtualZoneMatrix[0][myZoneBotPosition] * 255, 255, virtualZoneMatrix[2][myZoneBotPosition] * 255));
        zoneMatrix.show();
        // Case: If condition true then bot detected on '0th' column then MyZoneBotSpeed will be updated based on upcoming bot data else by time calculations
        if (i == 0) {
          myZoneBotSpeed = upcomingMyZoneBotSpeed;
          upcomingMyZoneBotSpeed = 0;
        } else {
          // add_log("Time elapsed: " + String(currentTime - lastColumnBotDetectionTime));
          myZoneBotSpeed = ((float)INTER_COLUMN_PITCH * 1000.0)/(float)(currentTime - lastColumnBotDetectionTime);
        }
        // add_log("New MyZoneBotSpeed: " + String(myZoneBotSpeed));
        // Updating the last bot detection time and return that bot is detected on new column
        lastColumnBotDetectionTime = currentTime;
        return true;
      }
    }

    // if (myZoneBotPosition == columnCount - 1 && millis() - lowStartTime[myZoneBotPosition] > 500) {
    //   SetVirtualColumnValue(myZoneBotPosition, virtualZoneMatrix[0][myZoneBotPosition] * 255, 0, virtualZoneMatrix[2][myZoneBotPosition] * 255);
    //   zoneMatrix.setPixelColor(myZoneBotPosition, zoneMatrix.Color(virtualZoneMatrix[0][myZoneBotPosition] * 255, 0, virtualZoneMatrix[2][myZoneBotPosition] * 255));
    //   zoneMatrix.show();
    //   myZoneBotPosition = -1; 
    //   myZoneBotSpeed = 0;
    // }
  }
  return false;
}

// The handler actually decides how many and what kind of intimation bot transfer requests needs to be made 
void HandleBotDetectedOnNewColumn(int myZoneBotPosition, float myZoneBotSpeed, float yourZoneBotSpeed, bool yourZoneBotAcceleration, float yourZoneBotDistance, int yourZoneBotMyZoneTailActiveLength_A, int yourZoneBotMyZoneTailActiveLength_B) {
  // Case: If condition true then make bot transfer request of zone transfer
  RequestToYourZoneModel request = IntimationBotTransfer(myZoneBotPosition, myZoneBotSpeed, !virtualZoneMatrix[1][myZoneBotPosition], yourZoneBotSpeed, yourZoneBotAcceleration, yourZoneBotDistance, currentZoneID, false, false);
  AddRequestToYourZoneTable(request);
  // Case: If condition true then make additional bot transfer request of zone transfer for another diverge lane
  if (zoneType == "diverge") {
    RequestToYourZoneModel requestDiverge = IntimationBotTransfer(myZoneBotPosition, myZoneBotSpeed, !virtualZoneMatrix[1][myZoneBotPosition], yourZoneBotSpeed, yourZoneBotAcceleration, yourZoneBotDistance, currentZoneID, false, true);
    AddRequestToYourZoneTable(requestDiverge);
  }
  // Case: If condition true then make additional mirror bot transfer request for another merge lane
  if (zoneType == "mergeLane") {
    RequestToYourZoneModel requestDiverge = IntimationBotTransfer(myZoneBotPosition, myZoneBotSpeed, !virtualZoneMatrix[1][myZoneBotPosition], yourZoneBotSpeed, yourZoneBotAcceleration, yourZoneBotDistance, currentZoneID, true, false);
    AddRequestToYourZoneTable(requestDiverge);
  }
}

// Generates length of tail because of bot in MyZone.
int TailRegulator(int myZoneBotPosition, float myZoneBotSpeed, float yourZoneBotSpeed) {
  // Case handling, if bot is not present in the zone then return 0 length
  if (myZoneBotPosition == -1) {
    return 0;
  }

  float relativeSpeed = myZoneBotSpeed - yourZoneBotSpeed;
  float relativeDistance = (relativeSpeed * relativeSpeed)/(2 * ACCELERATION);
  float dynamicSafetyGap = minimumSafetyGap;
  
  // Case handling, if the bot behind is moving faster than extend the tail
  if (relativeSpeed < 0) {
    dynamicSafetyGap += relativeDistance;
  }
  
  int tailLength = ceil(minimumSafetyGap/INTER_COLUMN_PITCH);
  return tailLength;
}

// Generates length of active tail in MyZone because of bot in MyZone and length of tail for the zone behind because of bot in MyZone and incoming tails from ahead zones
pair<int, int> TailSegmentor(int myZoneTailLength, int myZoneBotPosition, int yourZoneBotMyZoneTailActiveLength_A, int yourZoneBotMyZoneTailActiveLength_B) {
  pair<int, int> myPair = {0, 0};
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

// Generates the length of active tail in MyZone and recalculate the length of tail for the zone behind because of the bot in MyZone.
pair<int, int> TailMirrorer(int myZoneMirrorBotPosition, float myZoneMirrorBotSpeed, float yourZoneBotSpeed, int yourZoneTailLength) {
  // Case handling, if mirror bot is not present in the zone then return 0 active my zone mirror length and same yourZoneTailLength
  pair<int, int> myPair = {0, 0};
  if (myZoneMirrorBotPosition == -1) {
    myPair.second = yourZoneTailLength;
    return myPair;
  }

  float relativeSpeed = myZoneMirrorBotSpeed - yourZoneBotSpeed;
  float relativeDistance = (relativeSpeed * relativeSpeed)/(2 * ACCELERATION);
  float dynamicSafetyGap = minimumSafetyGap;
  
  // Case handling, if the bot behind is moving faster than extend the tail
  if (relativeSpeed < 0) {
    dynamicSafetyGap += relativeDistance;
  }
  
  int tailLength = ceil(dynamicSafetyGap/INTER_COLUMN_PITCH);
  
  int myZoneMirrorTailActiveLength = 0;
  if (myZoneMirrorBotPosition != -1) {
    yourZoneTailLength = max(tailLength - myZoneMirrorBotPosition, yourZoneTailLength);
    myZoneMirrorTailActiveLength = min(myZoneMirrorBotPosition, tailLength);
  }
  
  myPair.first = myZoneMirrorTailActiveLength;
  myPair.second = yourZoneTailLength;
	return myPair;
}

// Turns ON/OFF the relevant amount of traffic indicators
void TailSuperimposer(int myZoneBotPosition, int myZoneTailActiveLength, int myZoneMirrorBotPosition, int myZoneMirrorTailActiveLength, int yourZoneBotMyZoneTailActiveLength_A, int yourZoneBotMyZoneTailActiveLength_B) {
  // 'superimposedActiveZone' is the union of all the input active tail lengths and generates a snapshot of what pattern traffic indicators must turn ON and OFF in MyZone
  bool superimposedActiveZone[columnCount] = {0};
  
  if (myZoneBotPosition != -1) {
    for(int i = myZoneBotPosition - 1 ; i > myZoneBotPosition - 1 - myZoneTailActiveLength ; i--) {
      superimposedActiveZone[i] = 1;
    }
  }
  
  if (zoneType == "mergeLane") {
    if (myZoneMirrorBotPosition != -1) {
      for(int i = myZoneMirrorBotPosition - 1 ; i > myZoneMirrorBotPosition - 1 - myZoneMirrorTailActiveLength ; i--) {
        superimposedActiveZone[i] = 1;
      }
    }
  }

  if (zoneType == "diverge") {
    yourZoneBotMyZoneTailActiveLength_A = max(yourZoneBotMyZoneTailActiveLength_A, yourZoneBotMyZoneTailActiveLength_B);
  }
  
  if (yourZoneBotMyZoneTailActiveLength_A > columnCount) {
    for(int i = 0 ; i < columnCount ; i++) {
      superimposedActiveZone[i] = 1;
    }
  } else {
    for(int i = columnCount - 1 ; i > columnCount - 1 - yourZoneBotMyZoneTailActiveLength_A ; i--) {
      superimposedActiveZone[i] = 1;
    }
  }

  if (intimationTailTest) {
    // add_log("MyZoneBotPosition: " + String(myZoneBotPosition) + " MyZoneTailACtiveLength: " + String(myZoneTailActiveLength) + " YourZoneBotMyZoneTailActiveLength_A: " + String(yourZoneBotMyZoneTailActiveLength_A));
    intimationTailTest = false;
  }

  for(int i = 0 ; i < columnCount ; i++) {
    if (i <= endStationColumn && i >= startStationColumn) {
      superimposedActiveZone[i] = 0;
    }
  }
  
  for (int i = 0 ; i < columnCount ; i++) {
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

RequestToYourZoneModel IntimateHighFrequencyNoiseError(int column) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0xC;
  requestToYourZone.data1 = column;
  
  return requestToYourZone;
}

RequestToYourZoneModel IntimateLowFrequencyNoiseError(int column) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0xD;
  requestToYourZone.data1 = column;
  
  return requestToYourZone;
}

RequestToYourZoneModel IntimateUnexpectedBotError(int column) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0xE;
  requestToYourZone.data1 = column;
  
  return requestToYourZone;
}

void ReadTransition(int index) {
  if (!digitalRead(botIndicatorPins[index]) && !highDetectedStatus[index]) {
    if (millis() - lowStartTime[index] < 1000 && highFrequencyNoiseNotReported[index]) {
      // add_log("High Frequency Noise - Low Time is less than 1 second @ column: " + String(index));
      highFrequencyNoiseNotReported[index] = false;
      // RequestToYourZoneModel errorRequest = IntimateHighFrequencyNoiseError(index);
      // AddRequestToYourZoneTable(errorRequest);
    }
    // if (index == 5) {
    //   add_log("High Start Time: " + String(millis()));
    // }
    highStartTime[index] = millis();
    highDetectedStatus[index] = true;
  }

  if (highDetectedStatus[index] && !expectedStatus[index]) {
    if (millis() - highStartTime[index] > 300 && lowFrequencyNoiseNotReported[index]) {
      // add_log("Might Be Bot Stopped or Low Frequency Noise @ column: " + String(index));
      lowFrequencyNoiseNotReported[index] = false;
      // RequestToYourZoneModel errorRequest = IntimateLowFrequencyNoiseError(index);
      // AddRequestToYourZoneTable(errorRequest);
    }
  }

  if (digitalRead(botIndicatorPins[index]) && highDetectedStatus[index]) {
    if (highStartTime[index] - lowStartTime[index] > 1000 && millis() - highStartTime[index] < 300 && expectedStatus[index]) {
      botIsPresent[index] = true;
    }
    if (highStartTime[index] - lowStartTime[index] > 1000 && millis() - highStartTime[index] < 300 && !expectedStatus[index]) {
      // add_log("High Frequency Noise - Bot wasn't expected @ column: " + String(index));
      highFrequencyNoiseNotReported[index] = false;
      // RequestToYourZoneModel errorRequest = IntimateUnexpectedBotError(index);
      // AddRequestToYourZoneTable(errorRequest);
    }
    lowStartTime[index] = millis();
    highDetectedStatus[index] = false;
    
  }
}

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

void HandleExpectedBotRequest(RequestFromYourZoneModel request) {
  int expected = (int) request.data1;

  if (expected > columnCount) {
    for(int i = 0 ; i < columnCount ; i++) {
      expectedStatus[i] = true;
    }
  } else {
    for(int i = 0 ; i < expected ; i++) {
      expectedStatus[i] = true;
    }
  }

  RequestToYourZoneModel ackRequest;
  ackRequest.requestId = request.requestId;
  ackRequest.receiverId = request.senderId;
  ackRequest.originalSenderId = currentZoneID;
  ackRequest.requestType = 0x0;
  ackRequest.CommunicatorStatus = "COMPLETE";
  
  AddRequestToYourZoneTable(ackRequest);
}

void UpdateExpectedStatus(int index) {
  for(int i = 0 ; i <= index ; i++) {
    expectedStatus[i] = false;
  }

  if (columnCount - index - 1 < expectedSafetyRange) {
    RequestToYourZoneModel request = IntimateExpectedBotRequest(expectedSafetyRange - (columnCount - index - 1));
    AddRequestToYourZoneTable(request);
    for (int i = index + 1 ; i < columnCount ; i++) {
      expectedStatus[i] = true;
    }
  } else {
    for (int i = index + 1 ; i < index + 1 + expectedSafetyRange ; i++) {
      expectedStatus[i] = true;
    }
  }
}

void RESOLVE_REQUEST(void *pvParameters) {
  while (true) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    CopyNewRequestsToTemp();
    ResolveRequestToYourZoneTempTable();
    // RecieveRequest();
    CopyNewRequestsToFromTemp();
    ResolveRequestFromYourZoneTempTable();
    UpdateRealListFromTemp();
    UpdateRealListFromFromTemp();
  }
}


void setup() {
  // ----------------------------------------------------- Real Zone -----------------------------------------------------
  delay(1000);
  Serial.begin(115200);
  requestToTableMutex = xSemaphoreCreateMutex();
  if (requestToTableMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1); // Stop if mutex creation fails
  }
  requestFromTableMutex = xSemaphoreCreateMutex();
  if (requestFromTableMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1); // Stop if mutex creation fails
  }
  WiFiConfig();
  xTaskCreatePinnedToCore(
    HTTP_DEBUG_LOGGER,
    "debug_logging",
    10000,
    NULL,
    1,
    &httpDebugLog,
    0);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOOT_BUTTON_PIN), ButtonISR, FALLING);
  BoardConfig();
  NeoPixelConfig();
  CANconfig();
  for(int i = 0 ; i < 20 ; i++) {
    pinMode(botIndicatorPins[i], INPUT);
  }

  // Initialising RequestToYourZoneTable
  for (int i = 0; i < 20; ++i) {
    requestToYourZoneTable[i] = RequestToYourZoneModel();
    tempRequestToYourZoneTable[i] = RequestToYourZoneModel();
  }

  // Initialising RequestFromYourZoneTable
  for (int i = 0; i < 20; ++i) {
    requestFromYourZoneTable[i] = RequestFromYourZoneModel();
    tempRequestFromYourZoneTable[i] = RequestFromYourZoneModel();
  }

  // Turn ON column indicators
  for (int i = 0; i < columnCount ; i++) {
    SetVirtualColumnValue(i, 0, 0, 255);
    zoneMatrix.setPixelColor(i, zoneMatrix.Color(0, 0, 255));
  }
  zoneMatrix.show();
  Serial.println("Zone: Z: " + String(currentZoneID) + " Code: " + CODE_ID);
  Serial.println(__FILE__);
  debugLoggingString = "Z" + String(currentZoneID) + " " + CODE_ID + ": ";
  xTaskCreatePinnedToCore(
    RESOLVE_REQUEST,
    "resolve_request",
    4096,
    NULL,
    1,
    &resolveRequestToTask,
    0
  );
  // ---------------------------------------------------------------------------------------------------------------------

  // -------------------------------------------- Virtual Bot Initialization ---------------------------------------------
  // ---------------------------------------------------------------------------------------------------------------------
}

void loop() {
  loopStartTime = micros();
  if (otaFlag) {
    Serial.println("Taking OTA!");
    WiFiConfig();
    CheckForOTAUpdate();
  }
  // ----------------------------------------------- Virtual bot and virtual pin operations and manipulations -----------------------------------------------
  // --------------------------------------------------------------------------------------------------------------------------------------------------------

  // ----------------------------------------------------------------- Real zone operations -----------------------------------------------------------------
  
  bool botDetectedOnNewColumn = BotDetection();
  // --------------------------------------------------------------------------------------------------------------------------------------------------------
  PerformanceMonitor();
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

// Configuration function to read non-volatile memory of EEPROM
void BoardConfig() {
  preferences.begin("board-config", false);
  zoneType = preferences.getString("ZoneType", "regular");      // Default to "regular" if not set
  currentZoneID = preferences.getInt("CurrentZoneID", -1);      // Default to -1 if not set
  previousZoneID_A = preferences.getInt("PrevZoneID_A", -1);      // Default to -1 if not set
  previousZoneID_B = preferences.getInt("PrevZoneID_B", -1);      // Default to -1 if not set
  nextZoneID_A = preferences.getInt("NextZoneID_A", -1);      // Default to -1 if not set
  nextZoneID_B = preferences.getInt("NextZoneID_B", -1);      // Default to -1 if not set
  columnCount = preferences.getInt("ColumnCount", 0);           // Default to 0 if not set
  startStationColumn = preferences.getInt("StartStatCol", -1);
  endStationColumn = preferences.getInt("EndStatCol", -1);
  minimumSafetyGap = preferences.getFloat("MinSafGap", 3);
  slaveZoneID = preferences.getInt("SlaveId", -1);
  expectedSafetyRange = preferences.getInt("ExpGap", -1);

  if (currentZoneID == -1) {
    // add_log("Current Zone ID is not set!");
    while (true);
  }

  Serial.println("Board Type: " + zoneType + ", Column Count: " + String(columnCount) + ", Current Zone ID: Z" + String(currentZoneID) + ", Previous Zone ID: Z" + String(previousZoneID_A) + ", Next Zone ID: Z" + String(nextZoneID_A) + ", SlaveId: Z" + String(slaveZoneID) + ", Minimum Safety Gap (m): " + String(minimumSafetyGap, 3) + " and Expected Status Gap: " + String(expectedSafetyRange));

  // add_log("Segment Board Detected: " + String(CODE_ID));
  // add_log("Board Type: " + zoneType + ", Column Count: " + String(columnCount) + ", Current Zone ID: Z" + String(currentZoneID) + ", Previous Zone ID: Z" + String(previousZoneID_A) + " and Next Zone ID: Z" + String(nextZoneID_A));
  
  // Update strip with the correct columnCount
  zoneMatrix.updateLength(columnCount);
  zoneMatrix.begin();
}

// Set the values of LED column
// (r -> Bot Indicator, g -> Traffic Indicator, b -> Column Indicator)
void SetVirtualColumnValue(int i, uint8_t r, uint8_t g, uint8_t b) {
  virtualZoneMatrix[0][i] = (255 == r);
  virtualZoneMatrix[1][i] = (255 == g);
  virtualZoneMatrix[2][i] = (255 == b);
}

// Logger Function which will run as RTOS task
void HTTP_DEBUG_LOGGER(void* pvParameters) {
  while (true) {
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // if (cycleNumber%2 == 1) {
      if (debugLoggingString != "Z" + String(currentZoneID) + " " + CODE_ID + ": ") {
        int httpCode = -1;
        if (loggerFlag) {
          // WiFiConfig();
          httpDebugger.begin(HTTP_DEBUG_SERVER_URL);
          httpCode = httpDebugger.POST(debugLoggingString);
          // WiFi.disconnect(true);
        } else {
          httpCode = HTTP_CODE_OK;
        }
        if (httpCode == HTTP_CODE_OK) {
          debugLoggingString = "Z" + String(currentZoneID) + " " + CODE_ID + ": ";
        }
      }
    // }
  }
}

// Function to add logs into the log string
void add_log(String log) {
  debugLoggingString += " | " + log;
}

// Print the request from RequestToYourZoneTable at the given index
void PrintContentRequestToYourZoneTable(int index) {
  add_log("RequestToYourZoneTable(" + String(index) + ") - ");
  RequestToYourZoneModel request = requestToYourZoneTable[index];
  PrintRequestToYourZone(request);
}

// Print the request from RequestFromYourZoneTable at the given index
void PrintContentRequestFromYourZoneTable(int index) {
  add_log("RequestFromYourZoneTable(" + String(index) + ") - ");
  RequestFromYourZoneModel request = requestFromYourZoneTable[index];
  PrintRequestFromYourZone(request);
}

// Print the given RequestToYourZoneModel
void PrintRequestToYourZone(RequestToYourZoneModel request) {
  add_log("requestId: " + String(request.requestId));
  add_log("receiverId: " + String(request.receiverId));
  add_log("originalSenderId: " + String(request.originalSenderId));
  add_log("requestType: " + String(request.requestType));
  add_log("status: " + request.CommunicatorStatus);
  add_log("CANStatus: " + request.CANCommunicatorStatus);
  add_log("data1: " + String(request.data1));
  add_log("data2: " + String(request.data2));
  add_log("data3: " + String(request.data3));
  add_log("data4: " + String(request.data4));
  add_log("data5: " + String(request.data5));
}

// Print the given RequestFromYourZoneModel
void PrintRequestFromYourZone(RequestFromYourZoneModel request) {
  add_log("requestId: " + String(request.requestId));
  add_log("senderId: " + String(request.senderId));
  add_log("originalSenderId: " + String(request.originalSenderId));
  add_log("requestType: " + String(request.requestType));
  add_log("status: " + request.CommunicatorStatus);
  add_log("CANStatus: " + request.CANCommunicatorStatus);
  add_log("data1: " + String(request.data1));
  add_log("data2: " + String(request.data2));
  add_log("data3: " + String(request.data3));
  add_log("data4: " + String(request.data4));
  add_log("data5: " + String(request.data5));
}

// Calculate the time in seconds for a given speed, given distance and deceleration flag
float FindTimeToReachAtGivenDistanceAccordingToGivenSpeed(float distance, float speed, bool slowDown) {
  if (slowDown) {
    if (speed * speed - 2 * ACCELERATION * distance < 0) {
      return INT_MAX;
    } else {
      float timeToDec = (speed - sqrt(speed * speed - 2 * ACCELERATION * distance))/(ACCELERATION);
      return timeToDec;
    }
  } else {
    if (sqrt(speed * speed + 2 * ACCELERATION * distance) > MAX_SPEED) {
      float timeToAcc = (MAX_SPEED - MAX_SPEED)/ACCELERATION;
      float accelerationDistance = (MAX_SPEED * MAX_SPEED - speed * speed)/(2 * ACCELERATION);
      float distanceLeft = distance - accelerationDistance;
      float timeofConst = distanceLeft/MAX_SPEED;
      return (timeToAcc + timeofConst);
    } else {
      float timeToAcc = (sqrt(speed * speed + 2 * ACCELERATION * distance) - speed)/(ACCELERATION);
      return timeToAcc;
    }
  }
}

RequestToYourZoneModel IntimateLoopTime(float loopTime) {
  RequestToYourZoneModel requestToYourZone;
  requestToYourZone.requestId = requestId;
  requestId++;
  if (requestId == 255) requestId = 0;
  requestToYourZone.receiverId = slaveZoneID;
  requestToYourZone.originalSenderId = currentZoneID;
  requestToYourZone.requestType = 0x11;
  requestToYourZone.data1 = 0;
  requestToYourZone.data2 = loopTime;
  
  return requestToYourZone;
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
  if (numsOfIteration == 10000) {
    // add_log("Core 1 Loop Time (micros): " + String(loopTime));
    // add_log("RAM used (b): " + String(totalHeap - freeHeap));
    numsOfIteration = 0;
  }
  if (loopTime > 2100) {
    maxLoopTime = loopTime;
    add_log("Core 1 New Max. Loop Time (micros): " + String(maxLoopTime));
    // RequestToYourZoneModel errorRequest = IntimateLoopTime(loopTime/1000.0);
    // AddRequestToYourZoneTable(errorRequest);
  }
  if (totalHeap - freeHeap > maxMemoryUsed) {
    maxMemoryUsed = totalHeap - freeHeap;
    // add_log("New Max. Memory Usage (b): " + String(totalHeap - freeHeap));
  }
  numsOfIteration++;
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