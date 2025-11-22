GitHub Copilot: # WTM Column Detection - Comprehensive Test Plan

## Test Strategy
This test plan follows an atomic approach - testing individual functions and components in isolation before testing integrated behavior. Each test includes:
- **Objective**: What we're testing
- **Code Modifications**: What to comment/uncomment
- **Test Procedure**: Step-by-step actions
- **Expected Output**: What logs/behavior to expect
- **Pass Criteria**: How to determine success

---

## PHASE 1: BASIC SENSOR READING TESTS

### Test 1.1: Raw Sensor Reading
**Objective**: Verify TI and CI sensors are reading correctly

**Code Modifications**:
```cpp
// In loop(), comment out everything except:
void loop() {
  int trafficDetected = !digitalRead(TRAFFIC_INDICATOR_SENSOR);
  int columnDetected = !digitalRead(COLUMN_INDICATOR_SENSOR);
  String combo = String(trafficDetected) + String(columnDetected);
  
  add_log("TI=" + String(trafficDetected) + " CI=" + String(columnDetected) + " Combo=" + combo);
  delay(100); // Slow down for observation
}
```

**Test Procedure**:
1. Place bot in empty space (no columns) - Expected: "00"
2. Move bot to column with traffic - Expected: "11"
3. Move bot to column without traffic - Expected: "01"
4. Manually cover CI sensor only - Expected: "10"

**Expected Output**:
```
Combo=00
Combo=11
Combo=01
Combo=10
```

**Pass Criteria**: All four combinations (00, 01, 10, 11) correctly detected - Passed

---

### Test 1.2: State Transition Detection
**Objective**: Verify state machine transitions between BETWEEN_COLUMNS and IN_COLUMN

**Code Modifications**:
```cpp
// In loop(), add logging at state transitions:
switch (columnState) {
    case BETWEEN_COLUMNS:
      if (combo == "00") {
        // existing code
      } else {
        add_log("STATE CHANGE: BETWEEN_COLUMNS -> IN_COLUMN [" + combo + "]");
        // existing code
      }
      break;
      
    case IN_COLUMN:
      // existing code
      if (exitConfirmationCount >= EXIT_CONFIRMATION_THRESHOLD) {
        add_log("STATE CHANGE: IN_COLUMN -> BETWEEN_COLUMNS [10 consecutive 00s]");
        // existing code
      }
      break;
}
```

**Test Procedure**:
1. Start bot in empty space (00 zone)
2. Move slowly over a column
3. Exit column back to empty space

**Expected Output**:
```
STATE CHANGE: BETWEEN_COLUMNS -> IN_COLUMN [11]
Entering Column 1 [11]
... (frame counting)
STATE CHANGE: IN_COLUMN -> BETWEEN_COLUMNS [10 consecutive 00s]
```

**Pass Criteria**: Clean transitions with no state flickering - Passed

---

## PHASE 2: FRAME COUNTING TESTS

### Test 2.1: Frame Count Accumulation
**Objective**: Verify frame counts are accumulated correctly for each combo type

**Code Modifications**:
```cpp
// In IN_COLUMN case, add periodic logging:
case IN_COLUMN:
  totalFramesBetweenSpaces++;
  
  if (combo == "00") {
    frameCount00++;
  } else if (combo == "01") {
    frameCount01++;
  } else if (combo == "10") {
    frameCount10++;
  } else if (combo == "11") {
    frameCount11++;
  }
  
  // Add this logging every 10 frames
  if (totalFramesBetweenSpaces % 10 == 0) {
    add_log("Frames: 00=" + String(frameCount00) + " 01=" + String(frameCount01) + 
            " 10=" + String(frameCount10) + " 11=" + String(frameCount11));
  }
```

**Test Procedure**:
1. Move bot slowly over a column
2. Observe frame count increments
3. Note which combo has highest count

**Expected Output**:
```
Frames: 00=0 01=0 10=0 11=10
Frames: 00=0 01=1 10=0 11=19
Frames: 00=3 01=1 10=0 11=26
... (11 should be majority)
```

**Pass Criteria**: Majority frame count corresponds to actual column state - Passed

---

### Test 2.2: Exit Confirmation Counter
**Objective**: Verify 10 consecutive 00s are required for exit

**Code Modifications**:
```cpp
// In IN_COLUMN case:
if (combo == "00") {
  exitConfirmationCount++;
  add_log("Exit confirmation: " + String(exitConfirmationCount) + "/10");
  
  if (exitConfirmationCount >= EXIT_CONFIRMATION_THRESHOLD) {
    add_log("EXIT CONFIRMED!");
    // existing code
  }
} else {
  if (exitConfirmationCount > 0) {
    add_log("Exit confirmation RESET (was at " + String(exitConfirmationCount) + ")");
  }
  exitConfirmationCount = 0;
}
```

**Test Procedure**:
1. Move bot over column
2. As exiting, move VERY slowly to see 00s accumulate
3. Try to create interruption (non-00) before reaching 10

**Expected Output**:
```
Exit confirmation: 1/10
Exit confirmation: 2/10
...
Exit confirmation: 10/10
EXIT CONFIRMED!
```
OR if interrupted:
```
Exit confirmation: 5/10
Exit confirmation RESET (was at 5)
```

**Pass Criteria**: Exit only confirmed after uninterrupted 10 consecutive 00s - Passed

---

## PHASE 3: HEALTH CALCULATION TESTS

### Test 3.1: Majority Health Calculation
**Objective**: Verify majority health percentage calculation

**Code Modifications**:
```cpp
// Create a test harness function in loop():
void TestMajorityHealth() {
  // Test case 1: Perfect reading
  frameCount11 = 5; frameCount01 = 0; frameCount10 = 0;
  thresholdFrameCount = 5;
  int majorityCount = 5;
  HealthStatus health = CalculateMajorityHealth(majorityCount, thresholdFrameCount);
  add_log("Test 1 (5/5): " + HealthToString(health) + " - Expected: SAFE");
  
  // Test case 2: Warning threshold
  majorityCount = 4;
  health = CalculateMajorityHealth(majorityCount, thresholdFrameCount);
  add_log("Test 2 (4/5=80%): " + HealthToString(health) + " - Expected: SAFE");
  
  // Test case 3: Warning
  majorityCount = 3;
  health = CalculateMajorityHealth(majorityCount, thresholdFrameCount);
  add_log("Test 3 (3/5=60%): " + HealthToString(health) + " - Expected: WARNING");
  
  // Test case 4: Critical
  majorityCount = 2;
  health = CalculateMajorityHealth(majorityCount, thresholdFrameCount);
  add_log("Test 4 (2/5=40%): " + HealthToString(health) + " - Expected: CRITICAL");
  
  delay(10000); // Pause to read results
}

// Call in setup():
void setup() {
  // ... existing setup code
  TestMajorityHealth();
}
```

**Expected Output**:
```
Test 1 (5/5): SAFE - Expected: SAFE
Test 2 (4/5=80%): SAFE - Expected: SAFE
Test 3 (3/5=60%): WARNING - Expected: WARNING
Test 4 (2/5=40%): CRITICAL - Expected: CRITICAL
```

**Pass Criteria**: All test cases match expected health status - Passed

---

### Test 3.2: Minority Health Calculation
**Objective**: Verify minority (noise) health percentage calculation

**Code Modifications**:
```cpp
void TestMinorityHealth() {
  int totalFrames = 100;
  
  // Test case 1: Low noise (safe)
  int minorityTotal = 10; // 10% noise
  HealthStatus health = CalculateMinorityHealth(minorityTotal, totalFrames);
  add_log("Test 1 (10/100=10%): " + HealthToString(health) + " - Expected: SAFE");
  
  // Test case 2: Medium noise (warning)
  minorityTotal = 30; // 30% noise
  health = CalculateMinorityHealth(minorityTotal, totalFrames);
  add_log("Test 2 (30/100=30%): " + HealthToString(health) + " - Expected: WARNING");
  
  // Test case 3: High noise (critical)
  minorityTotal = 50; // 50% noise
  health = CalculateMinorityHealth(minorityTotal, totalFrames);
  add_log("Test 3 (50/100=50%): " + HealthToString(health) + " - Expected: CRITICAL");
  
  delay(10000);
}
```

**Expected Output**:
```
Test 1 (10/100=10%): SAFE - Expected: SAFE
Test 2 (30/100=30%): WARNING - Expected: WARNING
Test 3 (50/100=50%): CRITICAL - Expected: CRITICAL
```

**Pass Criteria**: All test cases match expected health status - Passed

---

### Test 3.3: Empty Space Health Calculation
**Objective**: Verify empty space noise detection

**Code Modifications**:
```cpp
void TestEmptySpaceHealth() {
  int totalFrames = 100;
  
  // Test case 1: Clean empty space
  int noiseFrames = 2; // 2% noise
  HealthStatus health = CalculateEmptySpaceHealth(noiseFrames, totalFrames);
  add_log("Test 1 (2/100=2%): " + HealthToString(health) + " - Expected: SAFE");
  
  // Test case 2: Some noise
  noiseFrames = 7; // 7% noise
  health = CalculateEmptySpaceHealth(noiseFrames, totalFrames);
  add_log("Test 2 (7/100=7%): " + HealthToString(health) + " - Expected: WARNING");
  
  // Test case 3: Excessive noise
  noiseFrames = 15; // 15% noise
  health = CalculateEmptySpaceHealth(noiseFrames, totalFrames);
  add_log("Test 3 (15/100=15%): " + HealthToString(health) + " - Expected: CRITICAL");
  
  delay(10000);
}
```

**Expected Output**:
```
Test 1 (2/100=2%): SAFE - Expected: SAFE
Test 2 (7/100=7%): WARNING - Expected: WARNING
Test 3 (15/100=15%): CRITICAL - Expected: CRITICAL
```

**Pass Criteria**: All test cases match expected health status - Passed

---

## PHASE 4: NOISE FILTERING TESTS

### Test 4.1: Minimum Frame Threshold Filter
**Objective**: Verify fluke columns (< 3 frames) are rejected

**Code Modifications**:
```cpp
// In IN_COLUMN case, enhance logging:
if (exitConfirmationCount >= EXIT_CONFIRMATION_THRESHOLD) {
  add_log("Total frames: " + String(totalFramesBetweenSpaces) + 
          " | Threshold: " + String(MIN_COLUMN_FRAMES_THRESHOLD));
          
  if (totalFramesBetweenSpaces < MIN_COLUMN_FRAMES_THRESHOLD) {
    add_log("REJECTION TEST: Frames too low, should reject");
    // existing rejection code
  } else {
    add_log("ACCEPTANCE TEST: Frames sufficient, should accept");
    // existing acceptance code
  }
}
```

**Test Procedure**:
1. Create artificial fluke by manually tapping sensor briefly
2. Ensure reading lasts < 3 frames
3. Observe rejection

**Expected Output**:
```
Total frames: 2 | Threshold: 3
REJECTION TEST: Frames too low, should reject
Rejected fluke column [11]: Only 2 frames (min: 3)
```

**Pass Criteria**: Columns with < 3 frames rejected; >= 3 frames accepted - Passed

---

### Test 4.2: Fluke Recovery Test
**Objective**: Verify empty space frame count is recovered after fluke

**Code Modifications**:
```cpp
// In BETWEEN_COLUMNS case:
} else {
  add_log("FLUKE DETECTION: Saving emptySpaceFrameCount=" + String(emptySpaceFrameCount));
  savedEmptySpaceFrameCount = emptySpaceFrameCount;
  // existing code
}

// In IN_COLUMN rejection:
if (totalFramesBetweenSpaces < MIN_COLUMN_FRAMES_THRESHOLD) {
  add_log("RECOVERY: Restoring " + String(savedEmptySpaceFrameCount) + 
          " frames + " + String(exitConfirmationCount) + " validation = " + 
          String(savedEmptySpaceFrameCount + exitConfirmationCount));
  // existing recovery code
}
```

**Test Procedure**:
1. Run bot in empty space for ~50 frames
2. Create brief fluke (< 3 frames)
3. Return to empty space
4. Verify frame count recovered

**Expected Output**:
```
Empty space: 50 frames accumulated
FLUKE DETECTION: Saving emptySpaceFrameCount=50
... fluke for 2 frames ...
RECOVERY: Restoring 50 frames + 10 validation = 60
Recovered empty space: 60 frames
```

**Pass Criteria**: Empty space count = original + validation frames (not reset to 0) - Passed

---

## PHASE 5: COLUMN DETECTION TESTS

### Test 5.1: Single Column Detection
**Objective**: Verify complete column detection cycle

**Test Procedure**:
1. Start bot in empty space
2. Move over ONE column slowly
3. Exit back to empty space
4. Observe complete cycle

**Expected Output**:
```
STATE CHANGE: BETWEEN_COLUMNS -> IN_COLUMN [11]
Entering Column 1 [11]
Exit confirmation: 1/10
...
Exit confirmation: 10/10
Col1[11]: Frames=35 (01:1,10:0,11:28,00:6) Maj=560% Min=2% Health:SAFE
Traffic Detected
```

**Pass Criteria**: - Passed
- Column number increments (totalColumnsDetected = 1)
- Majority code matches actual column state
- Health calculated correctly
- columnJustCompleted flag set

---

### Test 5.2: Multiple Columns Detection
**Objective**: Verify consecutive column detection

**Test Procedure**:
1. Move bot over 5 columns continuously
2. Observe each column logged separately
3. Check totalColumnsDetected increments

**Expected Output**:
```
Col1[11]: ... Health:SAFE
Col2[01]: ... Health:SAFE
Col3[11]: ... Health:WARNING
Col4[11]: ... Health:SAFE
Col5[01]: ... Health:SAFE
```

**Pass Criteria**: - Passed
- Each column detected and logged separately
- totalColumnsDetected = 5
- Empty spaces logged between columns

---

### Test 5.3: Column Indicator Malfunction Detection
**Objective**: Verify detection of '10' state (CI malfunction)

**Code Modifications**:
```cpp
// Manually create '10' scenario in test:
void TestCImalfunction() {
  detectedColumnCode = "10";
  columnJustCompleted = true;
  lastCompletedColumnCode = "10";
  
  // Trigger the decision logic:
  if (columnJustCompleted) {
    if (lastCompletedColumnCode == "10") {
      add_log("WARNING: Column Indicator malfunction detected! Reading '10' (CI=0, TI=1)");
    }
  }
}
```

**Expected Output**:
```
Col1[10]: Frames=... Health:...
WARNING: Column Indicator malfunction detected! Reading '10' (CI=0, TI=1)
```

**Pass Criteria**: Warning logged when '10' detected - Passed

---

## PHASE 6: EMPTY SPACE TESTS

### Test 6.1: Empty Space Frame Counting
**Objective**: Verify empty space frames counted correctly

**Code Modifications**:
```cpp
// In BETWEEN_COLUMNS case:
if (combo == "00") {
  emptySpaceFrameCount++;
  emptySpaceTotalFrameCount++;
  
  // Log every 10 frames
  if (emptySpaceFrameCount % 10 == 0) {
    add_log("Empty space progress: " + String(emptySpaceFrameCount) + " frames");
  }
}
```

**Test Procedure**:
1. Let bot travel in empty space for extended time
2. Observe frame count incrementing

**Expected Output**:
```
Empty space progress: 10 frames
Empty space progress: 20 frames
Empty space progress: 30 frames
...
```

**Pass Criteria**: Frame count increments correctly in 00 zones - Passed

---

### Test 6.2: Empty Space Health Monitoring
**Objective**: Verify empty space health calculation with noise

**Test Procedure**:
1. Run bot in empty space
2. Manually introduce brief noise (cover sensor momentarily)
3. Observe health calculation when entering next column

**Expected Output**:
```
Empty Space 1: Frames=45 (00:43, Noise:2) Health:SAFE
```
OR with more noise:
```
Empty Space 1: Frames=45 (00:38, Noise:7) Health:WARNING
```

**Pass Criteria**: Health reflects noise percentage correctly - Passed

---

### Test 6.3: Empty Space Validation Frames Inclusion
**Objective**: Verify 10 validation frames included in next empty space

**Code Modifications**:
```cpp
// After column completion:
emptySpaceFrameCount = EXIT_CONFIRMATION_THRESHOLD;
emptySpaceTotalFrameCount = EXIT_CONFIRMATION_THRESHOLD;
add_log("Empty space initialized with " + String(EXIT_CONFIRMATION_THRESHOLD) + " validation frames");
```

**Expected Output**:
```
Col1[11]: ... Health:SAFE
Empty space initialized with 10 validation frames
... continue in empty space ...
Empty Space 1: Frames=50 (00:50, Noise:0) Health:SAFE
```

**Pass Criteria**: Empty space count starts at 10 after column, not 0 - Passed

---

## PHASE 7: STATION SEGMENT VALIDATION TESTS

### Test 7.1: Segment Counter Accumulation
**Objective**: Verify unique columns and empty spaces counted per segment

**Code Modifications**:
```cpp
// Add logging when counters increment:
case IN_COLUMN:
  if (exitConfirmationCount >= EXIT_CONFIRMATION_THRESHOLD) {
    if (totalFramesBetweenSpaces >= MIN_COLUMN_FRAMES_THRESHOLD) {
      uniqueColumnsDetectedInSegment++;
      add_log("Segment column count: " + String(uniqueColumnsDetectedInSegment));
      // existing code
    }
  }
  
case BETWEEN_COLUMNS:
  if (currentColumnFrameCount > 0) {
    emptySpacesDetectedInSegment++;
    add_log("Segment empty space count: " + String(emptySpacesDetectedInSegment));
  }
```

**Test Procedure**:
1. Start at station D2
2. Move to station D3 (should have 10 columns, 11 empty spaces)
3. Observe counters incrementing

**Expected Output**:
```
Segment column count: 1
Segment empty space count: 1
Segment column count: 2
Segment empty space count: 2
...
Segment column count: 10
Segment empty space count: 11
```

**Pass Criteria**: Counters match expected values before reaching next station

---

### Test 7.2: Station-to-Station Validation
**Objective**: Verify segment validation against expected map

**Test Procedure**:
1. Start at D2, move to D3
2. Ensure exactly 10 columns and 11 empty spaces detected
3. Observe validation log

**Expected Output**:
```
Current station: D3
Segment Analysis :D2-D3
Unique Columns: 10 (Expected: 10)
Empty Spaces: 11 (Expected: 11)
Status: SEGMENT OK - All columns and spaces match expected values
```

**Pass Criteria**: Validation passes when actual matches expected

---

### Test 7.3: Segment Mismatch Detection
**Objective**: Verify warnings when actual doesn't match expected

**Code Modifications**:
```cpp
// Artificially modify counter to create mismatch:
void TestSegmentMismatch() {
  previousStation = "D2";
  currentStation = "D3";
  uniqueColumnsDetectedInSegment = 8; // Should be 10
  emptySpacesDetectedInSegment = 11;
  
  ValidateStationSegment(previousStation, currentStation, 
                        uniqueColumnsDetectedInSegment, emptySpacesDetectedInSegment);
}
```

**Expected Output**:
```
Segment Analysis :D2-D3
Unique Columns: 8 (Expected: 10)
Empty Spaces: 11 (Expected: 11)
WARNING: Column count mismatch! Difference: -2
Possible Issue: Missing columns detected or bot skipped columns
```

**Pass Criteria**: Warnings generated for mismatches with helpful diagnostics

---

## PHASE 8: INTEGRATION TESTS

### Test 8.1: Full Track Run
**Objective**: Complete end-to-end test with all features enabled

**Test Procedure**:
1. Enable ALL code (no modifications)
2. Run bot from D1 to D18 (full loop)
3. Collect all logs

**Expected Output**:
```
Current station: D1
... multiple columns and empty spaces ...
Current station: D2
Segment Analysis :D1-D2
... validation ...
Current station: D3
Segment Analysis :D2-D3
... continue through all stations ...
Current station: D18
```

**Pass Criteria**:
- All stations detected correctly
- All segments validated
- No unexpected errors
- Health monitoring active throughout

---

### Test 8.2: Decision Making Test
**Objective**: Verify columnJustCompleted flag enables correct decisions

**Code Modifications**:
```cpp
// In the decision block after column completion:
if (columnJustCompleted) {
  add_log("DECISION POINT: Column " + String(totalColumnsDetected) + 
          " [" + lastCompletedColumnCode + "] Health:" + HealthToString(lastCompletedColumnHealth));
  
  if (lastCompletedColumnCode == "11") {
    add_log("ACTION: Traffic detected - triggering diverter");
    // Your actual diverter logic here
  } else if (lastCompletedColumnCode == "01") {
    add_log("ACTION: No traffic - continuing normal operation");
  }
  
  if (lastCompletedColumnHealth == CRITICAL) {
    add_log("ACTION: Critical health - alerting maintenance");
  }
}
```

**Expected Output**:
```
Col5[11]: ... Health:SAFE
DECISION POINT: Column 5 [11] Health:SAFE
ACTION: Traffic detected - triggering diverter
Traffic Detected
```

**Pass Criteria**: Correct actions taken based on column code and health

---

### Test 8.3: Stress Test - High Speed
**Objective**: Verify system works at maximum bot speed (2 m/s)

**Test Procedure**:
1. Set bot speed to S2 (1070 RPM = 2 m/s)
2. Run through multiple columns
3. Verify no columns missed or misread

**Expected Output**:
```
Col1[11]: Frames=6 (01:0,10:0,11:5,00:1) Maj=100% Min=0% Health:SAFE
Col2[01]: Frames=5 (01:4,10:0,11:0,00:1) Maj=80% Min=0% Health:SAFE
Col3[11]: Frames=7 (01:0,10:0,11:6,00:1) Maj=120% Min=0% Health:SAFE
```

**Pass Criteria**:
- All columns detected (no skips)
- Frame counts reasonable for high speed (3-8 frames)
- No critical health warnings

---

### Test 8.4: Stress Test - Noisy Environment
**Objective**: Verify system handles electrical noise gracefully

**Test Procedure**:
1. Introduce electromagnetic interference (e.g., nearby motor)
2. Run through columns
3. Observe health warnings and noise filtering

**Expected Output**:
```
Col1[11]: Frames=25 (01:2,10:1,11:18,00:4) Maj=360% Min=12% Health:SAFE
Empty Space 1: Frames=55 (00:48, Noise:7) Health:WARNING
Col2[01]: Frames=30 (01:20,10:3,11:2,00:5) Maj=400% Min=16% Health:WARNING
```

**Pass Criteria**:
- Majority still correctly identified despite noise
- Health warnings generated appropriately
- No false column rejections

---

## TEST EXECUTION CHECKLIST

For each test, verify:
- [ ] Code modifications applied correctly
- [ ] Bot placed in correct starting position
- [ ] Logs captured (serial monitor or HTTP logger)
- [ ] Expected output matches actual output
- [ ] Pass criteria met
- [ ] Any failures documented with details

---

## TROUBLESHOOTING GUIDE

### Issue: State machine stuck in one state
**Check**:
- Sensor readings are updating
- EXIT_CONFIRMATION_THRESHOLD reached
- No infinite loops in state logic

### Issue: Frame counts not incrementing
**Check**:
- Sensors reading correctly (Test 1.1)
- Loop() executing (add millis() logging)
- Variables not being reset prematurely

### Issue: Health always CRITICAL
**Check**:
- thresholdFrameCount initialized correctly
- Majority count calculation
- Percentage thresholds (80%, 60%, etc.)

### Issue: Columns not detected
**Check**:
- MIN_COLUMN_FRAMES_THRESHOLD not too high
- Exit confirmation working (Test 2.2)
- Bot speed appropriate for frame capture

---

## SUCCESS CRITERIA SUMMARY

**Phase 1**: ✓ All sensor combinations readable  
**Phase 2**: ✓ All frame types counted correctly  
**Phase 3**: ✓ Health calculations accurate  
**Phase 4**: ✓ Noise filtering effective  
**Phase 5**: ✓ Columns detected reliably  
**Phase 6**: ✓ Empty spaces monitored  
**Phase 7**: ✓ Segment validation working  
**Phase 8**: ✓ Full integration successful  

---

**End of Test Plan**