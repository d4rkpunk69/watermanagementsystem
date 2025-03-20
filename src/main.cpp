#include "pinconfig.h"
// ============= Function Prototypes =============
void loraTask(void *parameter);
void displayTask(void *parameter);
void sensorTask(void *parameter);
void controlTask(void *parameter);
uint8_t calculateChecksum(void* packet, size_t size);
void sendCombinedTankData();
void sendAcknowledgement();
float readUltrasonicDistance(int index);
float gradualAdjustDistance(float currentLevel, float targetLevel);
float calculateWaterLevel(float distance);
void selectWaterSource();
void confirmWaterSource();
void activateWaterSource();
void deactivateWaterSource();
void updateDisplay();
void initSensors();
float getStableDistance(int index);
void showMessage(const String& msg, int delayTime);
void showLoadingScreen(int progress, String status);
void handleManualMode(uint8_t command);
void handleAutoMode();
void checkSafetyConditions();

// Auto mode condition checks
bool checkCaseA();
bool checkCaseB();
bool checkCaseC();
bool checkCaseD();
bool checkCaseE();

// Manual mode condition checks
bool checkCaseF();
bool checkCaseG();
bool checkCaseH();
bool checkCaseI();
bool checkCaseJ();

void pumpOn();
void pumpOff();

// ============= Setup =============
void setup() {
    heltec_ve(true);
    heltec_setup();
    Serial.begin(115200);

    showLoadingScreen(0, "Starting...");
    heltec_delay(500);
    
    initSensors(); 
    showLoadingScreen(30, "Sensors Initialized");
    
    pinMode(RAIN_SOLENOID_PIN, OUTPUT);
    pinMode(DEEPWELL_SOLENOID_PIN, OUTPUT);
    pinMode(PUMP_RELAY_PIN, OUTPUT);
    digitalWrite(RAIN_SOLENOID_PIN, LOW);
    digitalWrite(DEEPWELL_SOLENOID_PIN, LOW);
    digitalWrite(PUMP_RELAY_PIN, LOW);
    
    tankDataMutex = xSemaphoreCreateMutex();
    
    showLoadingScreen(60, "Initializing LoRa...");
    if(radio.begin(LORA_FREQUENCY) == RADIOLIB_ERR_NONE) {
        radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
        radio.setBandwidth(LORA_BANDWIDTH);
        radio.setCodingRate(LORA_CODING_RATE);
        radio.setSyncWord(LORA_SYNC_WORD);
        showLoadingScreen(80, "LoRa Ready");
    } else {
        showMessage("LoRa Init Failed", 0);
        while(1) { heltec_delay(1000); }
    }
    
    showLoadingScreen(100, "System Ready");
    heltec_delay(500);
    
    // Task Creation with Optimized Priorities
    xTaskCreatePinnedToCore(loraTask, "LoRaTask", 4096, NULL, 1, &loraTaskHandle, 0);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 2, &displayTaskHandle, 0);
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 3, &sensorTaskHandle, 1);
    xTaskCreatePinnedToCore(controlTask, "ControlTask", 2048, NULL, 1, &controlTaskHandle, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ============= Sensor Tasks =============
void sensorTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(250);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(1) {
      for (int i = 0; i < NUM_TANKS; i++) {
          float measuredDistance = getStableDistance(i);
          float correctedDistance = measuredDistance - SENSOR_OFFSET_CM; // Adjust for offset
          
          // Ensure correctedDistance is within valid range
          if (correctedDistance < 0) correctedDistance = 0;  // Prevent negative values
          if (correctedDistance > TANK_HEIGHTS_CM[i]) correctedDistance = TANK_HEIGHTS_CM[i];

          float targetWaterLevel = ((TANK_HEIGHTS_CM[i] - correctedDistance) / TANK_HEIGHTS_CM[i]) * 100;
          
          // Apply gradual adjustment
          float adjustedWaterLevel = gradualAdjustDistance(tanks[i].waterLevel, targetWaterLevel);
          
          if(xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(100))) {
              tanks[i].distance = correctedDistance;
              tanks[i].waterLevel = adjustedWaterLevel;
              tanks[i].lastUpdate = millis();
              tanks[i].valid = true;
              xSemaphoreGive(tankDataMutex);
          }
          
          // Small delay between readings to prevent interference
          vTaskDelay(pdMS_TO_TICKS(50));
      }
      
      uint32_t execTime = millis() - (xLastWakeTime * portTICK_PERIOD_MS);
      TickType_t xAdjust = pdMS_TO_TICKS(execTime < 250 ? 250 - execTime : 0);
      vTaskDelayUntil(&xLastWakeTime, xFrequency + xAdjust);
  }
}
/*
void sensorTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(250);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        for (int i = 0; i < NUM_TANKS; i++) {
            float distance = getStableDistance(i);
            float waterLevel = ((TANK_HEIGHT_CM - distance)/TANK_HEIGHT_CM) * 100;
            
            if(xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(100))) {
                tanks[i].distance = distance;
                tanks[i].waterLevel = waterLevel;
                tanks[i].lastUpdate = millis();
                tanks[i].valid = true;
                xSemaphoreGive(tankDataMutex);
            }
            
            // Small delay between readings to prevent interference
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        uint32_t execTime = millis() - (xLastWakeTime * portTICK_PERIOD_MS);
//         TickType_t xAdjust = pdMS_TO_TICKS(execTime < 250 ? 250 - execTime : 0);
//         vTaskDelayUntil(&xLastWakeTime, xFrequency + xAdjust);
//     }
// }
*/
// ============= LoRa Communication Task =============
void loraTask(void *parameter) {
  const TickType_t xMinDelay = pdMS_TO_TICKS(TX_INTERVAL_MIN);
  const TickType_t xMaxDelay = pdMS_TO_TICKS(TX_INTERVAL_MAX);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long lastSendTime = 0;

  while(1) {
      uint8_t dataBuffer[1];  // Expecting a single-byte command
      int packetLength = sizeof(dataBuffer);

      // Receive data
      int state = radio.receive(dataBuffer, packetLength);

      if (state == RADIOLIB_ERR_NONE) {
          uint8_t receivedCommand = dataBuffer[0];

          Serial.print("Received Control Command: ");
          Serial.println(receivedCommand);

          // Validate and handle command
          if (receivedCommand == 0 || receivedCommand == 11 || receivedCommand == 12) {
              Serial.println("✔ Valid Command Received!");

              if (xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(100))) {
                  handleManualMode(receivedCommand);
                  xSemaphoreGive(tankDataMutex);
              }

              // Send Acknowledgment (ACK)
              sendAcknowledgement();
              Serial.println("ACK sent.");
          } else {
              Serial.println("❌ Invalid Command Received!");
          }
      } else {
          Serial.println("❌ No valid packet received.");
      }

      // Periodic transmission of tank data
      if (millis() - lastSendTime > 10000) {
          sendCombinedTankData();
          lastSendTime = millis();
      }

      // Delay before next loop iteration
      TickType_t xDelay = xMinDelay + (rand() % (xMaxDelay - xMinDelay));
      vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

// ============= Control Task =============
void controlTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        if(xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(50))) {
            // Always check safety conditions first
            checkSafetyConditions();
            
            if(currentMode == MODE_AUTO) {
                handleAutoMode();
            } else if(currentMode == MODE_MANUAL) {
                // In manual mode, check specific conditions
                if(manualSource == SOURCE_DEEPWELL && checkCaseI()) {
                    // Case I: Turn off pump if deepwell is nearly empty
                    if(!pumpInSafetyMode) {
                        pumpInSafetyMode = true;
                        pumpSafetyTimer = millis();
                        currentStatus = STATUS_SAFETY_SHUTDOWN;
                        Serial.println("Case I: Deepwell nearly empty, entering safety shutdown");
                    }
                } 
                else if(manualSource == SOURCE_RAINWATER && checkCaseJ()) {
                    // Case J: Turn off pump if rainwater is nearly empty
                    if(!pumpInSafetyMode) {
                        pumpInSafetyMode = true;
                        pumpSafetyTimer = millis();
                        currentStatus = STATUS_SAFETY_SHUTDOWN;
                        Serial.println("Case J: Rainwater nearly empty, entering safety shutdown");
                    }
                }
                else if(checkCaseH()) {
                    // Case H: Turn off pump if main tank is full
                    deactivateWaterSource();
                    currentStatus = STATUS_DEFAULT;
                    Serial.println("Case H: Main tank full, stopping pump");
                }
            }
            
            xSemaphoreGive(tankDataMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============= Display Task =============
void displayTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        if(xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(50))) {
            updateDisplay();
            xSemaphoreGive(tankDataMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============= Sensor Functions =============
void initSensors() {
    // Initialize main tank sensor
    sensors[0].trigPin = MAIN_TRIG_PIN;
    sensors[0].echoPin = MAIN_ECHO_PIN;
    sensors[0].lastValidDistance = 0;
    
    // Initialize deepwell sensor
    sensors[1].trigPin = DEEP_TRIG_PIN;
    sensors[1].echoPin = DEEP_ECHO_PIN; // Fixed from potential bug
    sensors[1].lastValidDistance = 0;
    
    // Initialize rainwater sensor
    sensors[2].trigPin = RAIN_TRIG_PIN;
    sensors[2].echoPin = RAIN_ECHO_PIN;
    sensors[2].lastValidDistance = 0;
    
    // Configure all sensors
    for(int i = 0; i < NUM_TANKS; i++) {
        pinMode(sensors[i].trigPin, OUTPUT);
        pinMode(sensors[i].echoPin, INPUT_PULLUP);
        digitalWrite(sensors[i].trigPin, LOW);
    }
    
    delayMicroseconds(5000);
}

float getStableDistance(int index) {
    float readings[SAMPLES];
    int validReadings = 0;
    float validValues[SAMPLES];
    
    // Take multiple samples with adequate delay
    for(int i = 0; i < SAMPLES; i++) {
        readings[i] = readUltrasonicDistance(index);
        
        // Only include readings outside the blind zone
        if(readings[i] >= BLIND_DISTANCE_CM && readings[i] <= TANK_HEIGHT_CM) {
            validValues[validReadings] = readings[i];
            validReadings++;
        }
        
        delayMicroseconds(1000); // 1ms delay between readings
    }
    
    // If no valid readings, return the last valid distance
    if(validReadings == 0) {
        return sensors[index].lastValidDistance;
    }
    
    // Sort only the valid readings
    for(int i = 0; i < validReadings-1; i++) {
        for(int j = 0; j < validReadings-i-1; j++) {
            if(validValues[j] > validValues[j+1]) {
                float temp = validValues[j];
                validValues[j] = validValues[j+1];
                validValues[j+1] = temp;
            }
        }
    }
    
    // Return the median of valid readings
    float median = validValues[validReadings/2];
    sensors[index].lastValidDistance = median;
    return median;
}

float readUltrasonicDistance(int index) {
    // Ensure trigger pin starts LOW
    digitalWrite(sensors[index].trigPin, LOW);
    delayMicroseconds(5);
    
    // Send 10µs pulse to trigger the sensor
    digitalWrite(sensors[index].trigPin, HIGH);
    delayMicroseconds(15);
    digitalWrite(sensors[index].trigPin, LOW);
    
    // Measure the duration of the echo pulse
    long duration = pulseIn(sensors[index].echoPin, HIGH, TIMEOUT_SENSOR);
    
    // Calculate distance
    float distance = duration * 0.034 / 2;
    
    return distance;
}

float gradualAdjustDistance(float currentLevel, float targetLevel) {
  const float BIG_GAP = 10.0;  // If the difference is big, slow down changes
  const float SMALL_STEP = 0.5; // Fine adjustments step size
  
  if (fabs(currentLevel - targetLevel) > BIG_GAP) {
      if (currentLevel < targetLevel) {
          return currentLevel + 2.0;  // Faster increase for big gaps
      } else {
          return currentLevel - 2.0;  // Faster decrease for big gaps
      }
  } else {
      // Fine-tune with smaller step size
      if (currentLevel < targetLevel) {
          return currentLevel + SMALL_STEP;
      } else if (currentLevel > targetLevel) {
          return currentLevel - SMALL_STEP;
      }
  }
  return targetLevel;
}


// ============= Control Logic =============
void handleAutoMode() {
    switch(currentStatus) {
        case STATUS_DEFAULT:
            currentCase = DEF;
            if(tanks[0].valid && tanks[0].waterLevel <= NEARLY_EMPTY_THRESHOLD) {
                currentStatus = STATUS_LOW_WATER;
                Serial.println("Main tank is nearly empty, entering LOW_WATER status");
            }
            break;
        
        case STATUS_LOW_WATER:
            selectWaterSource();
            break;
        
        case STATUS_CONFIRMING_SOURCE:
            if(millis() - lastConfirmationTime > CONFIRMATION_DELAY) {
                confirmWaterSource();
            }
            break;
        
        case STATUS_FILLING:
            if(!pumpActive && (millis() - lastConfirmationTime > VALVE_PUMP_DELAY)) {
                pumpOn();
            }
            
            // Check stopping conditions
            if(checkCaseC()) {
                // Case C: Main tank is full
                Serial.println("Case C met: Main tank is full");
                deactivateWaterSource();
                currentStatus = STATUS_DEFAULT;
            }
            else if(checkCaseD()) {
                // Case D: Deepwell is nearly empty
                Serial.println("Case D met: Deepwell is nearly empty");
                deactivateWaterSource();
                deepwellWasLow = true;
                deepwellRestartTimer = millis();
                currentStatus = STATUS_WAITING_CONDITION_D;
            }
            else if(checkCaseE()) {
                // Case E: Both sources are nearly empty
                Serial.println("Case E met: Both sources are nearly empty");
                deactivateWaterSource();
                currentStatus = STATUS_DEFAULT;
            }
            break;
            
        case STATUS_WAITING_CONDITION_D:
            // Wait for 2 minutes before checking condition B
            if(millis() - deepwellRestartTimer > DEEPWELL_RESTART_DELAY) {
                Serial.println("Case D wait period complete, checking Case B");
                if(checkCaseB()) {
                    // If Case B is met, select rainwater source
                    selectedSource = SOURCE_RAINWATER;
                    currentStatus = STATUS_CONFIRMING_SOURCE;
                    confirmationCount = 0;
                    lastConfirmationTime = millis();
                    Serial.println("Case B met after wait: Selecting rainwater source");
                } else {
                    currentStatus = STATUS_DEFAULT;
                    Serial.println("Case B not met after wait: Returning to default status");
                }
                deepwellWasLow = false;
            }
            break;
            
        case STATUS_COOLDOWN:
            if(millis() - pumpCooldownTimer > PUMP_COOLDOWN_PERIOD) {
                pumpInCooldown = false;
                currentStatus = STATUS_DEFAULT;
                Serial.println("Pump cooldown complete, returning to default status");
            }
            break;
            
        case STATUS_SAFETY_SHUTDOWN:
            if(millis() - pumpSafetyTimer > PUMP_SAFETY_TIMEOUT) {
                deactivateWaterSource();
                pumpInSafetyMode = false;
                pumpCooldownTimer = millis();
                pumpInCooldown = true;
                currentStatus = STATUS_COOLDOWN;
                Serial.println("Safety shutdown complete, entering cooldown");
            }
            break;
    }
}

void handleManualMode(uint8_t command) {
    // Reset any active safety modes
    pumpInSafetyMode = false;
    
    if(command == 0) {
        // Switch to AUTO mode
        currentMode = MODE_AUTO;
        deactivateWaterSource();
        currentStatus = STATUS_DEFAULT;
        Serial.println("Switched to AUTO mode");
    } 
    else {
        // Parse manual command
        currentMode = MODE_MANUAL;
        uint8_t sourceCode = 0;
        
        if(command >= 10) {
            sourceCode = command % 10;
        } else {
            sourceCode = command;
        }
        
        // Deactivate current sources first
        deactivateWaterSource();
        
        // Activate the selected source
        if(sourceCode == 1) {
            // Rainwater source - Check Case G
            if(checkCaseG()) {
                digitalWrite(RAIN_SOLENOID_PIN, HIGH);
                selectedSource = SOURCE_RAINWATER;
                manualSource = SOURCE_RAINWATER;
                Serial.println("Manual mode: Activated RAINWATER source");
                
                // Start the pump after a delay to let the valve open
                vTaskDelay(pdMS_TO_TICKS(VALVE_PUMP_DELAY));
                pumpOn();
                currentStatus = STATUS_MANUAL;
            } else {
                Serial.println("Manual mode: Cannot activate RAINWATER - level too low");
            }
        } 
        else if(sourceCode == 2) {
            // Deepwell source - Check Case F
            if(checkCaseF()) {
                digitalWrite(DEEPWELL_SOLENOID_PIN, HIGH);
                selectedSource = SOURCE_DEEPWELL;
                manualSource = SOURCE_DEEPWELL;
                Serial.println("Manual mode: Activated DEEPWELL source");
                
                // Start the pump after a delay to let the valve open
                vTaskDelay(pdMS_TO_TICKS(VALVE_PUMP_DELAY));
                pumpOn();
                currentStatus = STATUS_MANUAL;
            } else {
                Serial.println("Manual mode: Cannot activate DEEPWELL - level too low");
            }
        } 
        else {
            // Invalid source or no source specified
            Serial.println("Manual mode: No source activated");
            currentCase = DEF;
            return;
        }
    }
}

void selectWaterSource() {
    if(!tanks[1].valid || !tanks[2].valid) {
        Serial.println("Cannot select source: Missing tank data");
        return;
    }
    
    // Check Case A: Main tank nearly empty, deepwell filled
    if(checkCaseA()) {
        selectedSource = SOURCE_DEEPWELL;
        currentStatus = STATUS_CONFIRMING_SOURCE;
        confirmationCount = 0;
        lastConfirmationTime = millis();
        Serial.println("Case A met: Selecting deepwell source");
        return;
    }
    
    // Check Case B: Main tank nearly empty, deepwell nearly empty, rainwater filled
    if(checkCaseB()) {
        selectedSource = SOURCE_RAINWATER;
        currentStatus = STATUS_CONFIRMING_SOURCE;
        confirmationCount = 0;
        lastConfirmationTime = millis();
        Serial.println("Case B met: Selecting rainwater source");
        return;
    }
    
    // If neither condition is met, revert to default
    currentStatus = STATUS_DEFAULT;
    selectedSource = SOURCE_NONE;
    Serial.println("No pumping conditions met, returning to default status");
}

void confirmWaterSource() {
    if(++confirmationCount >= 3) {
        activateWaterSource();
        currentStatus = STATUS_FILLING;
        Serial.println("Source confirmed and activated");
    }
    lastConfirmationTime = millis();
}

void activateWaterSource() {
    digitalWrite(RAIN_SOLENOID_PIN, LOW);
    digitalWrite(DEEPWELL_SOLENOID_PIN, LOW);
    digitalWrite(PUMP_RELAY_PIN, LOW);
    
    if(selectedSource == SOURCE_DEEPWELL) {
        digitalWrite(DEEPWELL_SOLENOID_PIN, HIGH);
        Serial.println("Activating Deepwell Source");
    } else if (selectedSource == SOURCE_RAINWATER) {
        digitalWrite(RAIN_SOLENOID_PIN, HIGH);
        Serial.println("Activating Rainwater Source");
    }
    
    lastConfirmationTime = millis();
    pumpActive = false;
}

void deactivateWaterSource() {
    digitalWrite(RAIN_SOLENOID_PIN, LOW);
    digitalWrite(DEEPWELL_SOLENOID_PIN, LOW);
    digitalWrite(PUMP_RELAY_PIN, LOW);
    selectedSource = SOURCE_NONE;
    confirmationCount = 0;
    pumpActive = false;
    Serial.println("All sources deactivated");
}

void pumpOn() {
    digitalWrite(PUMP_RELAY_PIN, HIGH);
    pumpActive = true;
    Serial.println("Pump turned ON");
}

void pumpOff() {
    digitalWrite(PUMP_RELAY_PIN, LOW);
    pumpActive = false;
    Serial.println("Pump turned OFF");
}

// ============= Safety and Condition Checks =============
void checkSafetyConditions() {
  // Check if we're in safety mode
  if(pumpInSafetyMode) {
      return; // Let the safety timer handle it
  }
  
  // Check if we're in cooldown
  if(pumpInCooldown) {
      return; // Let the cooldown timer handle it
  }
  
  // Only check water levels if the pump is active
  if(pumpActive) {
      bool sourceLow = false;
      
      // Check based on current mode and source
      if(currentMode == MODE_AUTO) {
          // In AUTO mode, check the active source water level
          if(selectedSource == SOURCE_DEEPWELL && tanks[1].valid && 
             tanks[1].waterLevel <= NEARLY_EMPTY_THRESHOLD) {
              sourceLow = true;
              Serial.println("Auto mode: Deepwell source is too low");
          } 
          else if(selectedSource == SOURCE_RAINWATER && tanks[2].valid && 
                  tanks[2].waterLevel <= NEARLY_EMPTY_THRESHOLD) {
              sourceLow = true;
              Serial.println("Auto mode: Rainwater source is too low");
          }
      }
      else if(currentMode == MODE_MANUAL) {
          // In MANUAL mode, check based on the selected source
          if(manualSource == SOURCE_DEEPWELL && tanks[1].valid && 
             tanks[1].waterLevel <= NEARLY_EMPTY_THRESHOLD) {
              sourceLow = true;
              Serial.println("Manual mode: Deepwell source is too low");
          } 
          else if(manualSource == SOURCE_RAINWATER && tanks[2].valid && 
                  tanks[2].waterLevel <= NEARLY_EMPTY_THRESHOLD) {
              sourceLow = true;
              Serial.println("Manual mode: Rainwater source is too low");
          }
      }
      
      // If source is low, start safety shutdown
      if(sourceLow && !pumpInSafetyMode) {
          pumpInSafetyMode = true;
          pumpSafetyTimer = millis();
          currentStatus = STATUS_SAFETY_SHUTDOWN;
          Serial.println("Source too low, entering safety shutdown for 10 seconds");
      }
  }
}

// ============= Auto Mode Condition Functions =============
bool checkCaseA() {
  // Case A: Main tank nearly empty (20%), deepwell filled (>20%)
  if(!tanks[0].valid || !tanks[1].valid) {
      Serial.println("Cannot check Case A: Missing tank data");
      return false;
  }
  
  bool result = (tanks[0].waterLevel <= NEARLY_EMPTY_THRESHOLD && 
                 tanks[1].waterLevel > FILLED_THRESHOLD);
  
  if(result) {
      Serial.println("Case A met: Main tank nearly empty, deepwell filled");
      currentCase = caseA;
  }
  
  return result;
}

bool checkCaseB() {
  // Case B: Main tank nearly empty (20%), deepwell nearly empty (20%), rainwater filled (>20%)
  if(!tanks[0].valid || !tanks[1].valid || !tanks[2].valid) {
      Serial.println("Cannot check Case B: Missing tank data");
      return false;
  }
  
  bool result = (tanks[0].waterLevel <= NEARLY_EMPTY_THRESHOLD && 
                 tanks[1].waterLevel <= NEARLY_EMPTY_THRESHOLD && 
                 tanks[2].waterLevel > FILLED_THRESHOLD);
  
  if(result) {
      Serial.println("Case B met: Main tank nearly empty, deepwell nearly empty, rainwater filled");
      currentCase = caseB;
  }
  
  return result;
}

bool checkCaseC() {
  // Case C: Main tank is full (95-100%)
  if(!tanks[0].valid) {
      Serial.println("Cannot check Case C: Missing main tank data");
      return false;
  }
  
  bool result = (tanks[0].waterLevel >= FULL_THRESHOLD_AUTO);
  
  if(result) {
      Serial.println("Case C met: Main tank is full (95-100%)");
      currentCase = caseC;
  }
  
  return result;
}

bool checkCaseD() {
  // Case D: Deepwell is nearly empty (20%)
  // Only applies if deepwell is the current source
  if(!tanks[1].valid || selectedSource != SOURCE_DEEPWELL) {
      return false;
  }
  
  bool result = (tanks[1].waterLevel <= NEARLY_EMPTY_THRESHOLD);
  
  if(result) {
      Serial.println("Case D met: Deepwell is nearly empty");
      currentCase = caseD;
  }
  
  return result;
}

bool checkCaseE() {
  // Case E: Both deepwell and rainwater are nearly empty (20%)
  if(!tanks[1].valid || !tanks[2].valid) {
      Serial.println("Cannot check Case E: Missing source tank data");
      return false;
  }
  
  bool result = (tanks[1].waterLevel <= NEARLY_EMPTY_THRESHOLD && 
                 tanks[2].waterLevel <= NEARLY_EMPTY_THRESHOLD);
  
  if(result) {
      Serial.println("Case E met: Both deepwell and rainwater are nearly empty");
      currentCase = caseE;
  }
  
  return result;
}

// ============= Manual Mode Condition Functions =============
bool checkCaseF() {
  // Case F: Deepwell has more than 20% water level
  if(!tanks[1].valid) {
      Serial.println("Cannot check Case F: Missing deepwell data");
      return false;
  }
  
  bool result = (tanks[1].waterLevel > FILLED_THRESHOLD);
  
  if(result) {
      Serial.println("Case F met: Deepwell has more than 20% water level");
      currentCase = caseF;
  }
  
  return result;
}

bool checkCaseG() {
  // Case G: Rainwater has more than 20% water level
  if(!tanks[2].valid) {
      Serial.println("Cannot check Case G: Missing rainwater data");
      return false;
  }
  
  bool result = (tanks[2].waterLevel > FILLED_THRESHOLD);
  
  if(result) {
      Serial.println("Case G met: Rainwater has more than 20% water level");
      currentCase = caseG;
  }
  
  return result;
}

bool checkCaseH() {
  // Case H: Main tank is full (90%)
  if(!tanks[0].valid) {
      Serial.println("Cannot check Case H: Missing main tank data");
      return false;
  }
  
  bool result = (tanks[0].waterLevel >= FULL_THRESHOLD_MANUAL);
  
  if(result) {
      Serial.println("Case H met: Main tank is full (90%)");
      currentCase = caseH;
  }
  
  return result;
}

bool checkCaseI() {
  // Case I: Deepwell is nearly empty (20%)
  if(!tanks[1].valid) {
      Serial.println("Cannot check Case I: Missing deepwell data");
      return false;
  }
  
  bool result = (tanks[1].waterLevel <= NEARLY_EMPTY_THRESHOLD);
  
  if(result) {
      Serial.println("Case I met: Deepwell is nearly empty");
      currentCase = caseI;
  }
  
  return result;
}

bool checkCaseJ() {
  // Case J: Rainwater is nearly empty (20%)
  if(!tanks[2].valid) {
      Serial.println("Cannot check Case J: Missing rainwater data");
      return false;
  }
  
  bool result = (tanks[2].waterLevel <= NEARLY_EMPTY_THRESHOLD);
  
  if(result) {
      Serial.println("Case J met: Rainwater is nearly empty");
      currentCase = caseJ;
  }
  return result;
}

// ============= Communication Functions =============
uint8_t calculateChecksum(void* packet, size_t size) {
  uint8_t sum = 0;
  const uint8_t* data = (uint8_t*)packet;
  for(size_t i = 0; i < size-1; i++) {
      sum ^= data[i];
  }
  return sum;
}

void sendCombinedTankData() {
  // Check if we have valid data for all tanks
  if(!tanks[0].valid || !tanks[1].valid || !tanks[2].valid) {
      Serial.println("Cannot send combined tank data: Missing valid tank data");
      return;
  }
  
  CombinedLoraPacket packet;
  packet.nodeID = NODE_ID;
  packet.main_level = tanks[0].waterLevel;
  packet.dw_level = tanks[1].waterLevel;
  packet.rain_level = tanks[2].waterLevel;
  packet.battery_v = heltec_vbat();
  packet.system_mode = static_cast<uint8_t>(currentMode);
  packet.system_status =static_cast<uint8_t>(currentStatus);
  packet.water_source = static_cast<uint8_t>(selectedSource);
  packet.current_case = static_cast<uint8_t>(currentCase);
  packet.checksum = 0;
  packet.checksum = calculateChecksum(&packet, sizeof(packet));
  
  int result = radio.transmit((uint8_t*)&packet, sizeof(packet));
  if(result == RADIOLIB_ERR_NONE) {
      Serial.println("Combined tank data sent successfully");
  } else {
      Serial.print("Failed to send combined tank data, error code: ");
      Serial.println(result);
  }
}

void sendAcknowledgement() {
  uint8_t ackPacket = 'A';  // Simple acknowledgment message
  int result = radio.transmit(&ackPacket, sizeof(ackPacket));

  if (result == RADIOLIB_ERR_NONE) {
      Serial.println("✅ ACK Sent Successfully!");
  } else {
      Serial.print("❌ Failed to send ACK, Error Code: ");
      Serial.println(result);
  }
}

// ============= Display Functions =============
void updateDisplay() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  // Mode and Status Line
  String modeText = (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL";
  String statusText = "Default";
  
  switch(currentStatus) {
      case STATUS_LOW_WATER: statusText = "Low Water"; break;
      case STATUS_CONFIRMING_SOURCE: statusText = "Confirming Source"; break;
      case STATUS_FILLING: statusText = "Filling"; break;
      case STATUS_MANUAL: statusText = "Manual Control"; break;
      case STATUS_SAFETY_SHUTDOWN: statusText = "Safety Shutdown"; break;
      case STATUS_COOLDOWN: statusText = "Cooldown"; break;
      case STATUS_WAITING_CONDITION_D: statusText = "Waiting (Case D)"; break;
  }
  
  display.drawString(0, 0, "Mode: " + modeText + " | " + statusText);
  display.drawHorizontalLine(0, 10, 128);
  
  // Tank Levels
  display.drawString(0, 12, "Main: " + String(tanks[0].valid ? String(tanks[0].waterLevel,1)+"%" : "---"));
  display.drawString(0, 24, "Deep: " + String(tanks[1].valid ? String(tanks[1].waterLevel,1)+"%" : "---"));
  display.drawString(0, 36, "Rain: " + String(tanks[2].valid ? String(tanks[2].waterLevel,1)+"%" : "---"));
  
  // Source and Pump Status
  String sourceText = "None";
  if(selectedSource == SOURCE_DEEPWELL) sourceText = "Deepwell";
  else if(selectedSource == SOURCE_RAINWATER) sourceText = "Rainwater";
  
  String pumpText = pumpActive ? "ON" : "OFF";
  display.drawString(0, 48, "Source: " + sourceText + " | Pump: " + pumpText);
  
  // Additional status information
  if(currentStatus == STATUS_CONFIRMING_SOURCE) {
      display.drawString(0, 60, "Confirming: " + String(confirmationCount) + "/3");
  } else if(pumpInSafetyMode) {
      unsigned long remaining = (PUMP_SAFETY_TIMEOUT - (millis() - pumpSafetyTimer)) / 1000;
      display.drawString(0, 60, "Safety: " + String(remaining) + "s remaining");
  } else if(pumpInCooldown) {
      unsigned long remaining = (PUMP_COOLDOWN_PERIOD - (millis() - pumpCooldownTimer)) / 1000;
      display.drawString(0, 60, "Cooldown: " + String(remaining) + "s remaining");
  } else if(currentStatus == STATUS_WAITING_CONDITION_D) {
      unsigned long remaining = (DEEPWELL_RESTART_DELAY - (millis() - deepwellRestartTimer)) / 1000;
      display.drawString(0, 60, "Waiting: " + String(remaining) + "s remaining");
  }
  
  display.display();
}

// ============= UI Helpers =============
void showMessage(const String& msg, int delayTime) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 24, msg);
  display.display();
  if(delayTime > 0) heltec_delay(delayTime);
}

void showLoadingScreen(int progress, String status) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 24, status);
  
  const int barWidth = 108;
  display.drawRect(10, 50, barWidth, 10);
  display.fillRect(10, 50, map(progress, 0, 100, 0, barWidth), 10);
  display.display();
}

float calculateWaterLevel(float distance) {
  if(distance >= TANK_HEIGHT_CM) {
      return 0.0f; // Empty tank
  }
  if(distance <= 0) {
      return 100.0f; // Full tank (or error)
  }
  float waterLevel = ((TANK_HEIGHT_CM - distance) / TANK_HEIGHT_CM) * 100.0f;
  return constrain(waterLevel, 0.0f, 100.0f);
}