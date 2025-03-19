#include "pinconfig.h"
// ============= Function Prototypes =============
// Task Functions
void loraTask(void *parameter);
void displayTask(void *parameter);
void buttonTask(void *parameter);
void bluetoothTask(void *parameter);

// LoRa Functions
uint8_t calculateChecksum(void* packet, size_t length);
void sendControlCommand(uint8_t command);
bool sendControlCommandWithAck(uint8_t command);
void sendAcknowledgement();

// Bluetooth Functions
void initBluetooth();
bool isBluetoothConnected();
void toggleBluetooth();
void sendTankDataBluetooth();
void handleBluetoothCommands();
void processBluetoothCommand(const String& command);
void updateBluetoothStats(bool isSending, size_t bytes);

// UI Functions
void showMessage(const String& msg, int delayTime);
void updateDisplay(TankTab tab);
void updateBTSettingsDisplay();
void drawTank(int x, int y, int width, int height, float fillPercent);
String formatTimeAgo(unsigned long ms);
String formatBytes(unsigned long bytes);
int batteryToPercent(float voltage);
void beep(int duration);

// ============= Main Arduino Functions =============
void setup() {
    // Initialize hardware
    heltec_ve(true);
    heltec_setup();
    Serial.begin(115200);
    
    // Configure pins
    pinMode(LEFT_btn, INPUT_PULLUP);
    pinMode(RIGHT_btn, INPUT_PULLUP);
    pinMode(OK_btn, INPUT_PULLUP);
    pinMode(UP_btn, INPUT_PULLUP);
    pinMode(DOWN_btn, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Configure STATE pin if used
    if (HC05_STATE != -1) {
        pinMode(HC05_STATE, INPUT);
    }
    
    // Create mutex for thread safety
    tankDataMutex = xSemaphoreCreateMutex();
    
    showMessage("Starting...", 1000);
    
    // Initialize Bluetooth
    initBluetooth();
    showMessage("Bluetooth Ready", 1000);
    
    // Initialize LoRa
    int state = radio.begin(LORA_FREQUENCY);
    if(state == RADIOLIB_ERR_NONE) {
        radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
        radio.setBandwidth(LORA_BANDWIDTH);
        radio.setCodingRate(LORA_CODING_RATE);
        radio.setSyncWord(LORA_SYNC_WORD);
        showMessage("LoRa Ready", 1000);
    } else {
        showMessage("LoRa Error: " + String(state), 0);
        while(1) { 
            heltec_delay(1000); // Halt with error
        }
    }
    
    // Create tasks pinned to specific cores
    xTaskCreatePinnedToCore(
        loraTask,
        "LoRaTask",
        4096,
        NULL,
        1,
        &loraTaskHandle,
        1  // Run on Core 0
    );
    
    xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        2,  // Higher priority for display
        &displayTaskHandle,
        1  // Run on Core 1
    );
    
    xTaskCreatePinnedToCore(
        buttonTask,
        "ButtonTask",
        2048,
        NULL,
        1,
        &buttonTaskHandle,
        0  // Run on Core 0
    );
    
    xTaskCreatePinnedToCore(
        bluetoothTask,
        "BluetoothTask",
        4096,
        NULL,
        2,
        &bluetoothTaskHandle,
        0  // Run on Core 1
    );
}

void loop() {
    // Empty loop - tasks handle everything
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ============= Task Implementations =============
void loraTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10ms check interval
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
      // Buffer large enough for the largest packet type
      uint8_t dataBuffer[max(sizeof(CombinedLoraPacket), sizeof(LoraPacket))];
      int len = 0;
      int state = radio.receive(dataBuffer, len);
      // Serial.print(dataBuffer[len]);
      // Serial.print(" -- ");
      // Serial.println(len);
      // Serial.printf("NODE4: %i -- ", sizeof(LoraPacket));
      // Serial.printf("NODE123: %i \n", sizeof(CombinedLoraPacket));

      if (state == RADIOLIB_ERR_NONE) {
          // Determine packet type by size and process accordingly
          if (dataBuffer[0] == 1) {
            Serial.println("Combined");
              CombinedLoraPacket* packet = (CombinedLoraPacket*)dataBuffer;
              uint8_t receivedChecksum = packet->checksum;
              packet->checksum = 0; // Clear checksum for calculation

              if (calculateChecksum(packet, sizeof(CombinedLoraPacket) - 1) == receivedChecksum) {
                  if (xSemaphoreTake(tankDataMutex, portMAX_DELAY)) {
                      // Update data for all tanks
                      tanks[0].waterLevel = packet->main_level;
                      tanks[1].waterLevel = packet->dw_level;
                      tanks[2].waterLevel = packet->rain_level;
                      tanks[0].battery_v = packet->battery_v;
                      tanks[1].battery_v = packet->battery_v;
                      tanks[2].battery_v = packet->battery_v;
                      tanks[0].lastUpdate = millis();
                      tanks[1].lastUpdate = millis();
                      tanks[2].lastUpdate = millis();
                      tanks[0].valid = true;
                      tanks[1].valid = true;
                      tanks[2].valid = true;
                      xSemaphoreGive(tankDataMutex);
                  }
                  // Acknowledge successful reception
                  // sendAcknowledgement();
                  beep(50);
                  if (btEnabled && btConnected) {
                      sendTankDataBluetooth();
                  }
              }
          } 
          else if (dataBuffer[0] == 4) {
            Serial.println("Just node 4");
              LoraPacket* packet = (LoraPacket*)dataBuffer;
              if (packet->nodeID == 4) { // Expecting node ID 4 for livestock
                  uint8_t receivedChecksum = packet->checksum;
                  packet->checksum = 0; // Clear checksum for calculation

                  if (calculateChecksum(packet, sizeof(LoraPacket) - 1) == receivedChecksum) {
                      if (xSemaphoreTake(tankDataMutex, portMAX_DELAY)) {
                          // Update livestock tank data
                          tanks[3].distance = packet->distance_cm;
                          tanks[3].waterLevel = ((TANK_HEIGHT_CM - packet->distance_cm) / TANK_HEIGHT_CM) * 100.0f;
                          tanks[3].battery_v = packet->battery_v;
                          tanks[3].batteryPercent = batteryToPercent(packet->battery_v);
                          tanks[3].lastUpdate = millis();
                          tanks[3].valid = true;
                          xSemaphoreGive(tankDataMutex);
                      }
                      // Acknowledge successful reception
                      // sendAcknowledgement();
                      beep(50);
                      if (btEnabled && btConnected) {
                          sendTankDataBluetooth();
                      }
                  }
              }
          } 
          else if (dataBuffer[0] == 'A') { // Check for ACK
            Serial.println("Received ACK");
            reset = true;
          }
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void displayTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz refresh rate
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true) {
        // Update display with current tab
        if(xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(50))) {
            if(currentTab == BT_SETTINGS) {
                updateBTSettingsDisplay();
            } else {
                updateDisplay(currentTab);
            }
            xSemaphoreGive(tankDataMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void buttonTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 20Hz button polling
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true) {
        unsigned long now = millis();
        
        // Check if debounce period has passed
        if(now - lastButtonPress > DEBOUNCE_DELAY) {
            bool buttonPressed = false;
            
            // Tab navigation with LEFT/RIGHT buttons
            if(digitalRead(LEFT_btn) == LOW) {
                currentTab = (TankTab)((currentTab + 1) % 5); // 5 tabs including BT_SETTINGS
                // Serial.println("Left button pressed, switching to tab: " + String(TANK_NAMES[currentTab]));
                buttonPressed = true;
            }
            else if(digitalRead(RIGHT_btn) == LOW) {
                currentTab = (TankTab)((currentTab + 4) % 5); // +4 mod 5 = -1 mod 5
                // Serial.println("Right button pressed, switching to tab: " + String(TANK_NAMES[currentTab]));
                buttonPressed = true;
            }
            
            // Handle OK button for all tabs
            if(digitalRead(OK_btn) == LOW) {
                Serial.println("OK button pressed");
                
                if(currentTab == BT_SETTINGS) {
                    // Toggle Bluetooth in settings tab
                    toggleBluetooth();
                } else if(currentTab <= RAINWATER) { // Main, Deepwell, or Rainwater tabs
                    
                    if(currentTab == MAIN) {
                        // For main tank, switch to AUTO mode
                        selectionMode = AUTO;
                        selectedSource = 0;
                        sendControlCommandWithAck(0); // 0 = AUTO mode
                        Serial.println("Switching to AUTO mode");
                    }
                    else if(currentTab == DEEPWELL) {
                        // Select deepwell in manual mode
                        selectionMode = MANUAL;
                        selectedSource = 2; // 2 = deepwell
                        sendControlCommandWithAck(12); // 12 = MANUAL + deepwell
                        Serial.println("Switching to MANUAL mode with DEEPWELL source");
                    }
                    else if(currentTab == RAINWATER) {
                        // Select rainwater in manual mode
                        selectionMode = MANUAL;
                        selectedSource = 1; // 1 = rainwater
                        sendControlCommandWithAck(11); // 11 = MANUAL + rainwater
                        Serial.println("Switching to MANUAL mode with RAINWATER source");
                    }
                }
                
                buttonPressed = true;
            }
            
            // UP button toggles selection mode in settings tab
            if(digitalRead(UP_btn) == LOW && currentTab == BT_SETTINGS) {
                Serial.println("UP button pressed, toggling mode");
                
                // Toggle between AUTO and MANUAL modes
                if(selectionMode == AUTO) {
                    selectionMode = MANUAL;
                    // Keep previous source selection or default to rainwater
                    if(selectedSource == 0) selectedSource = 1;
                    sendControlCommandWithAck(10 + selectedSource); // 11 or 12
                } else {
                    selectionMode = AUTO;
                    sendControlCommandWithAck(0); // 0 = AUTO mode
                }
                
                buttonPressed = true;
            }
            
            // DOWN button cycles through source selection in settings tab
            if(digitalRead(DOWN_btn) == LOW && currentTab == BT_SETTINGS) {
                Serial.println("DOWN button pressed, cycling source");
                
                if(selectionMode == MANUAL) {
                    // Cycle between rainwater (1) and deepwell (2)
                    selectedSource = (selectedSource == 1) ? 2 : 1;
                    
                    // Send the appropriate command
                    sendControlCommandWithAck(10 + selectedSource); // 11 or 12
                }
                
                buttonPressed = true;
            }
            
            if(buttonPressed) {
                beep(30);
                lastButtonPress = now;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void bluetoothTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz BT processing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true) {
        // Only process Bluetooth if enabled
        if(btEnabled) {
            // Check Bluetooth connection status
            bool prevConnected = btConnected;
            btConnected = isBluetoothConnected();
            
            // Track new connections
            if(btConnected && !prevConnected) {
                btStats.connectionCount++;
                btStats.lastConnectTime = millis();
                beep(10); // Notify user of connection
            }
            
            // Handle incoming Bluetooth commands
            handleBluetoothCommands();
            
            // Send periodic updates to Bluetooth
            unsigned long now = millis();
            if(btConnected && (now - lastBluetoothUpdate > BT_UPDATE_INTERVAL)) {
                sendTankDataBluetooth();
                lastBluetoothUpdate = now;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============= Bluetooth Functions =============
void initBluetooth() {
    // Start the hardware serial port for Bluetooth
    BTSerial.begin(BT_BAUD_RATE, SERIAL_8N1, HC05_RX, HC05_TX);
    btConnected = false;
    btBufferIndex = 0;
    btStats = {0}; // Reset statistics
    lastBtActivity = 0;
}

bool isBluetoothConnected() {
    // If STATE pin is connected, use it for direct status
    if (HC05_STATE != -1) {
        return digitalRead(HC05_STATE) == HIGH;
    }
    
    // Otherwise, use a timeout-based approach
    if(BTSerial.available()) {
        lastBtActivity = millis();
        return true;
    }
    
    // Consider disconnected after 5 seconds of inactivity
    if(btConnected && millis() - lastBtActivity > 5000) {
        return false;
    }
    
    return btConnected; // Keep current state if no change
}

void toggleBluetooth() {
    btEnabled = !btEnabled;
    
    if(btEnabled) {
        // Re-initialize the Bluetooth module
        BTSerial.begin(BT_BAUD_RATE, SERIAL_8N1, HC05_RX, HC05_TX);
        showMessage("Bluetooth ON", 1000);
    } else {
        // Stop the Bluetooth serial
        BTSerial.end();
        btConnected = false;
        showMessage("Bluetooth OFF", 1000);
    }
}

void sendTankDataBluetooth() {
    if(!btConnected) return;
    
    if(xSemaphoreTake(tankDataMutex, pdMS_TO_TICKS(100))) {
        // Create JSON-like string with all tank data
        String jsonData = "{\"tanks\":[";
        
        for(int i = 0; i < NUM_TANKS; i++) {
            // Calculate water level percentage
            float waterLevel = -1.0;
            if(tanks[i].valid && (millis() - tanks[i].lastUpdate < DATA_TIMEOUT)) {
                waterLevel = tanks[i].waterLevel;
            }
            
            jsonData += "{";
            jsonData += "\"name\":\"" + String(TANK_NAMES[i]) + "\",";
            jsonData += "\"level\":" + (waterLevel >= 0 ? String(waterLevel, 1) : "null") + ",";
            
            // Only include battery for the livestock (Node 4)
            if(i == 3) {
                jsonData += "\"battery\":" + (tanks[i].valid ? String(tanks[i].batteryPercent) : "null") + ",";
            } else {
                jsonData += "\"battery\":null,"; // No battery for other tanks
            }
            
            jsonData += "\"valid\":" + String(tanks[i].valid ? "true" : "false") + ",";
            jsonData += "\"lastUpdate\":" + String(tanks[i].valid ? millis() - tanks[i].lastUpdate : 0);
            jsonData += "}";
            
            if(i < NUM_TANKS - 1) {
                jsonData += ",";
            }
        }
        
        jsonData += "],";
        jsonData += "\"selectedSource\":\"" + String(selectedSource == 1 ? "rainwater" : 
                                         (selectedSource == 2 ? "deepwell" : "none")) + "\",";
        jsonData += "\"selectionMode\":\"" + String(selectionMode == AUTO ? "auto" : "manual") + "\",";
        jsonData += "\"commandPending\":" + String(!commandAckReceived ? "true" : "false") + ",";
        jsonData += "\"btStats\":{";
        jsonData += "\"sent\":" + String(btStats.bytesSent) + ",";
        jsonData += "\"received\":" + String(btStats.bytesReceived) + ",";
        jsonData += "\"commands\":" + String(btStats.commandsReceived) + ",";
        jsonData += "\"connections\":" + String(btStats.connectionCount);
        jsonData += "}}";
        
        BTSerial.println(jsonData);
        updateBluetoothStats(true, jsonData.length() + 2); // +2 for \r\n
        xSemaphoreGive(tankDataMutex);
    }
}

void handleBluetoothCommands() {
    if(!btEnabled || !BTSerial.available()) return;
    
    while(BTSerial.available()) {
        char c = BTSerial.read();
        updateBluetoothStats(false, 1);
        lastBtActivity = millis(); // Update activity timestamp
        
        // Handle line ending
        if(c == '\n' || c == '\r') {
            if(btBufferIndex > 0) {
                btBuffer[btBufferIndex] = '\0';
                processBluetoothCommand(String(btBuffer));
                btBufferIndex = 0;
            }
        }
        // Add character to buffer if there's space
        else if(btBufferIndex < BT_BUFFER_SIZE - 1) {
            btBuffer[btBufferIndex++] = c;
        }
    }
}

void processBluetoothCommand(const String& command) {
  btStats.commandsReceived++;
  
  if(command == "getData") {
      // Send immediate data update
      sendTankDataBluetooth();
  }
  else if(command == "ping") {
      String response = "pong";
      BTSerial.println(response);
      updateBluetoothStats(true, response.length() + 2);
  }
  else if(command.startsWith("setTab:")) {
      // Extract tab number
      int tabNum = command.substring(7).toInt();
      if(tabNum >= 0 && tabNum < 5) { // 0-4 (including BT_SETTINGS)
          currentTab = (TankTab)tabNum;
          String response = "Tab set to " + String(TANK_NAMES[currentTab]);
          BTSerial.println(response);
          updateBluetoothStats(true, response.length() + 2);
      } else {
          String response = "Invalid tab number";
          BTSerial.println(response);
          updateBluetoothStats(true, response.length() + 2);
      }
  }
  else if(command == "setAuto") {
      selectionMode = AUTO;
      selectedSource = 0;
      sendControlCommandWithAck(0); // 0 = AUTO mode
      
      String response = "Selection mode set to AUTO";
      BTSerial.println(response);
      updateBluetoothStats(true, response.length() + 2);
  }
  else if(command.startsWith("setManual:")) {
      // Extract source number: 1=rainwater, 2=deepwell
      int sourceNum = command.substring(10).toInt();
      
      if(sourceNum == 1 || sourceNum == 2) {
          selectionMode = MANUAL;
          selectedSource = sourceNum;
          
          // Send the appropriate command with acknowledgment
          sendControlCommandWithAck(10 + sourceNum); // 11 or 12
          
          String sourceName = (sourceNum == 1) ? "RAINWATER" : "DEEPWELL";
          String response = "Selection mode set to MANUAL with " + sourceName + " source";
          BTSerial.println(response);
          updateBluetoothStats(true, response.length() + 2);
      } else {
          String response = "Invalid source number. Use 1 for rainwater or 2 for deepwell.";
          BTSerial.println(response);
          updateBluetoothStats(true, response.length() + 2);
      }
  }
  else if(command == "getStats") {
      String stats = "{";
      stats += "\"sent\":\"" + formatBytes(btStats.bytesSent) + "\",";
      stats += "\"received\":\"" + formatBytes(btStats.bytesReceived) + "\",";
      stats += "\"commands\":" + String(btStats.commandsReceived) + ",";
      stats += "\"connections\":" + String(btStats.connectionCount) + ",";
      stats += "\"uptime\":\"" + formatTimeAgo(millis()) + "\"";
      stats += "}";
      
      BTSerial.println(stats);
      updateBluetoothStats(true, stats.length() + 2);
  }
  else if(command == "getTankLevels") {
      String levels = "{";
      
      for(int i = 0; i < NUM_TANKS; i++) {
          float waterLevel = -1.0;
          if(tanks[i].valid && (millis() - tanks[i].lastUpdate < DATA_TIMEOUT)) {
              waterLevel = tanks[i].waterLevel;
          }
          
          levels += "\"" + String(TANK_NAMES[i]) + "\":" + 
                    (waterLevel >= 0 ? String(waterLevel, 1) : "null");
          
          if(i < NUM_TANKS - 1) {
              levels += ",";
          }
      }
      
      levels += "}";
      BTSerial.println(levels);
      updateBluetoothStats(true, levels.length() + 2);
  }
  else if(command == "getMode") {
      String modeInfo = "{";
      modeInfo += "\"mode\":\"" + String(selectionMode == AUTO ? "auto" : "manual") + "\",";
      modeInfo += "\"source\":\"" + String(selectedSource == 1 ? "rainwater" : 
                                       (selectedSource == 2 ? "deepwell" : "none")) + "\"";
      modeInfo += "}";
      
      BTSerial.println(modeInfo);
      updateBluetoothStats(true, modeInfo.length() + 2);
  }
  else if(command == "help") {
      String help = "Available commands:";
      help += "\n  getData - Get current tank data";
      help += "\n  ping - Connection test";
      help += "\n  setTab:X - Switch to tab X (0-4)";
      help += "\n  setAuto - Set auto selection mode";
      help += "\n  setManual:X - Set manual mode with source X (1=rain, 2=deepwell)";
      help += "\n  getTankLevels - Get only tank water levels";
      help += "\n  getMode - Get current mode and source";
      help += "\n  getStats - Get Bluetooth statistics";
      help += "\n  help - Show this help";
      
      BTSerial.println(help);
      updateBluetoothStats(true, help.length() + 2);
  }
  else {
      String response = "Unknown command. Type 'help' for available commands.";
      BTSerial.println(response);
      updateBluetoothStats(true, response.length() + 2);
  }
}

void updateBluetoothStats(bool isSending, size_t bytes) {
  if(isSending) {
      btStats.bytesSent += bytes;
  } else {
      btStats.bytesReceived += bytes;
  }
}

// ============= LoRa Functions =============
uint8_t calculateChecksum(void* packet, size_t length) {
  uint8_t sum = 0;
  const uint8_t* data = (uint8_t*)packet;
  for(size_t i = 0; i < length; i++) {
      sum ^= data[i];
  }
  return sum;
}

// void sendControlCommand(uint8_t command) {
//   ControlPacket packet;
//   packet.command = command;
//   packet.checksum = 0;
//   packet.checksum = calculateChecksum(&packet, sizeof(packet) - 1);
//   reset = false;
//   while(!reset) {
//     int result = radio.transmit((uint8_t*)&packet, sizeof(packet));
//     if(result == RADIOLIB_ERR_NONE) {
//         Serial.println("Control command " + String(command) + " sent successfully");
//     } else {
//         Serial.print("Failed to send control command, error code: ");
//         Serial.println(result);
//         // delayMicroseconds(1000);
//         // sendControlCommand(command); //added
//     }
//   }
// }

void sendControlCommand(uint8_t command) {
  ControlPacket packet;
  packet.command = command;
  packet.checksum = 0;
  packet.checksum = calculateChecksum(&packet, sizeof(packet) - 1);
  reset = false;
  
  unsigned long startTime = millis();
  const unsigned long timeout = 5000; // Retry sending for up to 5 seconds
  const unsigned long resendInterval = 500; // Send every 500ms until ACK received
  
  while (!reset) {
      int result = radio.transmit((uint8_t*)&packet, sizeof(packet));
      
      if (result == RADIOLIB_ERR_NONE) {
          Serial.println("Control command " + String(command) + " sent successfully");
      } else {
          Serial.print("Failed to send control command, error code: ");
          Serial.println(result);
      }
      
      // Check if ACK is received
      unsigned long ackStartTime = millis();
      while (millis() - ackStartTime < resendInterval) {
          if (reset) {
              Serial.println("ACK received. Stopping retransmission.");
              return;
          }
          delay(10); // Small delay to allow other tasks to run
      }
      
      // Timeout check
      if (millis() - startTime > timeout) {
          Serial.println("Timeout reached, stopping command transmission.");
          return;
      }
  }
}


bool sendControlCommandWithAck(uint8_t command) {
  // Take mutex to ensure command is not interrupted
  if(xSemaphoreTake(tankDataMutex, portMAX_DELAY)) {
      // Reset flags and counters
      commandAckReceived = false;
      pendingCommand = command;
      commandRetryCount = 0;
      
      // Send the initial command
      sendControlCommand(command);
      lastCommandSent = millis();
      
      xSemaphoreGive(tankDataMutex);
      return true; // Indicate that command sending has started
  }
  return false; // Failed to get mutex, command not sent
}

void sendAcknowledgement() {
  // Send a simple ACK string
  const char* ack = "ACK";
  int result = radio.transmit((uint8_t*)ack, strlen(ack));
  
  if(result == RADIOLIB_ERR_NONE) {
      Serial.println("ACK sent successfully");
  } else {
      Serial.print("Failed to send ACK, error code: ");
      Serial.println(result);
      // delayMicroseconds(1000);
      // sendAcknowledgement();
  }
}

// ============= UI Functions =============
void showMessage(const String& msg, int delayTime) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 24, msg);
  display.display();
  if (delayTime > 0) {
      heltec_delay(delayTime);
  }
}

void updateDisplay(TankTab tab) {
  display.clear();
  display.drawHorizontalLine(0, 12, 128);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, TANK_NAMES[tab]);

  TankData &tank = tanks[tab];
  float waterLevel = -1.0;
  if (tank.valid && (millis() - tank.lastUpdate < DATA_TIMEOUT)) {
      waterLevel = tank.waterLevel;
  }

  if (tank.valid && (millis() - tank.lastUpdate < DATA_TIMEOUT)) {
      drawTank(90, 15, 30, 50, waterLevel / 100.0);

      display.setTextAlignment(TEXT_ALIGN_LEFT);
      
      // Only show battery for the livestock (Node 4)
      if(tab == LIVESTOCK) {
          display.drawString(0, 15, "Batt: " + String(tank.batteryPercent) + "%");
      } else {
          display.drawString(0, 15, "Power: Connected");
      }
      
      display.drawString(0, 30, "Level: " + String(waterLevel, 1) + "%");
      display.drawString(0, 45, "Last: " + formatTimeAgo(millis() - tank.lastUpdate));

      if (tank.lastAckTime > 0) {
          display.drawString(0, 55, "ACK: " + formatTimeAgo(millis() - tank.lastAckTime));
      }
      
      // Show status indicators for water levels
      String statusText = "";
      if(waterLevel <= NEARLY_EMPTY_THRESHOLD) {
          statusText = "NEARLY EMPTY";
      } else if(tab == MAIN && waterLevel >= FULL_THRESHOLD_AUTO) {
          statusText = "FULL";
      } else if(waterLevel > FILLED_THRESHOLD) {
          statusText = "FILLED";
      }
      
      if(statusText != "") {
          int textWidth = display.getStringWidth(statusText);
          display.drawString(64 - textWidth/2, 55, statusText);
      }
  } else {
      display.drawString(64, 30, "NO DATA");
  }

  display.display();
}

void updateBTSettingsDisplay() {
  display.clear();
  display.drawHorizontalLine(0, 12, 128);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "SETTINGS");

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 15, "BT: " + String(btEnabled ? "ON" : "OFF"));
  display.drawString(0, 25, "Mode: " + String(selectionMode == AUTO ? "AUTO" : "MANUAL"));
  
  String sourceText = "None";
  if(selectionMode == MANUAL) {
      if(selectedSource == 1) {
          sourceText = "RAINWATER";
      } else if(selectedSource == 2) {
          sourceText = "DEEPWELL";
      }
  }
  display.drawString(0, 35, "Source: " + sourceText);
  
  // Show connection status
  display.drawString(0, 45, "BT Status: " + String(btConnected ? "Connected" : "Disconnected"));
  
  // Command pending indicator
  if (!commandAckReceived) {
      display.drawString(0, 55, "Cmd Pending: " + String(pendingCommand));
  }

  display.display();
}

void drawTank(int x, int y, int width, int height, float fillPercent) {
  display.drawRect(x, y, width, height);
  int fillHeight = constrain(height * fillPercent, 0, height);
  display.fillRect(x, y + (height - fillHeight), width, fillHeight);
  
  // Add level markers
  int lowLevel = height * (NEARLY_EMPTY_THRESHOLD / 100.0);
  int highLevel = height * (FULL_THRESHOLD_AUTO / 100.0);
  
  // Draw markers on the side of the tank
  display.drawLine(x-3, y + height - lowLevel, x-1, y + height - lowLevel);
  display.drawLine(x-3, y + height - highLevel, x-1, y + height - highLevel);
}

String formatTimeAgo(unsigned long ms) {
  if (ms < 60000) {  // Less than 1 minute
      return String(ms / 1000) + "s";
  } else if (ms < 3600000) {  // Less than 1 hour
      return String(ms / 60000) + "m";
  } else {  // Hours
      return String(ms / 3600000) + "h";
  }
}

String formatBytes(unsigned long bytes) {
  if (bytes < 1024) {
      return String(bytes) + " B";
  } else if (bytes < 1048576) {
      return String(bytes / 1024.0, 1) + " KB";
  } else {
      return String(bytes / 1048576.0, 1) + " MB";
  }
}

int batteryToPercent(float voltage) {
  float percentage = ((voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN)) * 100.0;
  return constrain(percentage, 0, 100);
}

void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  vTaskDelay(duration / portTICK_PERIOD_MS);
  digitalWrite(BUZZER_PIN, LOW);
}
