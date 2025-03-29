#include "test/pinconfig.h"

// Function prototypes
void sensorTask(void *parameter);
void loraTask(void *parameter);
void batteryMonitorTask(void *parameter);
void initSensor();
float measureDistance();
float getStableDistance();
int batteryToPercent(float voltage);
uint8_t calculateChecksum(LoraPacket& packet);
void enterDeepSleep();
void showSleepScreen(int batteryPercent);
void showWakeupScreen(int batteryPercent);
void showMessage(const String& title, const String& message, int delayTime = 0);

void setup() {
    // For v3.2 only
    pinMode(VEXT_control, OUTPUT);
    digitalWrite(VEXT_control, LOW); // VEXT ON
    
    heltec_setup();
    Serial.begin(115200);
    
    // Initialize display
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.clear();
    
    // Show startup message
    showMessage("Starting Up", "Node ID: " + String(NODE_ID), 1000);

    // Check if we're waking up from deep sleep
    if (heltec_wakeup_was_timer()) {
        Serial.println("Waking up from deep sleep timer...");
        sleepCount++;
        
        // Check battery level immediately
        float batteryVoltage = heltec_vbat();
        int batteryPercent = batteryToPercent(batteryVoltage);
        Serial.println("Battery level on wake: " + String(batteryPercent) + "%");
        
        // Show wake-up status on display
        showWakeupScreen(batteryPercent);
        
        // If we were in low battery mode and battery is still low, go back to sleep
        if (wasInLowBatteryMode && batteryPercent < BATTERY_NORMAL_THRESHOLD) {
            Serial.println("Battery still low, returning to sleep...");
            showMessage("Low Battery", "Still below threshold\nReturning to sleep...", 2000);
            enterDeepSleep();
            // This code won't execute as the device will be in deep sleep
        }
        
        // If battery has recovered, reset the flag
        if (wasInLowBatteryMode && batteryPercent >= BATTERY_NORMAL_THRESHOLD) {
            wasInLowBatteryMode = false;
            Serial.println("Battery recovered, resuming normal operation");
            showMessage("Battery Recovered", "Level: " + String(batteryPercent) + "%\nResuming operation", 2000);
        }
    }
    
    // Create mutex for data protection
    dataMutex = xSemaphoreCreateMutex();
    
    // Initialize sensor
    Serial.println("Initializing sensor...");
    showMessage("Initializing", "Sensor setup...", 500);
    initSensor();
    
    // Initialize LoRa
    Serial.println("Initializing LoRa...");
    showMessage("Initializing", "LoRa radio...", 500);
    int loraState = radio.begin();
    if(loraState != RADIOLIB_ERR_NONE) {
        Serial.println("LoRa initialization failed: " + String(loraState));
        showMessage("ERROR", "LoRa init failed\nCode: " + String(loraState), 0);
        while(1); // Halt on critical error
    }
    
    // Configure LoRa
    radio.setFrequency(LORA_FREQUENCY);
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setBandwidth(LORA_BANDWIDTH);
    radio.setCodingRate(LORA_CODING_RATE);
    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setOutputPower(LORA_OUTPUT_POWER);
    
    showMessage("System Ready", "Starting tasks...", 1000);
    Serial.println("System ready");
    
    // Create tasks
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        4096,
        NULL,
        1,
        &sensorTaskHandle,
        0  // Run on Core 0
    );
    
    xTaskCreatePinnedToCore(
        loraTask,
        "LoRaTask",
        4096,
        NULL,
        1,
        &loraTaskHandle,
        0  // Run on Core 0
    );
    
    // Create dedicated battery monitor task
    xTaskCreatePinnedToCore(
        batteryMonitorTask,
        "BatteryTask",
        2048,
        NULL,
        2,  // Higher priority to ensure it runs
        NULL,
        1  // Run on Core 1
    );
}

void loop() {
    // Empty loop - tasks handle everything
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Dedicated battery monitoring task
void batteryMonitorTask(void *parameter) {
    // Wait a bit for system to stabilize
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    while(true) {
        // Check battery level
        float batteryVoltage = heltec_vbat();
        int batteryPercent = batteryToPercent(batteryVoltage);
        
        Serial.println("Battery monitor: " + String(batteryPercent) + "%");
        
        // Check if battery is low and we're not already in low battery mode
        if (batteryPercent < BATTERY_LOW_THRESHOLD && !wasInLowBatteryMode) {
            Serial.println("Low battery detected in monitor task: " + String(batteryPercent) + "%");
            
            // Show warning on display
            showMessage("LOW BATTERY", "Battery at " + String(batteryPercent) + "%\nPreparing for sleep...", 2000);
            
            // Send a final LoRa packet if possible
            if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500))) {
                float waterLevel = sensorData.waterLevel;
                
                LoraPacket packet;
                packet.nodeID = NODE_ID;
                packet.waterLevel = waterLevel;
                packet.battery_v = batteryVoltage;
                packet.checksum = 0;
                packet.checksum = calculateChecksum(packet);
                
                xSemaphoreGive(dataMutex);
                
                radio.transmit((byte*)&packet, sizeof(packet));
                Serial.println("Low battery notification sent");
            }
            
            // Allow time for packet to be sent
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
            // Enter deep sleep
            enterDeepSleep();
        }
        
        // Check every 30 seconds
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

// Function to enter deep sleep mode
void enterDeepSleep() {
    // Set flag to indicate we're in low battery mode
    wasInLowBatteryMode = true;
    
    // Get current battery percentage
    float batteryVoltage = heltec_vbat();
    int batteryPercent = batteryToPercent(batteryVoltage);
    
    // Show sleep screen with battery information
    showSleepScreen(batteryPercent);
    
    // Properly shutdown peripherals to save power
    Serial.println("Shutting down peripherals before sleep");
    
    // Suspend all tasks to ensure clean shutdown
    if (sensorTaskHandle != NULL) vTaskSuspend(sensorTaskHandle);
    if (loraTaskHandle != NULL) vTaskSuspend(loraTaskHandle);
    
    radio.sleep(); // Put LoRa radio to sleep
    
    // Prepare for deep sleep
    Serial.println("Entering deep sleep mode due to low battery");
    Serial.println("Sleep count: " + String(sleepCount));
    Serial.println("Will check battery again in " + String(SLEEP_CHECK_INTERVAL) + " seconds");
    
    // Flush serial
    Serial.flush();
    
    // Enter deep sleep with timer wakeup
    delay(100); // Short delay to ensure all operations complete
    heltec_deep_sleep(SLEEP_CHECK_INTERVAL);
    
    // This code will not be executed (device is in deep sleep)
}

// Sensor reading task
void sensorTask(void *parameter) {
    while(true) {
        float distance = getStableDistance();
        float waterLevel = ((TANK_HEIGHT_CM - distance) / TANK_HEIGHT_CM) * 100.0;
        // Constrain water level to valid range (0-100%)
        waterLevel = constrain(waterLevel, 0.0, 100.0);
        
        float batteryVoltage = heltec_vbat();
        int batteryPercent = batteryToPercent(batteryVoltage);
        
        // Update shared data with mutex protection
        if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            sensorData.distance = distance;
            sensorData.waterLevel = waterLevel;
            sensorData.batteryPercent = batteryPercent;
            xSemaphoreGive(dataMutex);
        }
        
        // Read every 2 seconds
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// LoRa communication task
void loraTask(void *parameter) {
    while(true) {
        float waterLevel = 0;
        float batteryVoltage = 0;
        
        // Get latest data with mutex protection
        if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            waterLevel = sensorData.waterLevel;
            batteryVoltage = heltec_vbat(); // Get fresh reading
            xSemaphoreGive(dataMutex);
        }
        
        // Create and send packet with water level instead of distance
        LoraPacket packet;
        packet.nodeID = NODE_ID;
        packet.waterLevel = waterLevel;  // Send water level directly
        packet.battery_v = batteryVoltage;  // Get fresh battery reading
        packet.checksum = 0;
        packet.checksum = calculateChecksum(packet);
        
        int state = radio.transmit((byte*)&packet, sizeof(packet));
        
        // Update transmission time with mutex protection
        if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            sensorData.lastTxTime = millis();
            xSemaphoreGive(dataMutex);
        }
        
        if(state == RADIOLIB_ERR_NONE) {
            Serial.println("Packet sent: WL=" + String(waterLevel) + "%, Batt=" + String(batteryVoltage) + "V");
            
            // Occasionally update display with current status
            static unsigned long lastDisplayUpdate = 0;
            if (millis() - lastDisplayUpdate > 30000) { // Every 30 seconds
                display.clear();
                display.setTextAlignment(TEXT_ALIGN_LEFT);
                display.drawString(0, 0, "Node: " + String(NODE_ID));
                display.drawString(0, 12, "Battery: " + String(batteryToPercent(batteryVoltage)) + "%");
                display.drawString(0, 24, "Water Level: " + String(waterLevel, 1) + "%");
                display.drawString(0, 36, "Tx Status: OK");
                
                // Draw simple battery icon
                int battWidth = 30;
                display.drawRect(90, 10, battWidth, 15);
                display.drawRect(90 + battWidth, 13, 3, 9); // Battery nub
                display.fillRect(90, 10, (battWidth * batteryToPercent(batteryVoltage) / 100), 15);
                
                display.display();
                lastDisplayUpdate = millis();
            }
        } else {
            Serial.println("TX Error: " + String(state));
        }
        
        // Random delay to avoid collisions
        vTaskDelay(random(TX_INTERVAL_MIN, TX_INTERVAL_MAX) / portTICK_PERIOD_MS);
    }
}

// Display the sleep screen with battery information
void showSleepScreen(int batteryPercent) {
    display.clear();
    
    // Draw header
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0, "SLEEP MODE");
    
    // Draw battery info
    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 20, "Battery Low: " + String(batteryPercent) + "%");
    display.drawString(64, 32, "Next check in " + String(SLEEP_CHECK_INTERVAL) + "s");
    display.drawString(64, 44, "Sleep count: " + String(sleepCount));
    
    // Draw battery icon
    int battWidth = 40;
    int battHeight = 15;
    int battX = 44;
    int battY = 56;
    display.drawRect(battX, battY, battWidth, battHeight);
    display.drawRect(battX + battWidth, battY + 3, 3, battHeight - 6); // Battery nub
    display.fillRect(battX, battY, (battWidth * batteryPercent / 100), battHeight);
    
    display.display();
    
    // Allow time for the user to see the display
    delay(DISPLAY_TIMEOUT);
}

// Display the wakeup screen with battery information
void showWakeupScreen(int batteryPercent) {
    display.clear();
    
    // Draw header
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0, "WAKING UP");
    
    // Draw battery info
    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 20, "Battery: " + String(batteryPercent) + "%");
    display.drawString(64, 32, "Sleep count: " + String(sleepCount));
    
    if (batteryPercent < BATTERY_NORMAL_THRESHOLD) {
        display.drawString(64, 44, "Battery still low");
    } else {
        display.drawString(64, 44, "Battery OK - resuming");
    }
    
    // Draw battery icon
    int battWidth = 40;
    int battHeight = 15;
    int battX = 44;
    int battY = 56;
    display.drawRect(battX, battY, battWidth, battHeight);
    display.drawRect(battX + battWidth, battY + 3, 3, battHeight - 6); // Battery nub
    display.fillRect(battX, battY, (battWidth * batteryPercent / 100), battHeight);
    
    display.display();
    
    // Allow time for the user to see the display
    delay(2000);
}

// Display a message on the OLED with title and content
void showMessage(const String& title, const String& message, int delayTime) {
    display.clear();
    
    // Draw title
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0, title);
    
    // Draw message (support for multi-line messages)
    display.setFont(ArialMT_Plain_10);
    
    int y = 22;
    int startPos = 0;
    int nextLinePos;
    
    // Parse and draw each line
    while ((nextLinePos = message.indexOf('\n', startPos)) != -1) {
        String line = message.substring(startPos, nextLinePos);
        display.drawString(64, y, line);
        y += 12;
        startPos = nextLinePos + 1;
    }
    
    // Draw the last line
    if (startPos < message.length()) {
        display.drawString(64, y, message.substring(startPos));
    }
    
    display.display();
    
    // Delay if specified
    if (delayTime > 0) {
        delay(delayTime);
    }
}

void initSensor() {
    sensor.trigPin = TRIG_PIN;
    sensor.echoPin = ECHO_PIN;
    sensor.lastValidDistance = TANK_HEIGHT_CM;  // Start with empty tank assumption
    
    pinMode(sensor.trigPin, OUTPUT);
    pinMode(sensor.echoPin, INPUT);
    digitalWrite(sensor.trigPin, LOW);
}

float gradualAdjustDistance(float currentDistance, float targetDistance) {
    // Gradually adjust the current distance towards the target
    if (fabs(currentDistance - targetDistance) > BIG_GAP) {
        if (currentDistance < targetDistance) {
            return currentDistance + 1;
        } else {
            return currentDistance - 1;
        }
    } else {
        // If within the ramp step, snap to target distance
        return targetDistance;
    }
}

float measureDistance() {
    digitalWrite(sensor.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.trigPin, HIGH);
    delayMicroseconds(15);
    digitalWrite(sensor.trigPin, LOW);

    long duration = pulseIn(sensor.echoPin, HIGH, TIMEOUT_US);
    float distance = (duration * 0.0343) / 2;
    
    if (distance > TANK_HEIGHT_CM || distance == 0) {
        return sensor.lastValidDistance;
    }
    Serial.println(distance);
    distance = distance + CALIBRATE - BLIND_ZONE_CM;
    
    if (fabs(distance - sensor.lastValidDistance) > BIG_GAP) {
        sensor.lastValidDistance = gradualAdjustDistance(sensor.lastValidDistance, distance);
    } else {
        // Normal update
        sensor.lastValidDistance = distance;
    }
    
    return sensor.lastValidDistance;
}

float getStableDistance() {
    float readings[SAMPLES];
    for (int i = 0; i < SAMPLES; i++) {
        readings[i] = measureDistance();
        delayMicroseconds(150);  // Small delay between readings
    }
    
    // Sort and return median
    for (int i = 0; i < SAMPLES-1; i++) {
        for (int j = 0; j < SAMPLES-i-1; j++) {
            if (readings[j] > readings[j+1]) {
                float temp = readings[j];
                readings[j] = readings[j+1];
                readings[j+1] = temp;
            }
        }
    }
    return readings[SAMPLES/2];
}

// Convert voltage to percentage
int batteryToPercent(float voltage) {
    int percent = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
    return constrain(percent, 0, 100);  // Ensure range 0-100
}

uint8_t calculateChecksum(LoraPacket& packet) {
    uint8_t* data = (uint8_t*)&packet;
    uint8_t sum = 0;
    for(size_t i = 0; i < sizeof(packet) - 1; i++) {
        sum ^= data[i];
    }
    return sum;
}