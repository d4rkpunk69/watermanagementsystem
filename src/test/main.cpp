// #include "pinconfig.h"
// // Function prototypes
// void sensorTask(void *parameter);
// void loraTask(void *parameter);
// void initSensor();
// float measureDistance();
// float getStableDistance();
// int batteryToPercent(float voltage);
// uint8_t calculateChecksum(LoraPacket& packet);
// void enterDeepSleep();

// void setup() {
//     // For v3.2 only
//     pinMode(VEXT_control, OUTPUT);
//     digitalWrite(VEXT_control, LOW); // VEXT ON
    
//     heltec_setup();
//     Serial.begin(115200);

//       // Check if we're waking up from deep sleep
//       if (heltec_wakeup_was_timer()) {
//         Serial.println("Waking up from deep sleep timer...");
//         sleepCount++;
        
//         // Check battery level immediately
//         float batteryVoltage = heltec_vbat();
//         int batteryPercent = batteryToPercent(batteryVoltage);
//         Serial.println("Battery level on wake: " + String(batteryPercent) + "%");
        
//         // If we were in low battery mode and battery is still low, go back to sleep
//         if (wasInLowBatteryMode && batteryPercent < BATTERY_NORMAL_THRESHOLD) {
//             Serial.println("Battery still low, returning to sleep...");
//             enterDeepSleep();
            
//             // This code won't execute as the device will be in deep sleep
//         }

        
//         // If battery has recovered, reset the flag
//         if (wasInLowBatteryMode && batteryPercent >= BATTERY_NORMAL_THRESHOLD) {
//             wasInLowBatteryMode = false;
//             Serial.println("Battery recovered, resuming normal operation");
//         }
//     }
    
//     // Create mutex for data protection
//     dataMutex = xSemaphoreCreateMutex();
    
//     // Initialize sensor
//     Serial.println("Initializing sensor...");
//     initSensor();
    
//     // Initialize LoRa
//     Serial.println("Initializing LoRa...");
//     int loraState = radio.begin();
//     if(loraState != RADIOLIB_ERR_NONE) {
//         Serial.println("LoRa initialization failed: " + String(loraState));
//         while(1); // Halt on critical error
//     }
    
//     // Configure LoRa
//     radio.setFrequency(LORA_FREQUENCY);
//     radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
//     radio.setBandwidth(LORA_BANDWIDTH);
//     radio.setCodingRate(LORA_CODING_RATE);
//     radio.setSyncWord(LORA_SYNC_WORD);
//     radio.setOutputPower(LORA_OUTPUT_POWER);
    
//     Serial.println("System ready");
    
//     // Create tasks
//     xTaskCreatePinnedToCore(
//         sensorTask,
//         "SensorTask",
//         4096,
//         NULL,
//         1,
//         &sensorTaskHandle,
//         0  // Run on Core 0
//     );
    
//     xTaskCreatePinnedToCore(
//         loraTask,
//         "LoRaTask",
//         4096,
//         NULL,
//         1,
//         &loraTaskHandle,
//         0  // Run on Core 0
//     );
// }

// void loop() {
//     // Empty loop - tasks handle everything
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
// }

// // Function to enter deep sleep mode
// void enterDeepSleep() {
//     // Set flag to indicate we're in low battery mode
//     wasInLowBatteryMode = true;
    
//     // Properly shutdown peripherals to save power
//     Serial.println("Shutting down peripherals before sleep");
//     radio.sleep(true); // Shut down LoRa radio
    
//     // Prepare for deep sleep
//     Serial.println("Entering deep sleep mode due to low battery");
//     Serial.println("Sleep count: " + String(sleepCount));
//     Serial.println("Will check battery again in " + String(SLEEP_CHECK_INTERVAL) + " seconds");
    
//     // Flush serial
//     Serial.flush();
    
//     // Enter deep sleep with timer wakeup
//     heltec_deep_sleep(SLEEP_CHECK_INTERVAL);
    
//     // This code will not be executed (device is in deep sleep)
// }
// // Sensor reading task
// void sensorTask(void *parameter) {
//     while(true) {
//         float distance = getStableDistance();
//         float waterLevel = ((TANK_HEIGHT_CM - distance) / TANK_HEIGHT_CM) * 100.0;
//         // Constrain water level to valid range (0-100%)
//         waterLevel = constrain(waterLevel, 0.0, 100.0);
        
//         float batteryVoltage = heltec_vbat();
//         int batteryPercent = batteryToPercent(batteryVoltage);
        
//         // Update shared data with mutex protection
//         if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
//             sensorData.distance = distance;
//             sensorData.waterLevel = waterLevel;
//             sensorData.batteryPercent = batteryPercent;
//             xSemaphoreGive(dataMutex);
//         }
        
//         // Read every 2 seconds
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// // LoRa communication task
// void loraTask(void *parameter) {
//     while(true) {
//         float waterLevel = 0;
//         float batteryVoltage = 0;
        
//         // Get latest data with mutex protection
//         if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
//             waterLevel = sensorData.waterLevel;
//             batteryVoltage = sensorData.batteryPercent;
//             xSemaphoreGive(dataMutex);
//         }
        
//         // Create and send packet with water level instead of distance
//         LoraPacket packet;
//         packet.nodeID = NODE_ID;
//         packet.waterLevel = waterLevel;  // Send water level directly
//         packet.battery_v = heltec_vbat();  // Get fresh battery reading
//         packet.checksum = 0;
//         packet.checksum = calculateChecksum(packet);
        
//         int state = radio.transmit((byte*)&packet, sizeof(packet));
        
//         // Update transmission time with mutex protection
//         if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
//             sensorData.lastTxTime = millis();
//             xSemaphoreGive(dataMutex);
//         }
        
//         if(state == RADIOLIB_ERR_NONE) {
//             Serial.println("Packet sent: WL=" + String(waterLevel) + "%, Batt=" + String(packet.battery_v) + "V");
//         } else {
//             Serial.println("TX Error: " + String(state));
//         }
        
//         // Random delay to avoid collisions
//         vTaskDelay(random(TX_INTERVAL_MIN, TX_INTERVAL_MAX) / portTICK_PERIOD_MS);
//     }
// }

// void initSensor() {
//     sensor.trigPin = TRIG_PIN;
//     sensor.echoPin = ECHO_PIN;
//     sensor.lastValidDistance = TANK_HEIGHT_CM;  // Start with empty tank assumption
    
//     pinMode(sensor.trigPin, OUTPUT);
//     pinMode(sensor.echoPin, INPUT);
//     digitalWrite(sensor.trigPin, LOW);
// }

// float gradualAdjustDistance(float currentDistance, float targetDistance) {
//     // Gradually adjust the current distance towards the target
//     if (fabs(currentDistance - targetDistance) > BIG_GAP) {
//         if (currentDistance < targetDistance) {
//             return currentDistance + 1;
//         } else {
//             return currentDistance - 1;
//         }
//     } else {
//         // If within the ramp step, snap to target distance
//         return targetDistance;
//     }
// }

// float measureDistance() {
//     digitalWrite(sensor.trigPin, LOW);
//     delayMicroseconds(2);
//     digitalWrite(sensor.trigPin, HIGH);
//     delayMicroseconds(15);
//     digitalWrite(sensor.trigPin, LOW);

//     long duration = pulseIn(sensor.echoPin, HIGH, TIMEOUT_US);
//     float distance = (duration * 0.0343) / 2;
    
//     if (distance > TANK_HEIGHT_CM || distance == 0) {
//         return sensor.lastValidDistance;
//     }
//     Serial.println(distance);
//     distance = distance + CALIBRATE - BLIND_ZONE_CM;
    
//     if (fabs(distance - sensor.lastValidDistance) > BIG_GAP) {
//         sensor.lastValidDistance = gradualAdjustDistance(sensor.lastValidDistance, distance);
//     } else {
//         // Normal update
//         sensor.lastValidDistance = distance;
//     }
    
//     return sensor.lastValidDistance;
// }

// float getStableDistance() {
//     float readings[SAMPLES];
//     for (int i = 0; i < SAMPLES; i++) {
//         readings[i] = measureDistance();
//         delayMicroseconds(150);  // Small delay between readings
//     }
    
//     // Sort and return median
//     for (int i = 0; i < SAMPLES-1; i++) {
//         for (int j = 0; j < SAMPLES-i-1; j++) {
//             if (readings[j] > readings[j+1]) {
//                 float temp = readings[j];
//                 readings[j] = readings[j+1];
//                 readings[j+1] = temp;
//             }
//         }
//     }
//     return readings[SAMPLES/2];
// }

// // Convert voltage to percentage
// int batteryToPercent(float voltage) {
//     int percent = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
//     return constrain(percent, 0, 100);  // Ensure range 0-100
// }

// uint8_t calculateChecksum(LoraPacket& packet) {
//     uint8_t* data = (uint8_t*)&packet;
//     uint8_t sum = 0;
//     for(size_t i = 0; i < sizeof(packet) - 1; i++) {
//         sum ^= data[i];
//     }
//     return sum;
// }
