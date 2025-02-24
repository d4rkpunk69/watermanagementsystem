// #include <Arduino.h>
// #include "common/lora_water_system.h"
// #include <heltec_unofficial.h>

// // Hardware Configuration
// #define TRIG_PIN 19
// #define ECHO_PIN 20
// #define NODE_ID 1        // Unique for each sender (1-3)
// #define TANK_HEIGHT_CM 100.0
// #define SAMPLES 7        // Median filter samples
// #define TX_INTERVAL_MIN 5000  // 5 seconds
// #define TX_INTERVAL_MAX 15000 // 15 seconds

// struct UltrasonicSensor {
//     uint8_t trigPin;
//     uint8_t echoPin;
//     float lastValidDistance;
// } sensor;

// void setup() {
//     // Initialize hardware
//     heltec_ve(true);
//     heltec_setup();
    
//     // Sensor setup
//     sensor = {TRIG_PIN, ECHO_PIN, 0.0};
//     pinMode(TRIG_PIN, OUTPUT);
//     pinMode(ECHO_PIN, INPUT);
    
//     // LoRa configuration
//     radio.begin();
//     radio.setFrequency(LORA_BAND);
//     radio.setSpreadingFactor(9);
//     radio.setSyncWord(LORA_SYNC_WORD);
//     radio.setOutputPower(17);
    
//     // Initial display
//     display.clear();
//     display.drawString(64, 0, "Node " + String(NODE_ID));
//     display.display();
// }

// float measureDistance() {
//     digitalWrite(sensor.trigPin, LOW);
//     heltec_delay(2);
//     digitalWrite(sensor.trigPin, HIGH);
//     heltec_delay(10);
//     digitalWrite(sensor.trigPin, LOW);

//     long duration = pulseIn(sensor.echoPin, HIGH, 26000);
//     float distance = (duration * 0.0343) / 2;
    
//     // Validate and return
//     if(distance <= 2.0 || distance > TANK_HEIGHT_CM) {
//         return sensor.lastValidDistance;
//     }
//     sensor.lastValidDistance = distance;
//     return distance;
// }

// float getFilteredDistance() {
//     float readings[SAMPLES];
    
//     // Collect samples
//     for(int i = 0; i < SAMPLES; i++) {
//         readings[i] = measureDistance();
//         heltec_delay(20);
//     }
    
//     // Sort samples (bubble sort for small array)
//     for(int i = 0; i < SAMPLES-1; i++) {
//         for(int j = 0; j < SAMPLES-i-1; j++) {
//             if(readings[j] > readings[j+1]) {
//                 float temp = readings[j];
//                 readings[j] = readings[j+1];
//                 readings[j+1] = temp;
//             }
//         }
//     }

//     return readings[SAMPLES/2]; // Return median
// }

// void transmitData() {
//     LoraPacket packet;
    
//     // Populate packet
//     packet.nodeID = NODE_ID;
//     packet.distance_cm = getFilteredDistance();
//     packet.battery_v = heltec_vbat();
//     packet.checksum = calculateChecksum(packet);
    
//     // Transmit
//     radio.transmit((byte*)&packet, sizeof(packet));
    
//     // Update display
//     display.clear();
//     display.drawString(0, 0, "Node " + String(NODE_ID));
//     display.drawString(0, 16, "Distance: " + String(packet.distance_cm, 1) + "cm");
//     display.drawString(0, 32, "Battery: " + String(packet.battery_v, 1) + "V");
//     display.display();
// }

// void loop() {
//     transmitData();
    
//     // Random delay to prevent collisions
//     delay(random(TX_INTERVAL_MIN, TX_INTERVAL_MAX));
// }
