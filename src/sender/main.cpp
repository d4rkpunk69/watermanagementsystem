/* 
#include <Arduino.h>
#include <heltec_unofficial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN 4 //19 7
#define ECHO_PIN 5 //20 6
#define NODE_ID 4

// Transmission Configuration
#define TX_ENABLED true      // Set to false to disable transmission
#define TX_INTERVAL 5000     // Fixed transmission interval in milliseconds
uint8_t CALIBRATE = 5;
#define BIG_GAP 5
#define SMALL_GAP 1

// Lora Configs
#define LORA_FREQUENCY 433.0
#define LORA_SPREADING_FACTOR 9
#define LORA_BANDWIDTH 125.0
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0xF3
#define LORA_OUTPUT_POWER 17

// Tank Configuration
uint8_t BLIND_ZONE_CM = 25.0;
#define TANK_HEIGHT_CM 100.0 
#define SAMPLES 7            
#define TIMEOUT_US 30000     
#define TX_INTERVAL_MIN 5000  
#define TX_INTERVAL_MAX 15000 

// For 3.2 only
const int VEXT_control = 36;
const int ADC_control = 37;

// Battery constants (fallback if heltec_battery_percent fails)
const float MIN_VOLTAGE = 3.2;
const float MAX_VOLTAGE = 4.2;

#pragma pack(push, 1)
// uint8_t nodeID;       
// float distance_cm;    
// float battery_v;      
// uint8_t checksum;    
struct __attribute__((packed)) LoraPacket {
    uint8_t nodeID;       
    float distance_cm;    
    float battery_v;      
    uint8_t checksum;     
};
//4 0 0 C8 42 24 90 80 40 FA CE 3F 64 53 C9 3F 0 0 F4 34 4 0 0 0 9 0 0 0 7 0 0 0 12 0 0 0 0 0 0 0 B0 BE CE 3F 0 0 0 0 0 0 0 0 A 0 0 0 8 0 0 0 CD CC CC 3F 
#pragma pack(pop)

struct UltrasonicSensor {
    int trigPin;
    int echoPin;
    float_t lastValidDistance;
};

// Shared data structure with synchronization
struct SharedData {
    float distance;
    float waterLevel;
    float batteryPercent;
    unsigned long lastTxTime;
    unsigned long lastAckTime;
    bool ackReceived;
};

// Global variables
UltrasonicSensor sensor;
SharedData sensorData = {0};
SemaphoreHandle_t dataMutex = NULL;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
float distance = 0;

// Function prototypes
void sensorTask(void *parameter);
void loraTask(void *parameter);
void displayTask(void *parameter);
void initSensor();
float measureDistance();
float getStableDistance();
int batteryToPercent(float voltage);
String formatTimeAgo(unsigned long timestamp);
uint8_t calculateChecksum(LoraPacket& packet);
String getLORAStatus(int state);
void showLoadingScreen(int progress, String status);

void setup() {
    // For v3.2 only
    // Serial.begin(9600);
    pinMode(VEXT_control, OUTPUT);
    digitalWrite(VEXT_control, LOW); // VEXT ON for OLED display
    
    heltec_setup();
    
    // Initialize serial
    Serial.begin(115200);
    
    // Create mutex for data protection
    dataMutex = xSemaphoreCreateMutex();
    
    // Initial loading screen
    showLoadingScreen(0, "Starting...");
    delay(500);
    
    // Initialize sensor
    showLoadingScreen(30, "Ultrasonic");
    initSensor();
    showLoadingScreen(50, "Sensor Ready");
    delay(500);
    
    // Initialize LoRa
    showLoadingScreen(70, "LoRa Radio");
    int loraState = radio.begin();
    if(loraState != RADIOLIB_ERR_NONE) {
        showLoadingScreen(100, "LoRa FAIL: " + String(loraState));
        while(1); // Halt on critical error
    }
    
    // Configure LoRa
    radio.setFrequency(LORA_FREQUENCY);
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setBandwidth(LORA_BANDWIDTH);
    radio.setCodingRate(LORA_CODING_RATE);
    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setOutputPower(LORA_OUTPUT_POWER);
    
    showLoadingScreen(90, "LoRa Ready");
    delay(500);
    
    // Complete initialization
    showLoadingScreen(100, "System Ready");
    delay(500);
    
    // Create tasks pinned to specific cores
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
    
    xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        2,  // Higher priority for display
        &displayTaskHandle,
        1  // Run on Core 1
    );
}

void loop() {
    // Empty loop - tasks handle everything
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Sensor reading task
void sensorTask(void *parameter) {
    while(true) {
        float distance = getStableDistance();
        float waterLevel = ((TANK_HEIGHT_CM - distance)/TANK_HEIGHT_CM) * 100;
        float batteryVoltage = heltec_vbat();
        int batteryPercent = heltec_battery_percent(batteryVoltage);
        
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
        float distance = 0;
        float batteryPercent = 0;
        
        // Get latest data with mutex protection
        if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            distance = sensorData.distance;
            batteryPercent = sensorData.batteryPercent;
            xSemaphoreGive(dataMutex);
        }
        
        // Create and send packet
        LoraPacket packet;
        packet.nodeID = NODE_ID;
        packet.distance_cm = distance;
        packet.battery_v = heltec_vbat();  // Get fresh battery reading
        packet.checksum = 0;
        packet.checksum = calculateChecksum(packet);
        
        int state = radio.transmit((byte*)&packet, sizeof(packet));
        
        // Update transmission time with mutex protection
        if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            sensorData.lastTxTime = millis();
            xSemaphoreGive(dataMutex);
        }
        
        // Serial.println("TX Status: " + getLORAStatus(state));
        
        // Wait for ACK with timeout 
        //no need acknowledging just send directly
        //------------UNCOMMENT IF NEEDED-----------------
        // if(radio.available()) {
        //     byte ackData[sizeof(LoraPacket)];
        //     size_t len = sizeof(ackData);
        //     int state = radio.receive(ackData, len);
            
        //     if(state == RADIOLIB_ERR_NONE && len == sizeof(LoraPacket)) {
        //         LoraPacket* ack = (LoraPacket*)ackData;
                
        //         // Update ACK time with mutex protection
        //         if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        //             sensorData.lastAckTime = millis();
        //             sensorData.ackReceived = true;
        //             xSemaphoreGive(dataMutex);
        //         }
        //     }
        // }
        
        
        // Random delay to avoid collisions
        vTaskDelay(random(TX_INTERVAL_MIN, TX_INTERVAL_MAX) / portTICK_PERIOD_MS);
    }
}

// Display update task
void displayTask(void *parameter) {
    while(true) {
        // Local copies of shared data
        float distance = 0;
        float waterLevel = 0;
        float batteryPercent = 0;
        unsigned long lastTxTime = 0;
        unsigned long lastAckTime = 0;
        bool ackReceived = false;
        
        // Get latest data with mutex protection
        if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            distance = sensorData.distance;
            waterLevel = sensorData.waterLevel;
            batteryPercent = sensorData.batteryPercent;
            lastTxTime = sensorData.lastTxTime;
            // lastAckTime = sensorData.lastAckTime; //again no acknowledgin
            // ackReceived = sensorData.ackReceived;  
            xSemaphoreGive(dataMutex);
        }
        
        // Update display
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        
        // Header with node ID
        display.drawString(64, 0, "Node " + String(NODE_ID) + " Deepwell");
        display.drawHorizontalLine(0, 10, 128);
        
        // Main data
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 15, "Batt: " + String(batteryPercent, 1) + "%");
        display.drawString(0, 25, "Dist: " + String(distance, 1) + "cm");
        display.drawString(0, 35, "Level: " + String(waterLevel, 1) + "%");
        
        // Transmission info
        if(lastTxTime > 0) {
            display.drawString(0, 45, "Sent: " + formatTimeAgo(lastTxTime));
        }
        
        // Acknowledgment info
        if(ackReceived) {
            display.drawString(0, 55, "ACK: " + formatTimeAgo(lastAckTime));
        } else if(lastTxTime > 0) {
            display.drawString(0, 55, "ACK: Waiting...");
        }
        
        display.display();
        
        // Update display every 100ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void initSensor() {
    sensor.trigPin = TRIG_PIN;
    sensor.echoPin = ECHO_PIN;
    sensor.lastValidDistance = TANK_HEIGHT_CM;  // Start with full tank assumption
    
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
    distance = (duration * 0.0343) / 2;
    if (distance > TANK_HEIGHT_CM || distance == 0) {
        return sensor.lastValidDistance;
    }
    distance = distance + CALIBRATE - BLIND_ZONE_CM;
    if (fabs(distance - sensor.lastValidDistance) > BIG_GAP) {
    sensor.lastValidDistance = gradualAdjustDistance(sensor.lastValidDistance, distance);
    } else {
        // Normal update
        sensor.lastValidDistance = distance;
    }
    return distance;
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

// Convert voltage to percentage (fallback method)
int batteryToPercent(float voltage) {
    int percent = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
    return constrain(percent, 0, 100);  // Ensure range 0-100
}

// Format time elapsed since timestamp
String formatTimeAgo(unsigned long timestamp) {
    unsigned long elapsed = millis() - timestamp;
    
    if(elapsed < 60000) {  // Less than 1 minute
        return String(elapsed / 1000) + "s";
    } else if(elapsed < 3600000) {  // Less than 1 hour
        return String(elapsed / 60000) + "m";
    } else {  // Hours
        return String(elapsed / 3600000) + "h";
    }
}

uint8_t calculateChecksum(LoraPacket& packet) {
    uint8_t* data = (uint8_t*)&packet;
    uint8_t sum = 0;
    for(size_t i = 0; i < sizeof(packet) - 1; i++) {
        sum ^= data[i];
    }
    return sum;
}

String getLORAStatus(int state) {
    switch(state) {
        case RADIOLIB_ERR_NONE:          return "OK";
        case RADIOLIB_ERR_PACKET_TOO_LONG: return "TooLong";
        case RADIOLIB_ERR_TX_TIMEOUT:    return "Timeout";
        case RADIOLIB_ERR_CRC_MISMATCH:  return "CRC Error";
        case RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH: return "Bad Preamble";
        case RADIOLIB_ERR_INVALID_BANDWIDTH: return "Bad BW";
        case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "Bad SF";
        case RADIOLIB_ERR_INVALID_CODING_RATE: return "Bad CR";
        default: return "Err:" + String(state);
    }
}

void showLoadingScreen(int progress, String status) {
    display.clear();
    
    // Draw border
    display.drawRect(0, 0, display.width(), display.height());
    
    // Draw progress bar
    int barWidth = display.width() - 4;
    display.drawProgressBar(2, 2, barWidth, 8, progress);
    
    // Show status text
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.width()/2, 15, "Initializing...");
    display.drawString(display.width()/2, 30, status);
    
    display.display();
}
*/