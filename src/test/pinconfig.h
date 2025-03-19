#include <Arduino.h>
#include <heltec_unofficial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN 4
#define ECHO_PIN 5
#define NODE_ID 4

// Transmission Configuration
#define TX_INTERVAL_MIN 5000
#define TX_INTERVAL_MAX 15000
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
#define BLIND_ZONE_CM 25
#define TANK_HEIGHT_CM 100.0
#define SAMPLES 7
#define TIMEOUT_US 30000

// For 3.2 only
#define VEXT_control  36

// Battery constants
const float MIN_VOLTAGE = 3.2;
const float MAX_VOLTAGE = 4.2;

#pragma pack(push, 1)
struct __attribute__((packed)) LoraPacket {
    uint8_t nodeID;
    float waterLevel;    // Changed from distance_cm to waterLevel
    float battery_v;
    uint8_t checksum;
};
#pragma pack(pop)

struct UltrasonicSensor {
    int trigPin;
    int echoPin;
    float_t lastValidDistance;
};

// Shared data structure
struct SharedData {
    float distance;
    float waterLevel;
    float batteryPercent;
    unsigned long lastTxTime;
};

// Global variables
UltrasonicSensor sensor;
SharedData sensorData = {0};
SemaphoreHandle_t dataMutex = NULL;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;
