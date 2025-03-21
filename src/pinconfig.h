#include <Arduino.h>
#include <heltec_unofficial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============= Configuration =============
// Ultrasonic sensor pins
#define MAIN_TRIG_PIN 7 // Main tank
#define MAIN_ECHO_PIN 6 // Main tank
#define RAIN_TRIG_PIN 4 // Rain tank
#define RAIN_ECHO_PIN 5 // Rain tank
#define DEEP_TRIG_PIN 2 // DW tank
#define DEEP_ECHO_PIN 3 // DW tank

// Relay pins
#define RAIN_SOLENOID_PIN 34
#define DEEPWELL_SOLENOID_PIN 36
#define PUMP_RELAY_PIN 35

// System parameters
#define TX_INTERVAL_MIN 2000
#define TX_INTERVAL_MAX 5000
#define DATA_TIMEOUT 300000
#define VALVE_PUMP_DELAY 5000
#define CONFIRMATION_DELAY 2000
#define LORA_FREQUENCY 433.0
#define LORA_SPREADING_FACTOR 9
#define LORA_BANDWIDTH 125.0
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0xF3
#define NODE_ID 1
#define NUM_TANKS 3
#define TANK_HEIGHT_CM 100.0

const float maintankheight = 89;
const float raintankheight = 90;
const float dwtankheight = 150;
const float TANK_HEIGHTS_CM[NUM_TANKS] = {maintankheight, raintankheight, dwtankheight}; // Example heights in cm for each tank
const float SENSOR_OFFSET_CM = 25.0; // Sensor is mounted 25 cm above the max water level

// Water level thresholds
#define NEARLY_EMPTY_THRESHOLD 20.0  // 20% water level is "Nearly Empty"
#define FILLED_THRESHOLD 20.0        // Above 20% is considered "Filled"
#define FULL_THRESHOLD_AUTO 95.0     // 95% water level is "Full" in AUTO mode (Case C)
#define FULL_THRESHOLD_MANUAL 90.0   // 90% water level is "Full" in MANUAL mode (Case H)

// Safety parameters
#define BLIND_DISTANCE_CM 25.0
#define SAMPLES 7
#define TIMEOUT_SENSOR 30000
#define PUMP_SAFETY_TIMEOUT 10000    // 10 seconds safety run after low water
#define PUMP_COOLDOWN_PERIOD 60000   // 60 seconds cooldown after pump stops
#define DEEPWELL_RESTART_DELAY 120000 // 2 minutes for condition D

// ============= Structures & Enums =============
#pragma pack(push, 1)
struct CombinedLoraPacket {
    uint8_t nodeID;
    float main_level;
    float dw_level;
    float rain_level;
    float battery_v;
    uint8_t system_mode;
    uint8_t system_status;
    uint8_t water_source;
    uint8_t current_case;
    uint8_t checksum;
};

struct ControlPacket {
    uint8_t command;  // 0 = AUTO, 1x = MANUAL (where x is the source)
    uint8_t checksum;
};
#pragma pack(pop)

struct TankData {
    float distance;
    float waterLevel;
    unsigned long lastUpdate;
    bool valid;
};

enum SystemMode { 
    MODE_AUTO, 
    MODE_MANUAL 
};
enum SystemStatus { 
    STATUS_DEFAULT, 
    STATUS_LOW_WATER, 
    STATUS_CONFIRMING_SOURCE, 
    STATUS_FILLING, 
    STATUS_MANUAL,
    STATUS_SAFETY_SHUTDOWN,
    STATUS_COOLDOWN,
    STATUS_WAITING_CONDITION_D
};
enum WaterSource { 
    SOURCE_NONE, 
    SOURCE_RAINWATER, 
    SOURCE_DEEPWELL 
};

// Case conditions
enum Case {
    //AUTO MODE
    caseA, // Case A - The Motor Pump is switched on when the Main Tank is “Nearly Empty” (20% water level) and the Deep Well is “Filled” (more than 20% water level).
    caseB,// Case B - The Motor Pump is switched on when the Main Tank is “Nearly Empty” (20% water level) and the Deep Well is “Nearly Empty” (20% water level) but the Rainwater Collector is “Filled” (more than 20% water level).
    caseC,// Case C -	The Motor Pump is switched off when the Main Tank is “Full” (95-100% water level).
    caseD,// Case D -	The Motor Pump is turned off when the Deep Well is “Nearly Empty” (20% water level) and it stays off for 2 minutes before the system decides whether to switch on the Motor Pump based on Case B.
    caseE,// Case E -	The Motor Pump is switched off when the Deep-well and Rainwater Collector are “Nearly Empty” (20% water level).
    //MANUAL MODE
    caseF,// Case F - The Motor Pump is switched on when selecting the Deep Well with more than 20% water level.
    caseG,// Case G- The Motor Pump is switched on when selecting the Rainwater Collector with more than 20% water level.
    caseH,// Case H -	The Motor Pump is switched off when the Main Tank is “Full” (90% water level).
    caseI,// Case I- The Motor Pump is switched off when the Deep Well is “Nearly Empty” (20% water level).
    caseJ,// Case J- The Motor Pump is switched off Rainwater Collector is “Nearly Empty” (20% water level).
    DEF, //default
};

Case currentCase = DEF;

struct UltrasonicSensor {
    int trigPin;
    int echoPin;
    float lastValidDistance;
};

// ============= Global Variables =============
TankData tanks[NUM_TANKS] = {0}; // Main, Deep, Rain
SystemMode currentMode = MODE_AUTO;
SystemStatus currentStatus = STATUS_DEFAULT;
WaterSource selectedSource = SOURCE_NONE;
WaterSource manualSource = SOURCE_NONE;
int confirmationCount = 0;
unsigned long lastConfirmationTime = 0;
unsigned long pumpSafetyTimer = 0;
unsigned long pumpCooldownTimer = 0;
unsigned long deepwellRestartTimer = 0;
bool pumpActive = false;
bool pumpInSafetyMode = false;
bool pumpInCooldown = false;
bool deepwellWasLow = false;
SemaphoreHandle_t tankDataMutex = NULL;
UltrasonicSensor sensors[NUM_TANKS];

// ============= Task Handles =============
TaskHandle_t loraTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
