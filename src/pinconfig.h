#include <Arduino.h>
#include <heltec_unofficial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============= HC-05 Bluetooth Module =============
#define HC05_TX 40  // Connect to RXD of HC-05
#define HC05_RX 42  // Connect to TXD of HC-05
#define HC05_STATE 41 // Optional connection status pin
#define BT_BAUD_RATE 9600

// ============= Configuration =============
// Button Configuration
#define OK_btn 26
#define LEFT_btn 20
#define RIGHT_btn 47
#define UP_btn 48
#define DOWN_btn 19
#define BUZZER_PIN 36

// Timing Configuration
#define DEBOUNCE_DELAY 200
#define DATA_TIMEOUT 300000  // 5 minutes in milliseconds
#define BT_UPDATE_INTERVAL 5000  // Bluetooth update interval (5 seconds)
#define COMMAND_RETRY_INTERVAL 1000 // 1 second between command retries

// LoRa Configuration
#define LORA_FREQUENCY 433.0
#define LORA_SPREADING_FACTOR 9
#define LORA_BANDWIDTH 125.0
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0xF3
#define NODE_ID 1  // This device is Node 1 (receiver)

// Tank Configuration
#define NUM_TANKS 4
#define TANK_HEIGHT_CM 100.0

// Water level thresholds
#define NEARLY_EMPTY_THRESHOLD 20.0  // 20% water level is "Nearly Empty"
#define FILLED_THRESHOLD 20.0        // Above 20% is considered "Filled"
#define FULL_THRESHOLD_AUTO 95.0     // 95% water level is "Full" in AUTO mode
#define FULL_THRESHOLD_MANUAL 90.0   // 90% water level is "Full" in MANUAL mode

// Battery Configuration
#define BATTERY_MIN 3.2
#define BATTERY_MAX 4.2

// Bluetooth Configuration
#define BT_DEVICE_NAME "TankMonitor"
#define BT_BUFFER_SIZE 128  // Command buffer size
bool reset = true;

// ============= Data Structures =============
// Combined LoRa Packet Structure - For receiving data from the combined node
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

// Livestock LoRa Packet Structure - For receiving data from node 4
struct __attribute__((packed)) LoraPacket {
    uint8_t nodeID;
    float waterLevel;    // Changed from distance_cm to waterLevel
    float battery_v;
    uint8_t checksum;
};

// Control Packet Structure - For sending commands to the combined node
struct ControlPacket {
    uint8_t command;  // 0 = AUTO, 1x = MANUAL (where x is the source)
    uint8_t checksum;
};
#pragma pack(pop)

enum Case {
    caseA,
    caseB,
    caseC,
    caseD,
    caseE,
    caseF,
    caseG,
    caseH,
    caseI,
    caseJ,
    DEF,
};
Case currentCase = DEF;

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
SystemStatus systemStatus;

// Tank Data Structure
struct TankData {
    float distance;
    float waterLevel;
    float battery_v;
    int batteryPercent;
    unsigned long lastUpdate;
    unsigned long lastAckTime;
    bool valid;
};

// Bluetooth Statistics Structure
struct BluetoothStats {
    unsigned long bytesSent;
    unsigned long bytesReceived;
    unsigned long commandsReceived;
    unsigned long lastConnectTime;
    unsigned long connectionCount;
};

// Tank Type Enumeration (added BT_SETTINGS tab)
enum TankTab {MAIN, DEEPWELL, RAINWATER, LIVESTOCK, BT_SETTINGS};

// Tank Selection Mode
enum SelectionMode {AUTO, MANUAL};

// ============= Global Variables =============
const char* TANK_NAMES[] = {"MAIN TANK", "DEEPWELL", "RAINWATER", "LIVESTOCK", "BT SETTINGS"};
const char *tankname[] = {"MAIN TANK", "RAINWATER", "DEEPWELL", "LIVESTOCK"};
TankData tanks[NUM_TANKS] = {0};
TankTab currentTab = MAIN;
unsigned long lastTabChange = 0;
unsigned long lastBluetoothUpdate = 0;
unsigned long lastButtonPress = 0;

// Bluetooth variables
HardwareSerial BTSerial(2); // UART2 on ESP32
bool btEnabled = true;
bool btConnected = false;
char btBuffer[BT_BUFFER_SIZE];
int btBufferIndex = 0;
BluetoothStats btStats = {0};
unsigned long lastBtActivity = 0;

// Tank selection variables
SelectionMode selectionMode = AUTO;  // Default to AUTO mode

enum WaterSource { SOURCE_NONE, SOURCE_RAINWATER, SOURCE_DEEPWELL };
WaterSource selectedSource = SOURCE_NONE;  // 0=none, 1=rainwater, 2=deepwell

// Command acknowledgment variables
bool commandAckReceived = true;  // Start with no pending commands
uint8_t pendingCommand = 0;
unsigned long lastCommandSent = 0;
uint8_t commandRetryCount = 0;

// Mutex for thread safety
SemaphoreHandle_t tankDataMutex = NULL;

// Task handles
TaskHandle_t loraTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t bluetoothTaskHandle = NULL;
