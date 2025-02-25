#include <Arduino.h>
#include <heltec_unofficial.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN 19
#define ECHO_PIN 20
#define NODE_ID 1

// Tank Configuration
#define BLIND_ZONE_CM 25.0   
#define TANK_HEIGHT_CM 100.0 
#define SAMPLES 7            
#define TIMEOUT_US 26000     
#define TX_INTERVAL_MIN 5000  
#define TX_INTERVAL_MAX 15000 

//for 3.2 only
const int VEXT_control = 36;
const int ADC_control = 37;
//end 3.2

#pragma pack(push, 1)

String getLORAStatus(int state);
struct LoraPacket {
    uint8_t nodeID;       
    float distance_cm;    
    float battery_v;      
    uint8_t checksum;     
};
#pragma pack(pop)

struct UltrasonicSensor {
    int trigPin;
    int echoPin;
    float lastValidDistance;
};

UltrasonicSensor sensor;

void initSensor() {
    sensor.trigPin = TRIG_PIN;
    sensor.echoPin = ECHO_PIN;
    sensor.lastValidDistance = TANK_HEIGHT_CM;  // Start with full tank assumption
    
    pinMode(sensor.trigPin, OUTPUT);
    pinMode(sensor.echoPin, INPUT);
    digitalWrite(sensor.trigPin, LOW);
}

#define INIT_DELAY 500  // Time between progress updates

// Loading screen graphics
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

// LoRa Configuration
void setupLora() {
    radio.begin();
    radio.setFrequency(915.0);
    radio.setSpreadingFactor(9);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setSyncWord(0xF3);
    radio.setOutputPower(17);
}

uint8_t calculateChecksum(LoraPacket& packet) {
    uint8_t* data = (uint8_t*)&packet;
    uint8_t sum = 0;
    for(size_t i = 0; i < sizeof(packet) - 1; i++) {
        sum ^= data[i];
    }
    return sum;
}

void transmitData(float distance) {
    LoraPacket packet;
    packet.nodeID = NODE_ID;
    packet.distance_cm = distance;
    packet.battery_v = heltec_vbat();
    packet.checksum = calculateChecksum(packet);

    int state = radio.transmit((byte*)&packet, sizeof(packet));
    
    // Display transmission status
    display.drawString(64, 40, "TX: " + getLORAStatus(state));
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

float measureDistance();
float getStableDistance();

void setup() {
    //v3.2 only
    pinMode(VEXT_control, OUTPUT);
//end 3.2
    heltec_setup();

    //v3.2 only
    digitalWrite(VEXT_control, LOW); // VEXT ON ffor OLED display
    //end 3.2


    setupLora();
    // Initial loading screen
    showLoadingScreen(0, "Starting...");
    delay(INIT_DELAY);

    // Initialize serial
    showLoadingScreen(10, "Serial COM");
    Serial.begin(115200);
    delay(INIT_DELAY);

    // Initialize sensor
    showLoadingScreen(30, "Ultrasonic");
    initSensor();
    showLoadingScreen(40, "Testing sensor...");
    delay(INIT_DELAY);
    showLoadingScreen(50, "Sensor Ready");
    delay(INIT_DELAY);

    showLoadingScreen(60, "Battery: " + String(heltec_vbat(), 1) + "V");
    delay(INIT_DELAY);

    // Initialize LoRa
    showLoadingScreen(70, "LoRa Radio");
    int loraState = radio.begin();
    if(loraState != RADIOLIB_ERR_NONE) {
        showLoadingScreen(100, "LoRa FAIL: " + String(loraState));
        showLoadingScreen(100, "Please restart");
        while(1); // Halt on critical error
    }
    
    // Configure LoRa
    radio.setFrequency(915.0);
    radio.setSpreadingFactor(9);
    radio.setSyncWord(0xF3);
    showLoadingScreen(90, "LoRa Ready");
    delay(INIT_DELAY);

    // Complete initialization
    showLoadingScreen(100, "System Ready");
    delay(500);
    
    // Clear for main screen
    display.clear();
    display.display();
}


float measureDistance() {
    digitalWrite(sensor.trigPin, LOW);
    heltec_delay(2);
    digitalWrite(sensor.trigPin, HIGH);
    heltec_delay(10);
    digitalWrite(sensor.trigPin, LOW);

    long duration = pulseIn(sensor.echoPin, HIGH, TIMEOUT_US);
    float distance = (duration * 0.0343) / 2;
    
    if (distance <= 2.0 || distance > TANK_HEIGHT_CM) {
        return sensor.lastValidDistance;
    }
    sensor.lastValidDistance = distance;
    return distance;
}

float getStableDistance() {
    float readings[SAMPLES];
    for (int i = 0; i < SAMPLES; i++) {
        readings[i] = measureDistance();
        heltec_delay(20);
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


void loop() {
    float distance = getStableDistance();
    float waterLevel = ((TANK_HEIGHT_CM - distance)/TANK_HEIGHT_CM) * 100;
    
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0, "Battery: " + String(heltec_vbat(), 1) + "V");
    display.drawString(64, 15, "Distance: " + String(distance, 1) + "cm");
    display.drawString(64, 30, "Level: " + String(waterLevel, 1) + "%");
    
    transmitData(distance);
    display.display();
    
    // heltec_delay(random(TX_INTERVAL_MIN, TX_INTERVAL_MAX));
    // heltec_delay(100);
}
