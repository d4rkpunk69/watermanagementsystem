#include <Arduino.h>
#include <heltec_3.2.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN 6
#define ECHO_PIN 7

// Tank Configuration
#define BLIND_ZONE_CM 25.0   // Ignore readings beyond this distance
#define TANK_HEIGHT_CM 100.0 // Adjust to your tank's total height
#define SAMPLES 7            // Number of samples for filtering (increase for better accuracy)
#define TIMEOUT_US 30000     // 30ms timeout for pulseIn

// Struct to Store Sensor Data
struct UltrasonicSensor {
    int trigPin;
    int echoPin;
    float lastValidDistance;
};

// Function Prototypes
void initSensor(UltrasonicSensor* sensor, int trig, int echo);
float measureDistance(UltrasonicSensor* sensor);
float getStableDistance(UltrasonicSensor* sensor);
void calculateWaterLevel(UltrasonicSensor* sensor);
float medianFilter(float values[], int size);

UltrasonicSensor sensor; // Static allocation to avoid dynamic memory issues

void setup() {
    heltec_ve(true);
    display.init();
    display.setContrast(255);
    display.flipScreenVertically();
    heltec_setup();
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT_PULLUP);
    initSensor(&sensor, TRIG_PIN, ECHO_PIN);
}

// Initialize Sensor Struct
void initSensor(UltrasonicSensor* sensor, int trig, int echo) {
    sensor->trigPin = trig;
    sensor->echoPin = echo;
    sensor->lastValidDistance = 0.0;
}

// Measure Distance with Improved Timing
float measureDistance(UltrasonicSensor* sensor) {
    digitalWrite(sensor->trigPin, LOW);
    delayMicroseconds(2);  // Ensure stable trigger
    digitalWrite(sensor->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor->trigPin, LOW);

    long duration = pulseIn(sensor->echoPin, HIGH, TIMEOUT_US); // Capture pulse duration
    float distance = (duration * 0.0343) / 2; // Convert to cm

    // Ensure valid reading
    if (distance == 0 || distance > TANK_HEIGHT_CM) {
        return sensor->lastValidDistance;
    }

    sensor->lastValidDistance = distance; // Update valid reading
    return distance;
}

// Get Stable Distance using Median Filter
float getStableDistance(UltrasonicSensor* sensor) {
    float readings[SAMPLES];

    // Collect multiple samples
    for (int i = 0; i < SAMPLES; i++) {
        readings[i] = measureDistance(sensor);
        delay(20); // Small delay to prevent interference
    }

    // Apply median filtering
    return medianFilter(readings, SAMPLES);
}

// Median Filter to Remove Outliers
float medianFilter(float values[], int size) {
    // Sort the array (Bubble Sort for small sample size)
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (values[j] > values[j + 1]) {
                float temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }
    // Return the median value
    return (size % 2 == 0) ? (values[size / 2 - 1] + values[size / 2]) / 2.0 : values[size / 2];
}

// Calculate and Display Water Level
void calculateWaterLevel(UltrasonicSensor* sensor) {
    float distance = getStableDistance(sensor);

    float waterLevel = ((TANK_HEIGHT_CM - distance) / TANK_HEIGHT_CM) * 100;
    waterLevel = constrain(waterLevel, 0, 100);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 10, "Distance: " + String(distance, 1) + "cm");
    display.drawString(64, 30, "Water Level: " + String(waterLevel, 1) + "%");
    display.display();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("cm | Water level: ");
    Serial.print(waterLevel);
    Serial.println("%");
}

void loop() {
    heltec_loop();
    calculateWaterLevel(&sensor);
    delay(500);
}
