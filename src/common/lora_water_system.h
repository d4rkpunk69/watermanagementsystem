// lora_water_system.h

#include <Arduino.h>
#pragma once
// Network Configuration
#define LORA_SYNC_WORD 0xF3
#define LORA_BAND 915.0
#define NODE_TIMEOUT 300000 // 5 minutes

// Packet Structure
#pragma pack(push, 1)
struct LoraPacket {
    uint8_t nodeID;
    float distance_cm;
    float battery_v;
    uint8_t checksum;
};
#pragma pack(pop)

// Utility Functions
uint8_t calculateChecksum(LoraPacket& packet) {
    uint8_t* data = (uint8_t*)&packet;
    uint8_t sum = 0;
    for(size_t i = 0; i < sizeof(packet) - 1; i++) {
        sum ^= data[i];
    }
    return sum;
}
