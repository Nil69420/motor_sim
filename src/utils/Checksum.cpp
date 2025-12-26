#include "utils/Checksum.hpp"

namespace motor_sim {
namespace utils {

uint8_t Checksum::calculate8bit(const uint8_t* data, size_t length) {
    if (length == 0) return 0;
    
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    
    return ~sum;  
}

bool Checksum::validate8bit(const uint8_t* data, size_t length, uint8_t checksum) {
    return calculate8bit(data, length) == checksum;
}

uint16_t Checksum::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0;
    
    for (size_t i = 0; i < length; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

bool Checksum::validateCRC16(const uint8_t* data, size_t length, uint16_t crc) {
    return calculateCRC16(data, length) == crc;
}

} // namespace utils
} // namespace motor_sim
