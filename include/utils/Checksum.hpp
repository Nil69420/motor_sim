#pragma once

#include <cstdint>
#include <cstddef>

namespace motor_sim {
namespace utils {


class Checksum {
public:
    /**
     * @brief Calculate simple 8-bit checksum (used by Feetech protocol)
     * 
     * Formula: ~(ID + Length + Instruction + Parameters...)
     */
    static uint8_t calculate8bit(const uint8_t* data, size_t length);

    /**
     * @brief Validate 8-bit checksum
     */
    static bool validate8bit(const uint8_t* data, size_t length, uint8_t checksum);

    /**
     * @brief Calculate CRC16-CCITT (alternative robust checksum)
     */
    static uint16_t calculateCRC16(const uint8_t* data, size_t length);

    /**
     * @brief Validate CRC16
     */
    static bool validateCRC16(const uint8_t* data, size_t length, uint16_t crc);

private:
    static constexpr uint16_t CRC16_POLY = 0x1021;
};

} // namespace utils
} // namespace motor_sim
