#pragma once

#include <cstdint>
#include <string>

namespace motor_sim {
namespace driver {

/**
 * @brief Motor operating mode
 */
enum class OperatingMode {
    POSITION,    // Position control mode
    VELOCITY,    // Continuous rotation mode
    TORQUE       // Torque control mode (if supported)
};

/**
 * @brief Motor error flags
 */
enum class ErrorFlag : uint8_t {
    NONE = 0x00,
    VOLTAGE = 0x01,          // Input voltage error
    ANGLE_LIMIT = 0x02,      // Goal position out of range
    OVERHEATING = 0x04,      // Temperature limit exceeded
    RANGE = 0x08,            // Command out of range
    CHECKSUM = 0x10,         // Checksum error
    OVERLOAD = 0x20,         // Overload error
    INSTRUCTION = 0x40       // Undefined instruction
};

inline ErrorFlag operator|(ErrorFlag a, ErrorFlag b) {
    return static_cast<ErrorFlag>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline ErrorFlag operator&(ErrorFlag a, ErrorFlag b) {
    return static_cast<ErrorFlag>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

/**
 * @brief Complete motor state representation
 */
struct MotorState {
    // Identity
    uint8_t id{1};
    uint16_t model_number{0};
    uint8_t firmware_version{0};

    // Configuration (EEPROM)
    uint32_t baud_rate{115200};
    uint8_t return_delay_time{250};  // Microseconds / 2
    uint16_t cw_angle_limit{0};      // Clockwise angle limit
    uint16_t ccw_angle_limit{4095};  // Counter-clockwise angle limit
    uint8_t temperature_limit{70};   // Celsius
    uint8_t min_voltage{60};         // Volts * 10
    uint8_t max_voltage{90};         // Volts * 10
    uint16_t max_torque{1023};       // Max torque value

    // Control (RAM)
    bool torque_enable{false};
    bool led{false};
    uint16_t goal_position{2048};    // 0-4095 (12-bit)
    uint16_t goal_speed{0};          // 0-1023
    uint16_t torque_limit{1023};

    // Status (RAM - read-only)
    uint16_t present_position{2048};
    uint16_t present_speed{0};
    uint16_t present_load{0};        // 0-1023, bit 10 = direction
    uint8_t present_voltage{75};     // Volts * 10
    uint8_t present_temperature{35}; // Celsius
    bool moving{false};
    bool registered{false};
    bool locked{false};

    // Error state
    ErrorFlag error{ErrorFlag::NONE};
    
    // Operating mode
    OperatingMode mode{OperatingMode::POSITION};

    /**
     * @brief Check if position is within angle limits
     */
    bool isPositionValid(uint16_t position) const {
        if (cw_angle_limit == 0 && ccw_angle_limit == 0) {
            return true;  // Wheel mode - no limits
        }
        return position >= cw_angle_limit && position <= ccw_angle_limit;
    }

    /**
     * @brief Check if in wheel mode (continuous rotation)
     */
    bool isWheelMode() const {
        return cw_angle_limit == 0 && ccw_angle_limit == 0;
    }

    /**
     * @brief Convert position to degrees (0-4095 -> 0-360)
     */
    float positionToDegrees(uint16_t position) const {
        return (position / 4095.0f) * 360.0f;
    }

    /**
     * @brief Convert degrees to position (0-360 -> 0-4095)
     */
    uint16_t degreesToPosition(float degrees) const {
        return static_cast<uint16_t>((degrees / 360.0f) * 4095.0f);
    }

    /**
     * @brief Convert speed to RPM
     */
    float speedToRPM(uint16_t speed) const {
        // Feetech: 1 unit = 0.111 RPM (approximately)
        return (speed & 0x3FF) * 0.111f;
    }

    /**
     * @brief Get load percentage
     */
    float getLoadPercentage() const {
        return ((present_load & 0x3FF) / 1023.0f) * 100.0f;
    }

    /**
     * @brief Check if load direction is CCW
     */
    bool isLoadCCW() const {
        return (present_load & 0x400) != 0;
    }
};

} // namespace driver
} // namespace motor_sim
