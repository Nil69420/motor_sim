#pragma once

#include "driver/MotorSimulator.hpp"
#include "protocol/FeetechProtocol.hpp"
#include "transport/ITransport.hpp"
#include <memory>
#include <functional>

namespace motor_sim {
namespace driver {

class FeetechActuator {
public:
    using ErrorCallback = std::function<void(uint8_t error_code, const std::string& message)>;
    using PositionCallback = std::function<void(uint16_t position)>;


    FeetechActuator(uint8_t id, 
                   std::shared_ptr<transport::ITransport> transport,
                   std::shared_ptr<MotorSimulator> simulator);

    // === Connection & Initialization ===

    /**
     * @brief Ping the motor to check connection
     */
    bool ping();

    /**
     * @brief Initialize motor with default settings
     */
    bool initialize();

    /**
     * @brief Factory reset
     */
    bool reset();

    // === Position Control ===

    /**
     * @brief Set goal position (0-4095)
     * @param position Target position
     * @param wait_for_completion Block until move completes
     */
    bool setPosition(uint16_t position, bool wait_for_completion = false);

    /**
     * @brief Get current position
     */
    std::optional<uint16_t> getPosition();

    /**
     * @brief Set position in degrees (0-360)
     */
    bool setPositionDegrees(float degrees, bool wait_for_completion = false);

    /**
     * @brief Get position in degrees
     */
    std::optional<float> getPositionDegrees();

    /**
     * @brief Wait until motor reaches target position
     * @param timeout Maximum wait time
     * @param tolerance Position error tolerance
     */
    bool waitForPosition(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000),
                        uint16_t tolerance = 10);

    // === Velocity Control ===

    /**
     * @brief Set moving speed (0-1023)
     * @param speed Speed value (0 = max speed)
     */
    bool setSpeed(uint16_t speed);

    /**
     * @brief Get current speed
     */
    std::optional<uint16_t> getSpeed();

    /**
     * @brief Set speed in RPM
     */
    bool setSpeedRPM(float rpm);

    // === Torque Control ===

    /**
     * @brief Enable/disable torque
     */
    bool setTorqueEnable(bool enable);

    /**
     * @brief Set torque limit (0-1023)
     */
    bool setTorqueLimit(uint16_t limit);

    /**
     * @brief Get current load
     */
    std::optional<uint16_t> getLoad();

    // === Configuration ===

    /**
     * @brief Set angle limits for position mode
     * @param cw_limit Clockwise limit
     * @param ccw_limit Counter-clockwise limit
     * Set both to 0 for wheel mode (continuous rotation)
     */
    bool setAngleLimits(uint16_t cw_limit, uint16_t ccw_limit);

    /**
     * @brief Set LED state
     */
    bool setLED(bool on);

    /**
     * @brief Change motor ID
     */
    bool setID(uint8_t new_id);

    // === Status ===

    /**
     * @brief Get motor temperature (Celsius)
     */
    std::optional<uint8_t> getTemperature();

    /**
     * @brief Get input voltage (Volts * 10)
     */
    std::optional<uint8_t> getVoltage();

    /**
     * @brief Check if motor is moving
     */
    std::optional<bool> isMoving();

    /**
     * @brief Get complete motor state
     */
    std::optional<MotorState> getState();

    // === Callbacks ===

    /**
     * @brief Set error callback
     */
    void setErrorCallback(ErrorCallback callback) {
        error_callback_ = callback;
    }

    /**
     * @brief Set position update callback
     */
    void setPositionCallback(PositionCallback callback) {
        position_callback_ = callback;
    }

    // === Simulation Control ===

    /**
     * @brief Update simulator (call periodically if using simulation)
     */
    void updateSimulation(float dt);

    /**
     * @brief Get simulator instance
     */
    std::shared_ptr<MotorSimulator> getSimulator() const {
        return simulator_;
    }

    /**
     * @brief Check if using simulation
     */
    bool isSimulated() const {
        return simulator_ != nullptr;
    }

    // === Getters ===

    uint8_t getId() const { return id_; }

private:
    uint8_t id_;
    std::shared_ptr<protocol::FeetechProtocol> protocol_;
    std::shared_ptr<MotorSimulator> simulator_;  // nullptr for real hardware
    
    ErrorCallback error_callback_;
    PositionCallback position_callback_;

    void handleError(uint8_t error_code, const std::string& context);
    bool writeRegister(protocol::Register reg, uint8_t value);
    bool writeRegister(protocol::Register reg, uint16_t value);
    std::optional<uint8_t> readRegister(protocol::Register reg);
    std::optional<uint16_t> readRegister16(protocol::Register reg);
};

} // namespace driver
} // namespace motor_sim
