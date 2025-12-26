#pragma once

#include "driver/MotorState.hpp"
#include <chrono>
#include <mutex>
#include <random>

namespace motor_sim {
namespace driver {


class MotorSimulator {
public:
    struct PhysicsConfig {
        float max_acceleration{2000.0f};     // Positions/s^2
        float max_velocity{1000.0f};         // Positions/s
        float position_noise{0.5f};          // Position reading noise (stddev)
        float thermal_coefficient{0.01f};    // Temperature rise per load
        float thermal_dissipation{0.1f};     // Cooling rate
        float voltage_noise{0.5f};           // Voltage fluctuation
        float friction_coefficient{0.05f};   // Static friction
        float inertia{0.01f};               // Rotational inertia
    };

    explicit MotorSimulator(const MotorState& initial_state, const PhysicsConfig& config);
    explicit MotorSimulator(const MotorState& initial_state = MotorState());

    /**
     * @brief Update simulation by one time step
     * @param dt Time step in seconds
     */
    void update(float dt);

    /**
     * @brief Get current motor state
     */
    MotorState getState() const;

    /**
     * @brief Set goal position
     */
    void setGoalPosition(uint16_t position);

    /**
     * @brief Set goal speed
     */
    void setGoalSpeed(uint16_t speed);

    /**
     * @brief Enable/disable torque
     */
    void setTorqueEnable(bool enable);

    /**
     * @brief Set torque limit
     */
    void setTorqueLim(uint16_t limit);

    /**
     * @brief Set angle limits
     */
    void setAngleLimits(uint16_t cw, uint16_t ccw);

    /**
     * @brief Get memory register value
     */
    uint8_t readRegister(uint8_t address) const;

    /**
     * @brief Set memory register value
     */
    void writeRegister(uint8_t address, uint8_t value);

    /**
     * @brief Read multiple registers
     */
    std::vector<uint8_t> readRegisters(uint8_t start_address, uint8_t length) const;

    /**
     * @brief Write multiple registers
     */
    void writeRegisters(uint8_t start_address, const std::vector<uint8_t>& data);

    /**
     * @brief Reset to factory defaults
     */
    void reset();

    /**
     * @brief Inject error condition
     */
    void injectError(ErrorFlag error);

    /**
     * @brief Clear error flags
     */
    void clearErrors();

    /**
     * @brief Check if motor is moving
     */
    bool isMoving() const;

private:
    MotorState state_;
    PhysicsConfig config_;
    
    // Simulation state
    float current_velocity_{0.0f};       // Current velocity in positions/s
    float current_acceleration_{0.0f};   // Current acceleration
    float actual_position_{2048.0f};     // Continuous position (with sub-position precision)
    
    std::chrono::steady_clock::time_point last_update_;
    
    mutable std::mutex mutex_;
    std::mt19937 rng_;
    std::normal_distribution<float> noise_dist_;

    // Physics simulation
    void updatePosition(float dt);
    void updateVelocity(float dt);
    void updateTemperature(float dt);
    void updateVoltage(float dt);
    void updateLoad();
    void checkLimits();

    // Helper functions
    float calculateTargetVelocity() const;
    float applyAccelerationLimit(float desired_velocity, float dt) const;
    float addNoise(float value, float noise_stddev);
    uint16_t clampPosition(float position) const;
};

} // namespace driver
} // namespace motor_sim
