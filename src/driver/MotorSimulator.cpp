#include "driver/MotorSimulator.hpp"
#include "protocol/ProtocolPacket.hpp"
#include "utils/Logger.hpp"
#include <algorithm>
#include <cmath>

namespace motor_sim {
namespace driver {

MotorSimulator::MotorSimulator(const MotorState& initial_state, const PhysicsConfig& config)
    : state_(initial_state)
    , config_(config)
    , current_velocity_(0.0f)
    , current_acceleration_(0.0f)
    , actual_position_(initial_state.present_position)
    , last_update_(std::chrono::steady_clock::now())
    , rng_(std::random_device{}())
    , noise_dist_(0.0f, 1.0f) {
    
    if (state_.model_number == 0) {
        state_.model_number = 0x000F;  // STS3032 example
    }
    if (state_.firmware_version == 0) {
        state_.firmware_version = 0x24;
    }
}

MotorSimulator::MotorSimulator(const MotorState& initial_state)
    : MotorSimulator(initial_state, PhysicsConfig{}) {
}

void MotorSimulator::update(float dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!state_.torque_enable) {
        current_velocity_ *= (1.0f - config_.friction_coefficient * dt); //friction
        if (std::abs(current_velocity_) < 0.1f) {
            current_velocity_ = 0.0f;
        }
        actual_position_ += current_velocity_ * dt;
        state_.present_position = clampPosition(actual_position_);
        state_.moving = false;
        return;
    }

    updateVelocity(dt);
    updatePosition(dt);
    updateLoad();
    updateTemperature(dt);
    updateVoltage(dt);
    checkLimits();
    
    state_.moving = std::abs(current_velocity_) > 1.0f || 
                   std::abs(static_cast<int>(state_.present_position) - 
                           static_cast<int>(state_.goal_position)) > 10;
}

MotorState MotorSimulator::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void MotorSimulator::setGoalPosition(uint16_t position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_.isWheelMode()) {
        LOG_WARN("MotorSim: Cannot set position in wheel mode");
        return;
    }
    
    if (!state_.isPositionValid(position)) {
        state_.error = state_.error | ErrorFlag::ANGLE_LIMIT;
        LOG_WARN("MotorSim: Goal position out of limits");
        return;
    }
    
    state_.goal_position = position;
    LOG_DEBUG("MotorSim: Goal position set to ", position);
}

void MotorSimulator::setGoalSpeed(uint16_t speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.goal_speed = speed & 0x3FF;  // 10-bit value
    LOG_DEBUG("MotorSim: Goal speed set to ", state_.goal_speed);
}

void MotorSimulator::setTorqueEnable(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.torque_enable = enable;
    
    if (!enable) {
        current_velocity_ = 0.0f;
        current_acceleration_ = 0.0f;
        state_.moving = false;
    }
    
    LOG_DEBUG("MotorSim: Torque ", (enable ? "enabled" : "disabled"));
}

void MotorSimulator::setTorqueLim(uint16_t limit) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.torque_limit = std::min(limit, state_.max_torque);
}

void MotorSimulator::setAngleLimits(uint16_t cw, uint16_t ccw) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.cw_angle_limit = cw;
    state_.ccw_angle_limit = ccw;
    
    if (cw == 0 && ccw == 0) {
        state_.mode = OperatingMode::VELOCITY;
        LOG_INFO("MotorSim: Switched to wheel mode");
    } else {
        state_.mode = OperatingMode::POSITION;
        LOG_INFO("MotorSim: Switched to position mode");
    }
}

void MotorSimulator::updatePosition(float dt) {
    actual_position_ += current_velocity_ * dt;
    
    // Apply position limits in position mode
    if (!state_.isWheelMode()) {
        if (actual_position_ < state_.cw_angle_limit) {
            actual_position_ = state_.cw_angle_limit;
            current_velocity_ = 0.0f;
        } else if (actual_position_ > state_.ccw_angle_limit) {
            actual_position_ = state_.ccw_angle_limit;
            current_velocity_ = 0.0f;
        }
    } else {
        // Wrap around in wheel mode
        while (actual_position_ >= 4096.0f) actual_position_ -= 4096.0f;
        while (actual_position_ < 0.0f) actual_position_ += 4096.0f;
    }
    
    // Add measurement noise
    float noisy_position = addNoise(actual_position_, config_.position_noise);
    state_.present_position = clampPosition(noisy_position);
}

void MotorSimulator::updateVelocity(float dt) {
    float target_velocity = calculateTargetVelocity();
    
    // Apply acceleration limits
    current_velocity_ = applyAccelerationLimit(target_velocity, dt);
    
    // Update present speed (magnitude)
    uint16_t speed_magnitude = static_cast<uint16_t>(std::abs(current_velocity_));
    speed_magnitude = std::min(speed_magnitude, static_cast<uint16_t>(1023));
    
    if (current_velocity_ < 0) {
        state_.present_speed = speed_magnitude | 0x400;
    } else {
        state_.present_speed = speed_magnitude;
    }
}

void MotorSimulator::updateTemperature(float dt) {
    // Temperature rises with load
    float load_ratio = state_.getLoadPercentage() / 100.0f;
    float temp_rise = load_ratio * config_.thermal_coefficient * 100.0f * dt;
    
    float ambient = 25.0f;
    float temp_drop = (state_.present_temperature - ambient) * 
                     config_.thermal_dissipation * dt;
    
    state_.present_temperature += static_cast<uint8_t>(temp_rise - temp_drop);
    
    state_.present_temperature = std::max(static_cast<uint8_t>(20), 
                                         std::min(static_cast<uint8_t>(85), 
                                                 state_.present_temperature));
}

void MotorSimulator::updateVoltage(float dt) {
    (void)dt;  
    // Simulate voltage fluctuations
    float base_voltage = 7.5f;  // 7.5V nominal
    float noise = addNoise(0.0f, config_.voltage_noise);
    float voltage = base_voltage + noise * 0.1f;
    
    state_.present_voltage = static_cast<uint8_t>(voltage * 10.0f);
}

void MotorSimulator::updateLoad() {
    // Calculate load based on acceleration and velocity
    float load_from_accel = std::abs(current_acceleration_) / config_.max_acceleration;
    float load_from_velocity = std::abs(current_velocity_) / config_.max_velocity;
    float load_from_position = 0.0f;
    
    if (!state_.isWheelMode()) {
        float position_error = std::abs(static_cast<int>(state_.goal_position) - 
                                       static_cast<int>(state_.present_position));
        load_from_position = position_error / 2048.0f;  // Normalize
    }
    
    float total_load = (load_from_accel + load_from_velocity + load_from_position) / 3.0f;
    total_load = std::min(1.0f, total_load);
    
    uint16_t load_value = static_cast<uint16_t>(total_load * 1023.0f);
    
    if (current_velocity_ < 0) {
        state_.present_load = load_value | 0x400;
    } else {
        state_.present_load = load_value;
    }
}

void MotorSimulator::checkLimits() {
    // Temperature limit
    if (state_.present_temperature > state_.temperature_limit) {
        state_.error = state_.error | ErrorFlag::OVERHEATING;
        state_.torque_enable = false;
        LOG_ERROR("MotorSim: Overheating detected!");
    }
    
    // Voltage limits
    if (state_.present_voltage < state_.min_voltage || 
        state_.present_voltage > state_.max_voltage) {
        state_.error = state_.error | ErrorFlag::VOLTAGE;
        LOG_WARN("MotorSim: Voltage out of range");
    }
    
    // Overload
    if (state_.getLoadPercentage() > 90.0f) {
        state_.error = state_.error | ErrorFlag::OVERLOAD;
        LOG_WARN("MotorSim: Overload detected");
    }
}

float MotorSimulator::calculateTargetVelocity() const {
    if (state_.isWheelMode()) {
        // In wheel mode, goal_speed directly controls velocity
        float speed = state_.goal_speed & 0x3FF;
        bool is_ccw = (state_.goal_speed & 0x400) != 0;
        return is_ccw ? -speed : speed;
    } else {
        // In position mode, calculate velocity needed to reach goal
        float position_error = state_.goal_position - actual_position_;
        
        // P controller with speed limit
        float kp = 5.0f;
        float desired_velocity = position_error * kp;
        
        // Apply speed limit
        float max_speed = (state_.goal_speed == 0) ? config_.max_velocity : 
                         static_cast<float>(state_.goal_speed);
        desired_velocity = std::max(-max_speed, std::min(max_speed, desired_velocity));
        
        return desired_velocity;
    }
}

float MotorSimulator::applyAccelerationLimit(float desired_velocity, float dt) const {
    float velocity_change = desired_velocity - current_velocity_;
    float max_change = config_.max_acceleration * dt;
    
    velocity_change = std::max(-max_change, std::min(max_change, velocity_change));
    
    return current_velocity_ + velocity_change;
}

float MotorSimulator::addNoise(float value, float noise_stddev) {
    if (noise_stddev <= 0.0f) return value;
    return value + noise_dist_(rng_) * noise_stddev;
}

uint16_t MotorSimulator::clampPosition(float position) const {
    int pos = static_cast<int>(std::round(position));
    return static_cast<uint16_t>(std::max(0, std::min(4095, pos)));
}

uint8_t MotorSimulator::readRegister(uint8_t address) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    using protocol::Register;
    
    switch (static_cast<Register>(address)) {
        case Register::MODEL_NUMBER_L: return state_.model_number & 0xFF;
        case Register::MODEL_NUMBER_H: return (state_.model_number >> 8) & 0xFF;
        case Register::FIRMWARE_VERSION: return state_.firmware_version;
        case Register::ID: return state_.id;
        case Register::TORQUE_ENABLE: return state_.torque_enable ? 1 : 0;
        case Register::LED: return state_.led ? 1 : 0;
        case Register::GOAL_POSITION_L: return state_.goal_position & 0xFF;
        case Register::GOAL_POSITION_H: return (state_.goal_position >> 8) & 0xFF;
        case Register::GOAL_SPEED_L: return state_.goal_speed & 0xFF;
        case Register::GOAL_SPEED_H: return (state_.goal_speed >> 8) & 0xFF;
        case Register::PRESENT_POSITION_L: return state_.present_position & 0xFF;
        case Register::PRESENT_POSITION_H: return (state_.present_position >> 8) & 0xFF;
        case Register::PRESENT_SPEED_L: return state_.present_speed & 0xFF;
        case Register::PRESENT_SPEED_H: return (state_.present_speed >> 8) & 0xFF;
        case Register::PRESENT_LOAD_L: return state_.present_load & 0xFF;
        case Register::PRESENT_LOAD_H: return (state_.present_load >> 8) & 0xFF;
        case Register::PRESENT_VOLTAGE: return state_.present_voltage;
        case Register::PRESENT_TEMPERATURE: return state_.present_temperature;
        case Register::MOVING: return state_.moving ? 1 : 0;
        case Register::TEMPERATURE_LIMIT: return state_.temperature_limit;
        case Register::CW_ANGLE_LIMIT_L: return state_.cw_angle_limit & 0xFF;
        case Register::CW_ANGLE_LIMIT_H: return (state_.cw_angle_limit >> 8) & 0xFF;
        case Register::CCW_ANGLE_LIMIT_L: return state_.ccw_angle_limit & 0xFF;
        case Register::CCW_ANGLE_LIMIT_H: return (state_.ccw_angle_limit >> 8) & 0xFF;
        default:
            LOG_WARN("MotorSim: Read from unknown register ", static_cast<int>(address));
            return 0;
    }
}

void MotorSimulator::writeRegister(uint8_t address, uint8_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    using protocol::Register;
    
    switch (static_cast<Register>(address)) {
        case Register::ID:
            state_.id = value;
            break;
        case Register::TORQUE_ENABLE:
            state_.torque_enable = value != 0;
            break;
        case Register::LED:
            state_.led = value != 0;
            break;
        case Register::GOAL_POSITION_L:
            state_.goal_position = (state_.goal_position & 0xFF00) | value;
            break;
        case Register::GOAL_POSITION_H:
            state_.goal_position = (state_.goal_position & 0x00FF) | (static_cast<uint16_t>(value) << 8);
            break;
        case Register::GOAL_SPEED_L:
            state_.goal_speed = (state_.goal_speed & 0xFF00) | value;
            break;
        case Register::GOAL_SPEED_H:
            state_.goal_speed = (state_.goal_speed & 0x00FF) | (static_cast<uint16_t>(value) << 8);
            break;
        case Register::CW_ANGLE_LIMIT_L:
            state_.cw_angle_limit = (state_.cw_angle_limit & 0xFF00) | value;
            break;
        case Register::CW_ANGLE_LIMIT_H:
            state_.cw_angle_limit = (state_.cw_angle_limit & 0x00FF) | (static_cast<uint16_t>(value) << 8);
            break;
        case Register::CCW_ANGLE_LIMIT_L:
            state_.ccw_angle_limit = (state_.ccw_angle_limit & 0xFF00) | value;
            break;
        case Register::CCW_ANGLE_LIMIT_H:
            state_.ccw_angle_limit = (state_.ccw_angle_limit & 0x00FF) | (static_cast<uint16_t>(value) << 8);
            break;
        case Register::TEMPERATURE_LIMIT:
            state_.temperature_limit = value;
            break;
        default:
            LOG_WARN("MotorSim: Write to unknown/readonly register ", static_cast<int>(address));
            break;
    }
}

std::vector<uint8_t> MotorSimulator::readRegisters(uint8_t start_address, uint8_t length) const {
    std::vector<uint8_t> data;
    data.reserve(length);
    
    for (uint8_t i = 0; i < length; ++i) {
        data.push_back(readRegister(start_address + i));
    }
    
    return data;
}

void MotorSimulator::writeRegisters(uint8_t start_address, const std::vector<uint8_t>& data) {
    for (size_t i = 0; i < data.size(); ++i) {
        writeRegister(start_address + i, data[i]);
    }
}

void MotorSimulator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = MotorState();
    current_velocity_ = 0.0f;
    current_acceleration_ = 0.0f;
    actual_position_ = 2048.0f;
    LOG_INFO("MotorSim: Reset to factory defaults");
}

void MotorSimulator::injectError(ErrorFlag error) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.error = state_.error | error;
}

void MotorSimulator::clearErrors() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.error = ErrorFlag::NONE;
}

bool MotorSimulator::isMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.moving;
}

} // namespace driver
} // namespace motor_sim
