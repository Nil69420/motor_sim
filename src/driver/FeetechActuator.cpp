#include "driver/FeetechActuator.hpp"
#include "utils/Logger.hpp"
#include <thread>

namespace motor_sim {
namespace driver {

FeetechActuator::FeetechActuator(uint8_t id, 
                                std::shared_ptr<transport::ITransport> transport,
                                std::shared_ptr<MotorSimulator> simulator)
    : id_(id)
    , protocol_(std::make_shared<protocol::FeetechProtocol>(transport))
    , simulator_(simulator) {
    
    if (!simulator_) {
        throw std::invalid_argument("FeetechActuator requires a MotorSimulator (simulation-only mode)");
    }
}

bool FeetechActuator::ping() {
    bool result = protocol_->ping(id_);
    if (!result) {
        handleError(protocol_->getLastError(), "Ping failed");
    }
    return result;
}

bool FeetechActuator::initialize() {
    LOG_INFO("FeetechActuator: Initializing motor ID ", static_cast<int>(id_));
    
    if (!ping()) {
        LOG_ERROR("FeetechActuator: Motor not responding");
        return false;
    }
    
    if (!setAngleLimits(0, 4095)) {
        return false;
    }
    
    if (!setSpeed(100)) {
        return false;
    }
    
    LOG_INFO("FeetechActuator: Initialization complete");
    return true;
}

bool FeetechActuator::reset() {
    auto packet = protocol::ProtocolPacket::createInstruction(
        id_, protocol::Instruction::RESET);
    auto response = protocol_->sendAndReceive(packet);
    
    if (simulator_) {
        simulator_->reset();
    }
    
    return response.has_value();
}

bool FeetechActuator::setPosition(uint16_t position, bool wait_for_completion) {
    if (!writeRegister(protocol::Register::GOAL_POSITION_L, position)) {
        handleError(protocol_->getLastError(), "Failed to set position");
        return false;
    }
    
    if (simulator_) {
        simulator_->setGoalPosition(position);
    }
    
    if (wait_for_completion) {
        return waitForPosition();
    }
    
    return true;
}

std::optional<uint16_t> FeetechActuator::getPosition() {
    return readRegister16(protocol::Register::PRESENT_POSITION_L);
}

bool FeetechActuator::setPositionDegrees(float degrees, bool wait_for_completion) {
    uint16_t position = static_cast<uint16_t>((degrees / 360.0f) * 4095.0f);
    return setPosition(position, wait_for_completion);
}

std::optional<float> FeetechActuator::getPositionDegrees() {
    auto position = getPosition();
    if (position) {
        return (*position / 4095.0f) * 360.0f;
    }
    return std::nullopt;
}

bool FeetechActuator::waitForPosition(std::chrono::milliseconds timeout, uint16_t tolerance) {
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        
        if (elapsed > timeout) {
            LOG_WARN("FeetechActuator: Position wait timeout");
            return false;
        }
        
        auto moving = isMoving();
        if (moving && !(*moving)) {
            return true;
        }
        
        auto current_pos = getPosition();
        if (!current_pos) {
            return false;
        }
        
        auto goal_pos_opt = readRegister16(protocol::Register::GOAL_POSITION_L);
        if (goal_pos_opt) {
            uint16_t error = std::abs(static_cast<int>(*current_pos) - 
                                     static_cast<int>(*goal_pos_opt));
            if (error <= tolerance) {
                return true;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool FeetechActuator::setSpeed(uint16_t speed) {
    if (!writeRegister(protocol::Register::GOAL_SPEED_L, speed)) {
        handleError(protocol_->getLastError(), "Failed to set speed");
        return false;
    }
    
    if (simulator_) {
        simulator_->setGoalSpeed(speed);
    }
    
    return true;
}

std::optional<uint16_t> FeetechActuator::getSpeed() {
    return readRegister16(protocol::Register::PRESENT_SPEED_L);
}

bool FeetechActuator::setSpeedRPM(float rpm) {
    // Feetech: 1 unit ≈ 0.111 RPM
    uint16_t speed = static_cast<uint16_t>(rpm / 0.111f);
    return setSpeed(speed);
}

bool FeetechActuator::setTorqueEnable(bool enable) {
    if (!writeRegister(protocol::Register::TORQUE_ENABLE, static_cast<uint8_t>(enable ? 1 : 0))) {
        handleError(protocol_->getLastError(), "Failed to set torque enable");
        return false;
    }
    
    if (simulator_) {
        simulator_->setTorqueEnable(enable);
    }
    
    return true;
}

bool FeetechActuator::setTorqueLimit(uint16_t limit) {
    return writeRegister(protocol::Register::TORQUE_LIMIT_L, limit);
}

std::optional<uint16_t> FeetechActuator::getLoad() {
    return readRegister16(protocol::Register::PRESENT_LOAD_L);
}

bool FeetechActuator::setAngleLimits(uint16_t cw_limit, uint16_t ccw_limit) {
    bool success = writeRegister(protocol::Register::CW_ANGLE_LIMIT_L, cw_limit);
    success &= writeRegister(protocol::Register::CCW_ANGLE_LIMIT_L, ccw_limit);
    
    if (simulator_) {
        simulator_->setAngleLimits(cw_limit, ccw_limit);
    }
    
    return success;
}

bool FeetechActuator::setLED(bool on) {
    return writeRegister(protocol::Register::LED, static_cast<uint8_t>(on ? 1 : 0));
}

bool FeetechActuator::setID(uint8_t new_id) {
    if (writeRegister(protocol::Register::ID, new_id)) {
        id_ = new_id;
        return true;
    }
    return false;
}

std::optional<uint8_t> FeetechActuator::getTemperature() {
    return readRegister(protocol::Register::PRESENT_TEMPERATURE);
}

std::optional<uint8_t> FeetechActuator::getVoltage() {
    return readRegister(protocol::Register::PRESENT_VOLTAGE);
}

std::optional<bool> FeetechActuator::isMoving() {
    auto moving = readRegister(protocol::Register::MOVING);
    if (moving) {
        return *moving != 0;
    }
    return std::nullopt;
}

std::optional<MotorState> FeetechActuator::getState() {
    // Always use simulator (simulation-only mode)
    return simulator_->getState();
}

void FeetechActuator::updateSimulation(float dt) {
    simulator_->update(dt);
    
    if (position_callback_) {
        auto state = simulator_->getState();
        position_callback_(state.present_position);
    }
}

void FeetechActuator::handleError(uint8_t error_code, const std::string& context) {
    if (error_code != 0 && error_callback_) {
        error_callback_(error_code, context);
    }
    
    if (error_code != 0) {
        LOG_ERROR("FeetechActuator: Error 0x", std::hex, static_cast<int>(error_code),
                 " in ", context);
    }
}

bool FeetechActuator::writeRegister(protocol::Register reg, uint8_t value) {
    return protocol_->writeByte(id_, reg, value);
}

bool FeetechActuator::writeRegister(protocol::Register reg, uint16_t value) {
    return protocol_->writeWord(id_, reg, value);
}

std::optional<uint8_t> FeetechActuator::readRegister(protocol::Register reg) {
    return protocol_->readByte(id_, reg);
}

std::optional<uint16_t> FeetechActuator::readRegister16(protocol::Register reg) {
    return protocol_->readWord(id_, reg);
}

} // namespace driver
} // namespace motor_sim
