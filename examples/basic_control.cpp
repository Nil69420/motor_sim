#include "driver/FeetechActuator.hpp"
#include "driver/MotorSimulator.hpp"
#include "transport/MockSerialBuffer.hpp"
#include "utils/Logger.hpp"
#include <iostream>
#include <thread>
#include <memory>

using namespace motor_sim;


class DeviceSimulator {
public:
    DeviceSimulator(std::shared_ptr<transport::MockSerialBuffer> device_port,
                   std::shared_ptr<driver::MotorSimulator> motor)
        : device_port_(device_port)
        , motor_(motor)
        , running_(false) {
    }

    void start() {
        running_ = true;
        thread_ = std::thread(&DeviceSimulator::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    ~DeviceSimulator() {
        stop();
    }

private:
    void run() {
        using namespace protocol;
        
        auto last_update = std::chrono::steady_clock::now();
        
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last_update).count();
            if (dt > 0.01f) {  
                motor_->update(dt);
                last_update = now;
            }
            
            if (device_port_->available() >= ProtocolPacket::MIN_PACKET_SIZE) {
                std::vector<uint8_t> buffer(device_port_->available());
                size_t bytes_read = device_port_->read(buffer.data(), buffer.size());
                
                if (bytes_read > 0) {
                    auto result = ProtocolPacket::findPacket(buffer.data(), bytes_read);
                    if (result) {
                        auto& packet = result->first;
                        handlePacket(packet);
                    }
                }
            }
            
            // Small delay to prevent CPU spinning and allow other threads to run
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void handlePacket(const protocol::ProtocolPacket& packet) {
        using namespace protocol;
        
        uint8_t error = 0;
        std::vector<uint8_t> response_params;
        
        switch (packet.getInstruction()) {
            case Instruction::PING:
                break;
                
            case Instruction::READ_DATA:
                if (packet.getParams().size() >= 2) {
                    uint8_t start_addr = packet.getParams()[0];
                    uint8_t length = packet.getParams()[1];
                    response_params = motor_->readRegisters(start_addr, length);
                }
                break;
                
            case Instruction::WRITE_DATA:
                if (packet.getParams().size() >= 2) {
                    uint8_t start_addr = packet.getParams()[0];
                    std::vector<uint8_t> data(packet.getParams().begin() + 1, 
                                             packet.getParams().end());
                    motor_->writeRegisters(start_addr, data);
                }
                break;
                
            case Instruction::RESET:
                motor_->reset();
                break;
                
            default:
                error = 0x40;  // Undefined instruction
                break;
        }
        
        auto response = ProtocolPacket::createStatusResponse(packet.getId(), error, response_params);
        auto response_data = response.serialize();
        device_port_->write(response_data.data(), response_data.size());
    }

    std::shared_ptr<transport::MockSerialBuffer> device_port_;
    std::shared_ptr<driver::MotorSimulator> motor_;
    std::thread thread_;
    bool running_;
};

int main() {
    utils::Logger::getInstance().setLevel(utils::LogLevel::INFO);
    
    std::cout << "=== Feetech Motor Simulator - Basic Control Example ===" << std::endl;
    std::cout << std::endl;
    
    // Create connected serial buffers (simulates a serial cable)
    auto [host_port, device_port] = transport::createSerialPair();
    
    driver::MotorState initial_state;
    initial_state.id = 1;
    auto motor_sim = std::make_shared<driver::MotorSimulator>(initial_state);
    
    DeviceSimulator device(device_port, motor_sim);
    device.start();
    
    std::cout << "✓ Created virtual serial connection" << std::endl;
    std::cout << "✓ Started motor simulator" << std::endl;
    std::cout << std::endl;
    
    driver::FeetechActuator actuator(1, host_port, motor_sim);
    
    std::cout << "Testing connection..." << std::endl;
    if (actuator.ping()) {
        std::cout << "✓ Motor responding!" << std::endl;
    } else {
        std::cout << "✗ Motor not responding" << std::endl;
        return 1;
    }
    std::cout << std::endl;
    
    std::cout << "Initializing motor..." << std::endl;
    if (!actuator.initialize()) {
        std::cout << "✗ Initialization failed" << std::endl;
        return 1;
    }
    std::cout << std::endl;
    
    std::cout << "Enabling torque..." << std::endl;
    actuator.setTorqueEnable(true);
    std::cout << std::endl;
    
    std::cout << "=== Position Control Demo ===" << std::endl;
    std::cout << std::endl;
    
    std::vector<float> target_positions = {90.0f, 180.0f, 270.0f, 180.0f, 0.0f};
    
    for (float target_deg : target_positions) {
        std::cout << "Moving to " << target_deg << " degrees..." << std::endl;
        actuator.setPositionDegrees(target_deg, false);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        for (int i = 0; i < 20; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            if (i % 3 == 0) {
                auto pos = actuator.getPositionDegrees();
                if (pos) {
                    std::cout << "  Position: " << std::fixed << std::setprecision(1) << *pos << "°";
                }
            }
            if (i % 3 == 1) {
                auto temp = actuator.getTemperature();
                if (temp) {
                    std::cout << " | Temp: " << static_cast<int>(*temp) << "°C";
                }
            }
            if (i % 3 == 2) {
                auto load = actuator.getLoad();
                if (load) {
                    std::cout << " | Load: " << (*load & 0x3FF) << std::endl;
                }
            }
            
            auto moving = actuator.isMoving();
            if (moving && !(*moving)) {
                std::cout << std::endl << "  ✓ Position reached!" << std::endl;
                break;
            }
        }
        std::cout << std::endl;
    }
    
    std::cout << "=== Final Motor State ===" << std::endl;
    auto state = actuator.getState();
    if (state) {
        std::cout << "ID: " << static_cast<int>(state->id) << std::endl;
        std::cout << "Position: " << state->positionToDegrees(state->present_position) << "°" << std::endl;
        std::cout << "Speed: " << state->speedToRPM(state->present_speed) << " RPM" << std::endl;
        std::cout << "Temperature: " << static_cast<int>(state->present_temperature) << "°C" << std::endl;
        std::cout << "Voltage: " << (state->present_voltage / 10.0f) << "V" << std::endl;
        std::cout << "Load: " << state->getLoadPercentage() << "%" << std::endl;
    }
    std::cout << std::endl;
    
    std::cout << "Disabling torque..." << std::endl;
    actuator.setTorqueEnable(false);
    
    std::cout << std::endl;
    std::cout << "=== Demo Complete ===" << std::endl;
    
    device.stop();
    
    return 0;
}
