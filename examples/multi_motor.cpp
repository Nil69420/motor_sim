#include "driver/FeetechActuator.hpp"
#include "driver/MotorSimulator.hpp"
#include "transport/MockSerialBuffer.hpp"
#include "utils/Logger.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <thread>

using namespace motor_sim;

class DeviceSimulator {
public:
    DeviceSimulator(std::shared_ptr<transport::MockSerialBuffer> device_port,
                   std::shared_ptr<driver::MotorSimulator> motor)
        : device_port_(device_port), motor_(motor), running_(false) {}

    void start() {
        running_ = true;
        thread_ = std::thread(&DeviceSimulator::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    ~DeviceSimulator() { stop(); }

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
                device_port_->read(buffer.data(), buffer.size());
                auto result = ProtocolPacket::findPacket(buffer.data(), buffer.size());
                if (result) handlePacket(result->first);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void handlePacket(const protocol::ProtocolPacket& packet) {
        using namespace protocol;
        uint8_t error = 0;
        std::vector<uint8_t> response_params;
        
        switch (packet.getInstruction()) {
            case Instruction::PING: break;
            case Instruction::READ_DATA:
                if (packet.getParams().size() >= 2) {
                    response_params = motor_->readRegisters(packet.getParams()[0], 
                                                           packet.getParams()[1]);
                }
                break;
            case Instruction::WRITE_DATA:
                if (packet.getParams().size() >= 2) {
                    std::vector<uint8_t> data(packet.getParams().begin() + 1, 
                                             packet.getParams().end());
                    motor_->writeRegisters(packet.getParams()[0], data);
                }
                break;
            default: error = 0x40; break;
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

struct MultiMotorSetup {
    std::shared_ptr<transport::MockSerialBuffer> host_port;
    std::shared_ptr<transport::MockSerialBuffer> device_port;
    std::shared_ptr<driver::MotorSimulator> simulator;
    std::shared_ptr<DeviceSimulator> device;
    std::shared_ptr<driver::FeetechActuator> actuator;
};

int main() {
    utils::Logger::getInstance().setLevel(utils::LogLevel::WARN);
    
    std::cout << "=== Multi-Motor Coordination Example ===" << std::endl;
    std::cout << "Simulating a 3-DOF robotic arm" << std::endl;
    std::cout << std::endl;
    
    // Setup 3 motors (base, shoulder, elbow)
    const int NUM_MOTORS = 3;
    std::vector<MultiMotorSetup> motors;
    
    for (int i = 0; i < NUM_MOTORS; ++i) {
        MultiMotorSetup setup;
        
        auto ports = transport::createSerialPair();
        setup.host_port = ports.first;
        setup.device_port = ports.second;
        
        driver::MotorState state;
        state.id = i + 1;
        setup.simulator = std::make_shared<driver::MotorSimulator>(state);
        
        setup.device = std::make_shared<DeviceSimulator>(setup.device_port, setup.simulator);
        setup.device->start();
        
        setup.actuator = std::make_shared<driver::FeetechActuator>(
            i + 1, setup.host_port, setup.simulator);
        
        motors.push_back(setup);
    }
    
    std::cout << "✓ Created " << NUM_MOTORS << " virtual motors" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Initializing motors..." << std::endl;
    for (size_t i = 0; i < motors.size(); ++i) {
        if (!motors[i].actuator->ping()) {
            std::cout << "✗ Motor " << (i+1) << " not responding" << std::endl;
            return 1;
        }
        motors[i].actuator->initialize();
        motors[i].actuator->setTorqueEnable(true);
        std::cout << "  Motor " << (i+1) << ": OK" << std::endl;
    }
    std::cout << std::endl;
    
    struct Pose {
        std::string name;
        std::vector<float> angles; 
    };
    
    std::vector<Pose> sequence = {
        {"Home Position",     {180.0f, 180.0f, 180.0f}},
        {"Reach Forward",     {180.0f, 220.0f, 160.0f}},
        {"Lift Up",           {180.0f, 180.0f, 240.0f}},
        {"Rotate Base",       {270.0f, 180.0f, 240.0f}},
        {"Lower Down",        {270.0f, 220.0f, 160.0f}},
        {"Return Home",       {180.0f, 180.0f, 180.0f}},
    };
    
    std::cout << "=== Executing Coordinated Motion Sequence ===" << std::endl;
    std::cout << std::endl;
    
    for (const auto& pose : sequence) {
        std::cout << "→ Moving to: " << pose.name << std::endl;
        
        for (size_t i = 0; i < motors.size(); ++i) {
            motors[i].actuator->setPositionDegrees(pose.angles[i], false);
        }
        
        bool all_stopped = false;
        int iterations = 0;
        const int max_iterations = 100;
        
        while (!all_stopped && iterations < max_iterations) {
            all_stopped = true;
            
            std::cout << "  ";
            for (size_t i = 0; i < motors.size(); ++i) {
                auto pos = motors[i].actuator->getPositionDegrees();
                auto moving = motors[i].actuator->isMoving();
                
                if (pos) {
                    std::cout << "M" << (i+1) << ":" << std::fixed 
                             << std::setprecision(1) << *pos << "° ";
                }
                
                if (moving && *moving) {
                    all_stopped = false;
                }
            }
            std::cout << std::endl;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            iterations++;
        }
        
        std::cout << "  ✓ Pose reached!" << std::endl;
        std::cout << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    std::cout << "=== Final Motor Status ===" << std::endl;
    std::cout << std::endl;
    
    for (size_t i = 0; i < motors.size(); ++i) {
        auto state = motors[i].actuator->getState();
        if (state) {
            std::cout << "Motor " << (i+1) << ":" << std::endl;
            std::cout << "  Position:    " << state->positionToDegrees(state->present_position) 
                     << "°" << std::endl;
            std::cout << "  Temperature: " << static_cast<int>(state->present_temperature) 
                     << "°C" << std::endl;
            std::cout << "  Load:        " << state->getLoadPercentage() << "%" << std::endl;
            std::cout << std::endl;
        }
    }
    
    // Cleanup
    std::cout << "Shutting down..." << std::endl;
    for (auto& motor : motors) {
        motor.actuator->setTorqueEnable(false);
        motor.device->stop();
    }
    
    std::cout << std::endl;
    std::cout << "=== Demo Complete ===" << std::endl;
    
    return 0;
}
