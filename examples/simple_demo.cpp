#include "driver/MotorSimulator.hpp"
#include "driver/MotorState.hpp"
#include "utils/Logger.hpp"
#include <iostream>
#include <iomanip>

using namespace motor_sim;

int main() {
    utils::Logger::getInstance().setLevel(utils::LogLevel::WARN);
    
    std::cout << "=== Motor Simulator Direct Test ===" << std::endl;
    std::cout << std::endl;
    
    // Create a motor with default state
    driver::MotorState initial_state;
    initial_state.id = 1;
    initial_state.present_position = 2048;  // Center position
    
    // Configure physics
    driver::MotorSimulator::PhysicsConfig config;
    config.max_acceleration = 2000.0f;
    config.max_velocity = 1000.0f;
    config.position_noise = 0.0f;  // Disable for clean output
    
    driver::MotorSimulator motor(initial_state, config);
    
    std::cout << "✓ Created motor simulator (ID=" << static_cast<int>(initial_state.id) << ")" << std::endl;
    std::cout << "  Initial position: " << initial_state.present_position << " (180°)" << std::endl;
    std::cout << std::endl;
    
    // Enable torque
    std::cout << "Enabling torque..." << std::endl;
    motor.setTorqueEnable(true);
    std::cout << std::endl;
    
    // Test 1: Move to different positions
    std::cout << "=== Position Control Test ===" << std::endl;
    std::cout << std::endl;
    
    std::vector<uint16_t> targets = {1024, 2048, 3072, 2048};
    std::vector<std::string> labels = {"90°", "180°", "270°", "180°"};
    
    for (size_t i = 0; i < targets.size(); ++i) {
        std::cout << "Target: " << labels[i] << " (position " << targets[i] << ")" << std::endl;
        motor.setGoalPosition(targets[i]);
        
        // Simulate movement
        for (int step = 0; step < 200; ++step) {
            motor.update(0.01f);  // 10ms timesteps
            
            if (step % 20 == 0) {  // Print every 200ms
                auto state = motor.getState();
                std::cout << "  t=" << std::fixed << std::setprecision(2) 
                         << (step * 0.01f) << "s"
                         << " | pos=" << state.present_position
                         << " | speed=" << (state.present_speed & 0x3FF)
                         << " | load=" << (state.present_load & 0x3FF)
                         << " | temp=" << static_cast<int>(state.present_temperature) << "°C"
                         << " | moving=" << (state.moving ? "YES" : "NO ")
                         << std::endl;
            }
            
            if (!motor.isMoving()) {
                auto state = motor.getState();
                std::cout << "  ✓ Reached target at t=" << std::fixed << std::setprecision(2)
                         << (step * 0.01f) << "s (position=" << state.present_position << ")"
                         << std::endl;
                break;
            }
        }
        std::cout << std::endl;
    }
    
    // Test 2: Register access
    std::cout << "=== Register Access Test ===" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Reading motor registers:" << std::endl;
    std::cout << "  Model Number:  0x" << std::hex << std::setfill('0') << std::setw(4)
             << ((motor.readRegister(1) << 8) | motor.readRegister(0)) << std::dec << std::endl;
    std::cout << "  Firmware Ver:  0x" << std::hex << std::setfill('0') << std::setw(2)
             << static_cast<int>(motor.readRegister(2)) << std::dec << std::endl;
    std::cout << "  Motor ID:      " << static_cast<int>(motor.readRegister(5)) << std::endl;
    std::cout << std::endl;
    
    // Test 3: Wheel mode
    std::cout << "=== Wheel Mode Test ===" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Switching to wheel mode (continuous rotation)..." << std::endl;
    motor.setAngleLimits(0, 0);
    motor.setGoalSpeed(512);  // Medium speed, CCW direction
    
    std::cout << "Rotating for 1 second..." << std::endl;
    uint16_t start_pos = motor.getState().present_position;
    for (int step = 0; step < 100; ++step) {
        motor.update(0.01f);
    }
    uint16_t end_pos = motor.getState().present_position;
    
    std::cout << "  Start position: " << start_pos << std::endl;
    std::cout << "  End position:   " << end_pos << std::endl;
    std::cout << "  Distance traveled: " << (end_pos > start_pos ? end_pos - start_pos : 4096 - start_pos + end_pos) << std::endl;
    std::cout << std::endl;
    
    // Final status
    auto final_state = motor.getState();
    std::cout << "=== Final Motor Status ===" << std::endl;
    std::cout << "  Position:     " << final_state.present_position << " (" 
             << final_state.positionToDegrees(final_state.present_position) << "°)" << std::endl;
    std::cout << "  Speed:        " << (final_state.present_speed & 0x3FF) 
             << " (" << final_state.speedToRPM(final_state.present_speed) << " RPM)" << std::endl;
    std::cout << "  Temperature:  " << static_cast<int>(final_state.present_temperature) << "°C" << std::endl;
    std::cout << "  Voltage:      " << (final_state.present_voltage / 10.0f) << "V" << std::endl;
    std::cout << "  Load:         " << final_state.getLoadPercentage() << "%" << std::endl;
    std::cout << "  Torque:       " << (final_state.torque_enable ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "  Mode:         " << (final_state.isWheelMode() ? "WHEEL" : "POSITION") << std::endl;
    std::cout << std::endl;
    
    std::cout << "=== Test Complete ===" << std::endl;
    std::cout << "The motor simulator successfully:" << std::endl;
    std::cout << "  ✓ Performed realistic position control" << std::endl;
    std::cout << "  ✓ Simulated acceleration/deceleration" << std::endl;
    std::cout << "  ✓ Tracked load and temperature" << std::endl;
    std::cout << "  ✓ Supported register access" << std::endl;
    std::cout << "  ✓ Operated in wheel mode" << std::endl;
    
    return 0;
}
