#include "driver/MotorSimulator.hpp"
#include "driver/MotorState.hpp"
#include "utils/Logger.hpp"
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace motor_sim;

struct TrackingData {
    double time;
    float position;
    float velocity;
    float load;
    uint8_t temperature;
};

int main() {
    utils::Logger::getInstance().setLevel(utils::LogLevel::WARN);
    
    std::cout << "=== Position Tracking & Data Logging Example ===" << std::endl;
    std::cout << std::endl;
    
    // Setup - Direct motor simulation
    driver::MotorState initial_state;
    initial_state.id = 1;
    initial_state.present_position = 2048;  // 180° center
    
    driver::MotorSimulator::PhysicsConfig config;
    config.max_acceleration = 2000.0f;
    config.max_velocity = 1000.0f;
    
    driver::MotorSimulator motor(initial_state, config);
    motor.setTorqueEnable(true);
    
    std::cout << "✓ Motor initialized" << std::endl;
    std::cout << std::endl;
    
    // Tracking experiment: sinusoidal motion
    std::cout << "Executing sinusoidal trajectory..." << std::endl;
    std::cout << "Frequency: 0.2 Hz | Amplitude: ±90° | Duration: 10 seconds" << std::endl;
    std::cout << std::endl;
    
    std::vector<TrackingData> log;
    auto start_time = std::chrono::steady_clock::now();
    const float duration = 10.0f;  // seconds
    const float frequency = 0.2f;  // Hz
    const float amplitude_deg = 90.0f; // degrees
    const float center_deg = 180.0f;   // center position
    const float amplitude = amplitude_deg * 4096.0f / 360.0f;  // Convert to raw units
    const float center = center_deg * 4096.0f / 360.0f;
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();
        
        if (elapsed > duration) break;
        
        // Update simulation
        motor.update(0.02f);  // 20ms timesteps
        
        // Generate sinusoidal reference
        float reference = center + amplitude * std::sin(2.0f * M_PI * frequency * elapsed);
        motor.setGoalPosition(static_cast<uint16_t>(reference));
        
        // Read actual state
        auto state = motor.getState();
        
        TrackingData data;
        data.time = elapsed;
        data.position = state.present_position * 360.0f / 4096.0f;
        data.velocity = (state.present_speed & 0x3FF) * 0.111f;  // Convert to RPM
        data.load = (state.present_load & 0x3FF);
        data.temperature = state.present_temperature;
        log.push_back(data);
            
        
        // Print progress every 0.5 seconds
        if (static_cast<int>(elapsed * 2) != static_cast<int>((elapsed - 0.02f) * 2)) {
            float ref_deg = reference * 360.0f / 4096.0f;
            float error = ref_deg - data.position;
            std::cout << "t=" << std::fixed << std::setprecision(1) << elapsed 
                     << "s | Ref: " << ref_deg << "° | Actual: " << data.position 
                     << "° | Error: " << error << "°" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    std::cout << std::endl;
    std::cout << "✓ Trajectory complete!" << std::endl;
    std::cout << "Collected " << log.size() << " data points" << std::endl;
    std::cout << std::endl;
    
    // Save to CSV
    std::string filename = "motor_tracking_data.csv";
    std::ofstream csv(filename);
    csv << "Time(s),Position(deg),Velocity(RPM),Load,Temperature(C)\n";
    
    for (const auto& data : log) {
        csv << std::fixed << std::setprecision(3)
            << data.time << ","
            << data.position << ","
            << data.velocity << ","
            << data.load << ","
            << static_cast<int>(data.temperature) << "\n";
    }
    csv.close();
    
    std::cout << "✓ Data saved to " << filename << std::endl;
    std::cout << std::endl;
    
    // Calculate statistics
    if (!log.empty()) {
        float max_error = 0.0f;
        float sum_error = 0.0f;
        
        for (const auto& data : log) {
            float reference = center_deg + amplitude_deg * std::sin(2.0f * M_PI * frequency * data.time);
            float error = std::abs(reference - data.position);
            max_error = std::max(max_error, error);
            sum_error += error;
        }
        
        float mean_error = sum_error / log.size();
        
        std::cout << "=== Tracking Performance ===" << std::endl;
        std::cout << "Mean tracking error: " << std::fixed << std::setprecision(2) 
                 << mean_error << "°" << std::endl;
        std::cout << "Max tracking error:  " << max_error << "°" << std::endl;
        std::cout << "Final temperature:   " << static_cast<int>(log.back().temperature) 
                 << "°C" << std::endl;
        std::cout << std::endl;
    } else {
        std::cout << "⚠ No data collected (communication issues)" << std::endl;
        std::cout << std::endl;
    }
    
    motor.setTorqueEnable(false);
    
    std::cout << "=== Demo Complete ===" << std::endl;
    std::cout << "Tip: Plot the CSV file to visualize the tracking performance!" << std::endl;
    
    return 0;
}
