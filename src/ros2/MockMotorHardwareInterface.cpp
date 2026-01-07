#include "ros2/MockMotorHardwareInterface.hpp"
#include "utils/Logger.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <cmath>
#include <thread>

namespace motor_sim {
namespace ros2 {

// ============================================================================
// DeviceSimulator - Processes commands from host and updates motor simulator
// ============================================================================
class DeviceSimulator {
public:
    DeviceSimulator(std::shared_ptr<transport::MockSerialBuffer> device_port,
                   std::shared_ptr<driver::MotorSimulator> motor)
        : device_port_(device_port)
        , motor_(motor)
        , running_(false) {}

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
                        handlePacket(result->first);
                    }
                }
            }
            
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
                error = 0x40;
                break;
        }
        
        auto response = ProtocolPacket::createStatusResponse(
            packet.getId(), error, response_params);
        auto response_data = response.serialize();
        device_port_->write(response_data.data(), response_data.size());
    }

    std::shared_ptr<transport::MockSerialBuffer> device_port_;
    std::shared_ptr<driver::MotorSimulator> motor_;
    std::thread thread_;
    bool running_;
};

using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

// Custom interface for temperature
static constexpr char HW_IF_TEMPERATURE[] = "temperature";

MockMotorHardwareInterface::MockMotorHardwareInterface()
    : configured_(false)
    , activated_(false)
    , logger_(rclcpp::get_logger("MockMotorHardwareInterface")) {
}

MockMotorHardwareInterface::~MockMotorHardwareInterface() {
    // Ensure safe shutdown
    if (activated_) {
        on_deactivate(rclcpp_lifecycle::State());
    }
    if (configured_) {
        on_cleanup(rclcpp_lifecycle::State());
    }
}

// ============================================================================
// Lifecycle Implementation
// ============================================================================

CallbackReturn MockMotorHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    
    // Store hardware info from params
    info_ = params.hardware_info;

    RCLCPP_INFO(logger_, "Initializing Mock Motor Hardware Interface...");
    
    // Parse each joint from URDF
    joints_.resize(info_.joints.size());
    
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& joint_info = info_.joints[i];
        auto& joint = joints_[i];
        
        joint.name = joint_info.name;
        joint.motor_id = parseMotorId(joint_info);
        joint.position_offset = parsePositionOffset(joint_info);
        parseScaleFactors(joint_info, joint);
        
        // Initialize command/state to safe values
        joint.command_position = 0.0;
        joint.command_velocity = 0.0;
        joint.command_effort = 0.0;
        joint.state_position = 0.0;
        joint.state_velocity = 0.0;
        joint.state_effort = 0.0;
        joint.state_temperature = 25.0;
        
        RCLCPP_INFO(logger_, "  Joint '%s': motor_id=%d, offset=%.3f rad",
                    joint.name.c_str(), joint.motor_id, joint.position_offset);
    }
    
    RCLCPP_INFO(logger_, "Initialized %zu joints", joints_.size());
    return CallbackReturn::SUCCESS;
}

// Legacy on_init for backward compatibility (deprecated)
CallbackReturn MockMotorHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = info;
    return on_init(params);
}

CallbackReturn MockMotorHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Configuring Mock Motor Hardware Interface...");
    
    try {
        // Create motor simulators and actuators for each joint
        for (auto& joint : joints_) {
            // Create transport layer (each motor gets its own serial pair)
            auto [host, device] = transport::createSerialPair();
            
            // Create simulator with initial state
            driver::MotorState initial_state;
            initial_state.id = joint.motor_id;
            initial_state.present_position = 2048;  // Center position
            
            joint.simulator = std::make_shared<driver::MotorSimulator>(initial_state);
            
            // Create device simulator thread (processes commands)
            auto* device_sim = new DeviceSimulator(device, joint.simulator);
            joint.device_sim = device_sim;
            device_sim->start();
            
            // Create actuator connected to simulator
            joint.actuator = std::make_shared<driver::FeetechActuator>(
                joint.motor_id, host, joint.simulator);
            
            // Small delay to let device simulator start
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // Verify communication
            if (!joint.actuator->ping()) {
                RCLCPP_ERROR(logger_, "Failed to ping motor %d (%s)",
                            joint.motor_id, joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            
            RCLCPP_INFO(logger_, "  Configured motor %d (%s)",
                       joint.motor_id, joint.name.c_str());
        }
        
        configured_ = true;
        RCLCPP_INFO(logger_, "Configuration complete");
        return CallbackReturn::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Configuration failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

CallbackReturn MockMotorHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Activating Mock Motor Hardware Interface...");
    
    try {
        // Initialize and enable all motors
        for (auto& joint : joints_) {
            if (!joint.actuator->initialize()) {
                RCLCPP_ERROR(logger_, "Failed to initialize motor %d (%s)",
                            joint.motor_id, joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            
            if (!joint.actuator->setTorqueEnable(true)) {
                RCLCPP_ERROR(logger_, "Failed to enable torque for motor %d (%s)",
                            joint.motor_id, joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            
            // Read initial position
            if (auto pos = joint.actuator->getPosition()) {
                joint.state_position = rawToRadians(*pos, joint);
                joint.command_position = joint.state_position;  // Start at current position
            }
            
            RCLCPP_INFO(logger_, "  Activated motor %d (%s) at position %.3f rad",
                       joint.motor_id, joint.name.c_str(), joint.state_position);
        }
        
        activated_ = true;
        RCLCPP_INFO(logger_, "Activation complete - motors ready for control");
        return CallbackReturn::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Activation failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

CallbackReturn MockMotorHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Deactivating Mock Motor Hardware Interface...");
    
    try {
        // Disable torque on all motors (safe shutdown)
        for (auto& joint : joints_) {
            if (joint.actuator) {
                joint.actuator->setTorqueEnable(false);
                RCLCPP_INFO(logger_, "  Deactivated motor %d (%s)",
                           joint.motor_id, joint.name.c_str());
            }
        }
        
        activated_ = false;
        RCLCPP_INFO(logger_, "Deactivation complete - motors safe");
        return CallbackReturn::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Deactivation failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

CallbackReturn MockMotorHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Cleaning up Mock Motor Hardware Interface...");
    
    try {
        // Release all resources
        for (auto& joint : joints_) {
            if (joint.device_sim) {
                auto* device_sim = static_cast<DeviceSimulator*>(joint.device_sim);
                device_sim->stop();
                delete device_sim;
                joint.device_sim = nullptr;
            }
            joint.actuator.reset();
            joint.simulator.reset();
        }
        
        configured_ = false;
        RCLCPP_INFO(logger_, "Cleanup complete");
        return CallbackReturn::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Cleanup failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}

// ============================================================================
// Command/State Interface Export
// ============================================================================

std::vector<hardware_interface::StateInterface>
MockMotorHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (auto& joint : joints_) {
        // Standard interfaces
        state_interfaces.emplace_back(
            joint.name, HW_IF_POSITION, &joint.state_position);
        state_interfaces.emplace_back(
            joint.name, HW_IF_VELOCITY, &joint.state_velocity);
        state_interfaces.emplace_back(
            joint.name, HW_IF_EFFORT, &joint.state_effort);
        
        // Custom interface for temperature monitoring
        state_interfaces.emplace_back(
            joint.name, HW_IF_TEMPERATURE, &joint.state_temperature);
    }
    
    RCLCPP_INFO(logger_, "Exported %zu state interfaces", state_interfaces.size());
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MockMotorHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (auto& joint : joints_) {
        // Standard command interfaces
        command_interfaces.emplace_back(
            joint.name, HW_IF_POSITION, &joint.command_position);
        command_interfaces.emplace_back(
            joint.name, HW_IF_VELOCITY, &joint.command_velocity);
        command_interfaces.emplace_back(
            joint.name, HW_IF_EFFORT, &joint.command_effort);
    }
    
    RCLCPP_INFO(logger_, "Exported %zu command interfaces", command_interfaces.size());
    return command_interfaces;
}

// ============================================================================
// Real-time Communication Loop
// ============================================================================

hardware_interface::return_type MockMotorHardwareInterface::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
    
    if (!activated_) {
        return hardware_interface::return_type::OK;
    }
    
    // Update simulation for all motors
    updateSimulation(period);
    
    // Read state directly from simulator (fast path - no protocol overhead)
    for (auto& joint : joints_) {
        auto state = joint.simulator->getState();
        
        // Position
        joint.state_position = rawToRadians(state.present_position, joint);
        
        // Velocity  
        joint.state_velocity = rawSpeedToRadPerSec(state.present_speed, joint);
        
        // Effort (load)
        joint.state_effort = rawLoadToTorque(state.present_load, joint);
        
        // Temperature
        joint.state_temperature = static_cast<double>(state.present_temperature);
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockMotorHardwareInterface::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
    
    if (!activated_) {
        return hardware_interface::return_type::OK;
    }
    
    // Send commands directly to simulator (fast path - no protocol overhead)
    for (auto& joint : joints_) {
        // Position control (primary mode)
        uint16_t target_position = radiansToRaw(joint.command_position, joint);
        joint.simulator->setGoalPosition(target_position);
        
        // Velocity control (if commanded)
        if (std::abs(joint.command_velocity) > 1e-6) {
            uint16_t speed = static_cast<uint16_t>(
                std::abs(joint.command_velocity) / joint.velocity_scale);
            joint.simulator->setGoalSpeed(speed);
        }
    }
    
    return hardware_interface::return_type::OK;
}

// ============================================================================
// Helper Methods
// ============================================================================

uint8_t MockMotorHardwareInterface::parseMotorId(
    const hardware_interface::ComponentInfo& joint) {
    
    auto it = joint.parameters.find("motor_id");
    if (it == joint.parameters.end()) {
        RCLCPP_WARN(logger_, "No motor_id specified for joint %s, using default 1",
                   joint.name.c_str());
        return 1;
    }
    
    int id = std::stoi(it->second);
    if (id < 0 || id > 253) {
        RCLCPP_ERROR(logger_, "Invalid motor_id %d for joint %s (must be 0-253)",
                    id, joint.name.c_str());
        return 1;
    }
    
    return static_cast<uint8_t>(id);
}

double MockMotorHardwareInterface::parsePositionOffset(
    const hardware_interface::ComponentInfo& joint) {
    
    auto it = joint.parameters.find("position_offset");
    if (it == joint.parameters.end()) {
        return 0.0;
    }
    
    return std::stod(it->second);
}

void MockMotorHardwareInterface::parseScaleFactors(
    const hardware_interface::ComponentInfo& joint_info,
    JointInfo& joint) {
    
    // Position scale (units per radian)
    // Default: 4096 units / (2*PI radians) = ~651.9 units/rad
    auto it = joint_info.parameters.find("position_scale");
    joint.position_scale = (it != joint_info.parameters.end()) ?
        std::stod(it->second) : (4096.0 / (2.0 * M_PI));
    
    // Velocity scale (default: 0.111 RPM per unit)
    it = joint_info.parameters.find("velocity_scale");
    joint.velocity_scale = (it != joint_info.parameters.end()) ?
        std::stod(it->second) : 0.111;
    
    // Effort scale (load to torque conversion)
    it = joint_info.parameters.find("effort_scale");
    joint.effort_scale = (it != joint_info.parameters.end()) ?
        std::stod(it->second) : 0.001;  // 0.1% per unit
}

double MockMotorHardwareInterface::rawToRadians(
    uint16_t raw_position, const JointInfo& joint) const {
    
    // Convert raw position (0-4095) to radians with offset
    double radians = (static_cast<double>(raw_position) / joint.position_scale);
    return radians + joint.position_offset;
}

uint16_t MockMotorHardwareInterface::radiansToRaw(
    double radians, const JointInfo& joint) const {
    
    // Remove offset and convert to raw units
    double adjusted = radians - joint.position_offset;
    int32_t raw = static_cast<int32_t>(adjusted * joint.position_scale);
    
    // Clamp to valid range [0, 4095]
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;
    
    return static_cast<uint16_t>(raw);
}

double MockMotorHardwareInterface::rawSpeedToRadPerSec(
    uint16_t raw_speed, const JointInfo& joint) const {
    
    // Extract speed value (lower 10 bits) and direction (bit 10)
    uint16_t speed_val = raw_speed & 0x3FF;
    bool is_ccw = (raw_speed & 0x400) != 0;
    
    // Convert to RPM, then to rad/s
    double rpm = speed_val * joint.velocity_scale;
    double rad_per_sec = rpm * (2.0 * M_PI / 60.0);
    
    return is_ccw ? rad_per_sec : -rad_per_sec;
}

double MockMotorHardwareInterface::rawLoadToTorque(
    uint16_t raw_load, const JointInfo& joint) const {
    
    // Extract load value (lower 10 bits)
    uint16_t load_val = raw_load & 0x3FF;
    
    // Convert load percentage to estimated torque
    double load_percent = load_val * 0.1;  // 0.1% per unit
    double torque = (load_percent / 100.0) * joint.effort_scale;
    
    return torque;
}

void MockMotorHardwareInterface::updateSimulation(const rclcpp::Duration& period) {
    float dt = period.seconds();
    
    for (auto& joint : joints_) {
        // Update motor physics simulation
        joint.actuator->updateSimulation(dt);
    }
}

} // namespace ros2
} // namespace motor_sim

// ============================================================================
// Plugin Export (required for ros2_control to load this as a plugin)
// ============================================================================

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    motor_sim::ros2::MockMotorHardwareInterface,
    hardware_interface::SystemInterface
)
