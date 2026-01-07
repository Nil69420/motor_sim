#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "driver/FeetechActuator.hpp"
#include "driver/MotorSimulator.hpp"
#include "transport/MockSerialBuffer.hpp"

#include <memory>
#include <string>
#include <vector>

namespace motor_sim {
namespace ros2 {

/**
 * @brief ROS2 Hardware Interface for Mock Motor Simulation
 * 
 * Implements the ros2_control SystemInterface to bridge our motor simulation
 * with ROS2 controllers (JointTrajectoryController, PositionController, etc.)
 * 
 * Lifecycle:
 *   on_init()       -> Create motor instances from URDF
 *   on_configure()  -> Initialize transport and connect to simulators
 *   on_activate()   -> Enable torque, start communication
 *   on_deactivate() -> Disable torque, stop safely
 *   on_cleanup()    -> Release all resources
 */
class MockMotorHardwareInterface : public hardware_interface::SystemInterface {
public:
    MockMotorHardwareInterface();
    virtual ~MockMotorHardwareInterface();

    // ========================================================================
    // Lifecycle Management (ros2_control required methods)
    // ========================================================================

    /**
     * @brief Initialize hardware from URDF description
     * 
     * Called once at node startup. Parses robot description to:
     * - Extract joint names and motor IDs
     * - Create MotorSimulator instances
     * - Set up communication transport
     * 
     * @param info Hardware information from URDF
     * @return SUCCESS if initialization successful
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override 
        __attribute__((deprecated("Use on_init with HardwareComponentInterfaceParams")));
    
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    /**
     * @brief Configure hardware and establish connections
     * 
     * Called when transitioning to "inactive" state. Actions:
     * - Initialize MockSerialBuffer transport
     * - Create FeetechActuator instances
     * - Connect actuators to simulators
     * - Verify communication
     * 
     * @param previous_state The previous lifecycle state
     * @return SUCCESS if configuration successful
     */
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Activate motors and start control loop
     * 
     * Called when transitioning to "active" state. Actions:
     * - Enable torque on all motors
     * - Initialize command/state interfaces
     * - Start real-time communication thread (if needed)
     * 
     * @param previous_state The previous lifecycle state
     * @return SUCCESS if activation successful
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Deactivate motors safely
     * 
     * Called when transitioning from "active" state. Actions:
     * - Disable torque on all motors
     * - Stop any background threads
     * - Ensure motors are in safe state
     * 
     * @param previous_state The previous lifecycle state
     * @return SUCCESS if deactivation successful
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Clean up and release all resources
     * 
     * Called when transitioning to "unconfigured" state. Actions:
     * - Disconnect from motors
     * - Release transport resources
     * - Clear all state
     * 
     * @param previous_state The previous lifecycle state
     * @return SUCCESS if cleanup successful
     */
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    // ========================================================================
    // Hardware Interface Export (Command/State Interfaces)
    // ========================================================================

    /**
     * @brief Export command interfaces to ROS2 controllers
     * 
     * Provides interfaces for:
     * - position (radians)
     * - velocity (rad/s)
     * - effort (Nm) - optional
     * 
     * @return Vector of command interface descriptions
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * @brief Export state interfaces for ROS2 controllers
     * 
     * Provides interfaces for:
     * - position (radians)
     * - velocity (rad/s)
     * - effort (Nm)
     * - temperature (°C) - custom interface
     * 
     * @return Vector of state interface descriptions
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ========================================================================
    // Real-time Communication Loop
    // ========================================================================

    /**
     * @brief Read current state from motors
     * 
     * Called at every control cycle (typically 100-1000 Hz).
     * Updates state interfaces with current motor feedback.
     * 
     * @param time Current time
     * @param period Time since last read
     * @return SUCCESS if read successful
     */
    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    /**
     * @brief Write commands to motors
     * 
     * Called at every control cycle after controllers compute commands.
     * Sends position/velocity/effort commands to motors.
     * 
     * @param time Current time
     * @param period Time since last write
     * @return SUCCESS if write successful
     */
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    // ========================================================================
    // Internal State Management
    // ========================================================================

    /**
     * @brief Motor joint information
     */
    struct JointInfo {
        std::string name;                              ///< Joint name from URDF
        uint8_t motor_id;                              ///< Feetech motor ID (1-253)
        std::shared_ptr<driver::FeetechActuator> actuator;  ///< Motor driver API
        std::shared_ptr<driver::MotorSimulator> simulator;  ///< Physics simulation
        void* device_sim;                              ///< Background simulation thread (opaque)
        
        // Command interface storage (written by controllers)
        double command_position;                       ///< Target position (rad)
        double command_velocity;                       ///< Target velocity (rad/s)
        double command_effort;                         ///< Target effort (Nm)
        
        // State interface storage (read from motors)
        double state_position;                         ///< Current position (rad)
        double state_velocity;                         ///< Current velocity (rad/s)
        double state_effort;                           ///< Current effort (Nm)
        double state_temperature;                      ///< Motor temperature (°C)
        
        // Configuration
        double position_offset;                        ///< Zero position offset
        double position_scale;                         ///< Units/radian conversion
        double velocity_scale;                         ///< Velocity conversion
        double effort_scale;                           ///< Effort conversion
    };

    std::vector<JointInfo> joints_;                    ///< All controlled joints
    
    // Lifecycle state tracking
    bool configured_;
    bool activated_;
    
    // Logger
    rclcpp::Logger logger_;

    // ========================================================================
    // Helper Methods
    // ========================================================================

    /**
     * @brief Parse motor ID from URDF parameter
     */
    uint8_t parseMotorId(const hardware_interface::ComponentInfo& joint);

    /**
     * @brief Parse position offset from URDF parameter
     */
    double parsePositionOffset(const hardware_interface::ComponentInfo& joint);

    /**
     * @brief Parse scale factors from URDF parameters
     */
    void parseScaleFactors(const hardware_interface::ComponentInfo& joint, JointInfo& info);

    /**
     * @brief Convert raw motor position to radians
     */
    double rawToRadians(uint16_t raw_position, const JointInfo& joint) const;

    /**
     * @brief Convert radians to raw motor position
     */
    uint16_t radiansToRaw(double radians, const JointInfo& joint) const;

    /**
     * @brief Convert raw motor speed to rad/s
     */
    double rawSpeedToRadPerSec(uint16_t raw_speed, const JointInfo& joint) const;

    /**
     * @brief Convert raw motor load to torque (Nm)
     */
    double rawLoadToTorque(uint16_t raw_load, const JointInfo& joint) const;

    /**
     * @brief Update simulation for all motors
     */
    void updateSimulation(const rclcpp::Duration& period);
};

} // namespace ros2
} // namespace motor_sim
