#include <gtest/gtest.h>
#include "driver/MotorSimulator.hpp"
#include "protocol/ProtocolPacket.hpp"
#include <thread>
#include <chrono>

using namespace motor_sim::driver;
using namespace motor_sim;

class MotorSimulatorTest : public ::testing::Test {
protected:
    MotorState initial_state;
    MotorSimulator::PhysicsConfig config;
    
    void SetUp() override {
        initial_state.id = 1;
        initial_state.present_position = 2048;
        
        config.max_acceleration = 2000.0f;
        config.max_velocity = 1000.0f;
        config.position_noise = 0.0f;  
    }
};

TEST_F(MotorSimulatorTest, InitialState) {
    MotorSimulator sim(initial_state, config);
    auto state = sim.getState();
    
    EXPECT_EQ(state.id, 1);
    EXPECT_EQ(state.present_position, 2048);
    EXPECT_FALSE(state.moving);
    EXPECT_FALSE(state.torque_enable);
}

TEST_F(MotorSimulatorTest, TorqueEnable) {
    MotorSimulator sim(initial_state, config);
    
    sim.setTorqueEnable(true);
    auto state = sim.getState();
    EXPECT_TRUE(state.torque_enable);
    
    sim.setTorqueEnable(false);
    state = sim.getState();
    EXPECT_FALSE(state.torque_enable);
}

TEST_F(MotorSimulatorTest, PositionControl) {
    MotorSimulator sim(initial_state, config);
    sim.setTorqueEnable(true);
    
    sim.setGoalPosition(3000);
    
    for (int i = 0; i < 100; ++i) {
        sim.update(0.01f);  // 10ms steps
    }
    
    auto state = sim.getState();
    EXPECT_GT(state.present_position, initial_state.present_position);
    EXPECT_LE(state.present_position, 3000);
}

TEST_F(MotorSimulatorTest, MovingFlag) {
    MotorSimulator sim(initial_state, config);
    sim.setTorqueEnable(true);
    sim.setGoalPosition(3000);
    
    sim.update(0.1f);
    EXPECT_TRUE(sim.isMoving());
    
    for (int i = 0; i < 500; ++i) {
        sim.update(0.01f);
        if (!sim.isMoving()) break;
    }
    
    auto state = sim.getState();
    EXPECT_NEAR(state.present_position, 3000, 20);  
}

TEST_F(MotorSimulatorTest, AngleLimits) {
    MotorSimulator sim(initial_state, config);
    sim.setTorqueEnable(true);
    
    sim.setAngleLimits(1000, 3000);
    
    sim.setGoalPosition(500);  
    
    auto state = sim.getState();
    EXPECT_EQ(state.error & ErrorFlag::ANGLE_LIMIT, ErrorFlag::ANGLE_LIMIT);
}

TEST_F(MotorSimulatorTest, WheelMode) {
    MotorSimulator sim(initial_state, config);
    
    sim.setAngleLimits(0, 0);
    auto state = sim.getState();
    
    EXPECT_TRUE(state.isWheelMode());
    EXPECT_EQ(state.mode, OperatingMode::VELOCITY);
}

TEST_F(MotorSimulatorTest, RegisterReadWrite) {
    MotorSimulator sim(initial_state, config);
    
    sim.writeRegister(static_cast<uint8_t>(protocol::Register::TORQUE_ENABLE), 1);
    
    uint8_t value = sim.readRegister(static_cast<uint8_t>(protocol::Register::TORQUE_ENABLE));
    EXPECT_EQ(value, 1);
}

TEST_F(MotorSimulatorTest, MultiRegisterReadWrite) {
    MotorSimulator sim(initial_state, config);
    
    std::vector<uint8_t> data = {0x00, 0x04};  // Position 1024 (little-endian)
    sim.writeRegisters(static_cast<uint8_t>(protocol::Register::GOAL_POSITION_L), data);
    
    auto read_data = sim.readRegisters(
        static_cast<uint8_t>(protocol::Register::GOAL_POSITION_L), 2);
    
    EXPECT_EQ(read_data[0], 0x00);
    EXPECT_EQ(read_data[1], 0x04);
}

TEST_F(MotorSimulatorTest, TemperatureSimulation) {
    MotorSimulator sim(initial_state, config);
    sim.setTorqueEnable(true);
    
    sim.setGoalPosition(4000);
    
    for (int i = 0; i < 100; ++i) {
        sim.update(0.1f);
    }
    
    auto final_temp = sim.getState().present_temperature;
    EXPECT_GE(final_temp, 20);
    EXPECT_LE(final_temp, 85);
}

TEST_F(MotorSimulatorTest, Reset) {
    MotorSimulator sim(initial_state, config);
    
    sim.setTorqueEnable(true);
    sim.setGoalPosition(3000);
    sim.update(0.1f);
    
    sim.reset();
    
    auto state = sim.getState();
    EXPECT_FALSE(state.torque_enable);
    EXPECT_EQ(state.goal_position, 2048);  
}

TEST_F(MotorSimulatorTest, ErrorInjection) {
    MotorSimulator sim(initial_state, config);
    
    sim.injectError(ErrorFlag::OVERHEATING);
    auto state = sim.getState();
    
    EXPECT_EQ(state.error & ErrorFlag::OVERHEATING, ErrorFlag::OVERHEATING);
    
    sim.clearErrors();
    state = sim.getState();
    EXPECT_EQ(state.error, ErrorFlag::NONE);
}
