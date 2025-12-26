# Motor Driver Simulation System

A C++ library for simulating Feetech servo motor actuators **without real hardware**. This system provides realistic motor physics simulation, protocol-level communication, and comprehensive testing capabilities for development and testing purposes.

> **Note**: This is a **simulation-only** library. It does not connect to real hardware - all motor behavior is simulated through physics models and mock serial communication.

## Features

### **Core Capabilities**
- **Realistic Physics Simulation**: Acceleration profiles, inertia, temperature modeling, voltage fluctuations
- **Protocol Simulation**: Complete Feetech SCS/STS protocol with checksum validation over mock serial
- **Mock Serial Communication**: Simulated serial buffers for protocol testing
- **Thread-Safe Design**: Safe for concurrent access and multi-motor systems
- **Comprehensive Testing**: Unit tests and example programs demonstrating all features

### **Architecture**

```
┌─────────────────────────────────────────┐
│     FeetechActuator (High-Level API)    │
│  setPosition() | getPosition() | etc.   │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│    FeetechProtocol (Packet Handler)     │
│  Encoding | Decoding | Timeout          │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│   MockSerialBuffer (Simulated Serial)   │
│  Thread-safe circular buffer pairs      │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│   MotorSimulator (Physics Engine)       │
│  Position | Velocity | Temperature      │
└─────────────────────────────────────────┘
```

**How it Works:**
- `MotorSimulator` provides realistic physics (acceleration, load, temperature)
- `MockSerialBuffer` simulates bidirectional serial communication
- `FeetechProtocol` handles packet encoding/decoding over the mock serial
- `FeetechActuator` provides high-level motor control API
- All components work together to simulate real motor behavior without hardware

## Quick Start

### Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 6+, MSVC 2017+)
- CMake 3.16 or later
- (Optional) Google Test for unit tests

### Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run Examples

```bash
# Basic position control
./basic_control

# Position tracking with data logging
./position_tracking

# Multi-motor coordination
./multi_motor
```

### Run Tests

```bash
cd build
ctest --output-on-failure
```

## Usage Examples

### Example 1: Basic Motor Control (Simulation Only)

```cpp
#include "driver/FeetechActuator.hpp"
#include "driver/MotorSimulator.hpp"
#include "transport/MockSerialBuffer.hpp"

using namespace motor_sim;

int main() {
    // Create simulated serial connection (no real hardware needed)
    auto [host_port, device_port] = transport::createSerialPair();
    
    // Create motor physics simulator
    auto motor_sim = std::make_shared<driver::MotorSimulator>();
    
    // Create actuator interface (simulation-only, no real Feetech motor)
    driver::FeetechActuator actuator(1, host_port, motor_sim);
    
    // Initialize and control the simulated motor
    actuator.initialize();
    actuator.setTorqueEnable(true);
    actuator.setPositionDegrees(90.0f);
    
    return 0;
}
```

### Example 2: Realistic Protocol Simulation

For realistic protocol testing, run a device simulator in the background:

```cpp
// Device simulator responds to protocol commands
class DeviceSimulator {
    void run() {
        while (running_) {
            motor_->update(0.01f);  // Update physics
            processIncomingPackets();  // Handle commands
        }
    }
};

DeviceSimulator device(device_port, motor_sim);
device.start();  // Runs in background thread

// Now actuator.setPosition() will trigger realistic responses
```

## API Reference

### FeetechActuator Class

#### Position Control
```cpp
bool setPosition(uint16_t position, bool wait = false);
std::optional<uint16_t> getPosition();
bool setPositionDegrees(float degrees, bool wait = false);
std::optional<float> getPositionDegrees();
```

#### Velocity Control
```cpp
bool setSpeed(uint16_t speed);
std::optional<uint16_t> getSpeed();
bool setSpeedRPM(float rpm);
```

#### Configuration
```cpp
bool setAngleLimits(uint16_t cw, uint16_t ccw);
bool setTorqueEnable(bool enable);
bool setLED(bool on);
```

#### Status
```cpp
std::optional<uint8_t> getTemperature();
std::optional<uint8_t> getVoltage();
std::optional<uint16_t> getLoad();
std::optional<bool> isMoving();
```

### MotorSimulator Class

#### Physics Update
```cpp
void update(float dt);  // Call at ~100Hz for realistic simulation
```

#### Configuration
```cpp
void setGoalPosition(uint16_t position);
void setTorqueEnable(bool enable);
void setAngleLimits(uint16_t cw, uint16_t ccw);
```

#### Testing
```cpp
void injectError(ErrorFlag error);
void clearErrors();
void reset();
```

## Protocol Details

### Feetech Packet Structure
```
[0xFF][0xFF][ID][LEN][INSTR][PARAM...][CHECKSUM]
```

- **Header**: 0xFF 0xFF
- **ID**: Motor ID (1-253)
- **Length**: Instruction + Parameters + Checksum
- **Instruction**: Command type (PING, READ, WRITE, etc.)
- **Parameters**: Command-specific data
- **Checksum**: ~(ID + Length + Instruction + Parameters)

### Supported Instructions
- `PING` (0x01): Check motor presence
- `READ_DATA` (0x02): Read from memory
- `WRITE_DATA` (0x03): Write to memory
- `RESET` (0x06): Factory reset

### Register Map
Key registers:
- `GOAL_POSITION` (46-47): Target position (0-4095)
- `GOAL_SPEED` (48-49): Moving speed
- `PRESENT_POSITION` (56-57): Current position
- `PRESENT_TEMPERATURE` (63): Temperature in °C
- `TORQUE_ENABLE` (40): Enable/disable torque

## Testing

### Unit Tests
```bash
cd build
./unit_tests
```

Tests cover:
- Circular buffer operations
- Mock serial communication
- Protocol packet encoding/decoding
- Motor physics simulation
- Checksum validation

### Integration Testing

Create end-to-end tests combining all layers:

```cpp
TEST(IntegrationTest, FullStack) {
    auto [host, device] = createSerialPair();
    auto sim = std::make_shared<MotorSimulator>();
    DeviceSimulator dev(device, sim);
    dev.start();
    
    FeetechActuator actuator(1, host, sim);
    ASSERT_TRUE(actuator.ping());
    ASSERT_TRUE(actuator.setPosition(2048));
    
    dev.stop();
}
```

## Advanced Features

### Error Injection
```cpp
motor_sim->injectError(ErrorFlag::OVERHEATING);
// Motor will respond with error status
```

### Custom Physics
```cpp
MotorSimulator::PhysicsConfig config;
config.max_acceleration = 1000.0f;
config.thermal_coefficient = 0.02f;
config.position_noise = 1.0f;

MotorSimulator sim(initial_state, config);
```

### Callbacks
```cpp
actuator.setErrorCallback([](uint8_t error, const std::string& msg) {
    std::cerr << "Motor error: " << msg << std::endl;
});

actuator.setPositionCallback([](uint16_t position) {
    std::cout << "Position: " << position << std::endl;
});
```

## Configuration

### Motor Parameters (EEPROM)
- Model number, firmware version
- Angle limits (CW/CCW)
- Temperature/voltage limits
- Max torque

### Physics Tuning
- Acceleration limits
- Velocity limits
- Thermal properties
- Noise characteristics

## Troubleshooting

### Motor Not Responding
```cpp
if (!actuator.ping()) {
    // Check serial connection
    // Verify motor ID
    // Check if device simulator is running
}
```

### Position Not Updating
```cpp
// Ensure torque is enabled
actuator.setTorqueEnable(true);

// Check if in valid range
actuator.setAngleLimits(0, 4095);

// Update simulation
motor_sim->update(0.01f);
```

### Checksum Errors
```cpp
// Enable debug logging
Logger::getInstance().setLevel(LogLevel::DEBUG);

// Check for communication errors
buffer->injectError();  // Test error handling
```

## Performance

- **Update Rate**: 100-1000 Hz typical
- **Latency**: <1ms (simulated)
- **Memory**: ~100KB per motor instance
- **Thread Safety**: Full concurrent access support

## Roadmap

- [ ] ROS 2 integration
- [ ] Real-time plotting tools
- [ ] Trajectory planning utilities
- [ ] Multi-protocol support (Dynamixel, etc.)
- [ ] Hardware-in-the-loop testing
- [ ] Python bindings

## Contributing

Contributions welcome! Please ensure:
- Code follows existing style
- All tests pass
- New features include tests
- Documentation is updated

## License

MIT License - See LICENSE file for details

## References

- [Feetech SCS Series Manual](https://www.feetechrc.com)
- [Serial Communication Protocol Design](https://en.wikipedia.org/wiki/Serial_communication)
- Professional Robotics Software Architecture Patterns

## Contact

For questions or support, please open an issue on the project repository.

---

**Built with ❤️ for robotics development and testing**
