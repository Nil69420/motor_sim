cmake_minimum_required(VERSION 3.16)
project(mock_interface VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_ROS2 "Build ROS2 hardware interface" ON)

find_package(Threads REQUIRED)

# ROS2 dependencies (optional, only if BUILD_ROS2 is ON)
if(BUILD_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(controller_interface REQUIRED)
    find_package(controller_manager REQUIRED)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

set(TRANSPORT_SOURCES
    src/transport/MockSerialBuffer.cpp
)

set(PROTOCOL_SOURCES
    src/protocol/ProtocolPacket.cpp
    src/protocol/FeetechProtocol.cpp
)

set(DRIVER_SOURCES
    src/driver/MotorState.cpp
    src/driver/MotorSimulator.cpp
    src/driver/FeetechActuator.cpp
)

set(UTILS_SOURCES
    src/utils/Checksum.cpp
    src/utils/Logger.cpp
)

add_library(motor_driver_sim STATIC
    ${TRANSPORT_SOURCES}
    ${PROTOCOL_SOURCES}
    ${DRIVER_SOURCES}
    ${UTILS_SOURCES}
)

# Enable position-independent code for ROS2 shared library compatibility
if(BUILD_ROS2)
    set_target_properties(motor_driver_sim PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_link_libraries(motor_driver_sim
    PUBLIC
        Threads::Threads
)

# ============================================================================
# ROS2 Hardware Interface Plugin
# ============================================================================

if(BUILD_ROS2)
    # Build ROS2 hardware interface as a shared library (plugin)
    add_library(mock_interface_ros2 SHARED
        src/ros2/MockMotorHardwareInterface.cpp
    )

    target_include_directories(mock_interface_ros2 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    # Link against ROS2 dependencies
    ament_target_dependencies(mock_interface_ros2
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
        controller_interface
    )

    # Link against our motor simulation library
    target_link_libraries(mock_interface_ros2
        motor_driver_sim
    )

    # Export plugin description
    pluginlib_export_plugin_description_file(
        hardware_interface
        mock_motor_hardware_interface_plugin.xml
    )

    # Install both libraries
    install(TARGETS motor_driver_sim mock_interface_ros2
        EXPORT export_mock_interface_ros2
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    # Install ROS2 configuration files
    install(DIRECTORY
        launch
        config
        urdf
        DESTINATION share/${PROJECT_NAME}/
    )

    # Export dependencies
    ament_export_targets(export_mock_interface_ros2 HAS_LIBRARY_TARGET)
    ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
        controller_interface
    )

    # Ament package
    ament_package()
endif()

if(BUILD_EXAMPLES)
    add_executable(simple_demo examples/simple_demo.cpp)
    target_link_libraries(simple_demo motor_driver_sim)
    
    add_executable(basic_control examples/basic_control.cpp)
    target_link_libraries(basic_control motor_driver_sim)

    add_executable(position_tracking examples/position_tracking.cpp)
    target_link_libraries(position_tracking motor_driver_sim)
    
    add_executable(multi_motor examples/multi_motor.cpp)
    target_link_libraries(multi_motor motor_driver_sim)
endif()

if(BUILD_TESTS)
    enable_testing()
    
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(unit_tests
        tests/unit/test_circular_buffer.cpp
        tests/unit/test_mock_serial.cpp
        tests/unit/test_protocol.cpp
        tests/unit/test_motor_simulator.cpp
    )
    
    target_link_libraries(unit_tests
        motor_driver_sim
        gtest_main
        gmock
    )
    
    include(GoogleTest)
    gtest_discover_tests(unit_tests)
endif()

# ============================================================================
# Install (non-ROS2 mode)
# ============================================================================

if(NOT BUILD_ROS2)
    install(TARGETS motor_driver_sim
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
    )

    install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
    )
endif()
