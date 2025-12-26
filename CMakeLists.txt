cmake_minimum_required(VERSION 3.16)
project(motor_driver_sim VERSION 1.0.0 LANGUAGES CXX)

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

find_package(Threads REQUIRED)

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

target_link_libraries(motor_driver_sim
    PUBLIC
        Threads::Threads
)

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

install(TARGETS motor_driver_sim
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)
