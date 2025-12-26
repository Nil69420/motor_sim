#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <optional>

namespace motor_sim {
namespace protocol {

/**
 * @brief Feetech Protocol Instructions (Commands)
 */
enum class Instruction : uint8_t {
    PING = 0x01,
    READ_DATA = 0x02,
    WRITE_DATA = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    SYNC_WRITE = 0x83,
    RESET = 0x06,
};

/**
 * @brief Feetech Protocol Memory Map (Register Addresses)
 */
enum class Register : uint8_t {
    // EEPROM Area (persisted)
    MODEL_NUMBER_L = 0,
    MODEL_NUMBER_H = 1,
    FIRMWARE_VERSION = 2,
    ID = 5,
    BAUD_RATE = 6,
    RETURN_DELAY_TIME = 7,
    CW_ANGLE_LIMIT_L = 8,
    CW_ANGLE_LIMIT_H = 9,
    CCW_ANGLE_LIMIT_L = 10,
    CCW_ANGLE_LIMIT_H = 11,
    TEMPERATURE_LIMIT = 13,
    MIN_VOLTAGE = 14,
    MAX_VOLTAGE = 15,
    MAX_TORQUE_L = 16,
    MAX_TORQUE_H = 17,
    
    // RAM Area (volatile)
    TORQUE_ENABLE = 40,
    LED = 41,
    CW_COMPLIANCE_MARGIN = 42,
    CCW_COMPLIANCE_MARGIN = 43,
    CW_COMPLIANCE_SLOPE = 44,
    CCW_COMPLIANCE_SLOPE = 45,
    GOAL_POSITION_L = 46,
    GOAL_POSITION_H = 47,
    GOAL_SPEED_L = 48,
    GOAL_SPEED_H = 49,
    TORQUE_LIMIT_L = 50,
    TORQUE_LIMIT_H = 51,
    PRESENT_POSITION_L = 56,
    PRESENT_POSITION_H = 57,
    PRESENT_SPEED_L = 58,
    PRESENT_SPEED_H = 59,
    PRESENT_LOAD_L = 60,
    PRESENT_LOAD_H = 61,
    PRESENT_VOLTAGE = 62,
    PRESENT_TEMPERATURE = 63,
    REGISTERED = 64,
    MOVING = 66,
    LOCK = 67,
    PUNCH_L = 68,
    PUNCH_H = 69,
};

/**
 * @brief Protocol packet structure
 * 
 * Feetech packet format:
 * [0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAM1]...[PARAMN][CHECKSUM]
 */
class ProtocolPacket {
public:
    static constexpr uint8_t HEADER_0 = 0xFF;
    static constexpr uint8_t HEADER_1 = 0xFF;
    static constexpr uint8_t BROADCAST_ID = 0xFE;
    static constexpr size_t MAX_PARAMS = 256;
    static constexpr size_t MIN_PACKET_SIZE = 6; // Headers + ID + Len + Instr + Checksum

    ProtocolPacket();
    
    /**
     * @brief Create instruction packet
     */
    static ProtocolPacket createInstruction(uint8_t id, Instruction instruction, 
                                           const std::vector<uint8_t>& params = {});

    /**
     * @brief Create read request packet
     */
    static ProtocolPacket createReadRequest(uint8_t id, Register start_addr, uint8_t length);

    /**
     * @brief Create write request packet
     */
    static ProtocolPacket createWriteRequest(uint8_t id, Register start_addr, 
                                            const std::vector<uint8_t>& data);

    /**
     * @brief Create status response packet
     */
    static ProtocolPacket createStatusResponse(uint8_t id, uint8_t error, 
                                              const std::vector<uint8_t>& params = {});

    /**
     * @brief Serialize packet to bytes
     */
    std::vector<uint8_t> serialize() const;

    /**
     * @brief Deserialize bytes into packet
     * @return nullopt if invalid packet
     */
    static std::optional<ProtocolPacket> deserialize(const uint8_t* data, size_t length);

    /**
     * @brief Find packet in a stream of bytes
     * @return Pair of (packet, bytes_consumed) or nullopt
     */
    static std::optional<std::pair<ProtocolPacket, size_t>> 
        findPacket(const uint8_t* data, size_t length);

    // Getters
    uint8_t getId() const { return id_; }
    uint8_t getLength() const { return length_; }
    Instruction getInstruction() const { return instruction_; }
    uint8_t getError() const { return error_; }
    const std::vector<uint8_t>& getParams() const { return params_; }
    bool isValid() const { return is_valid_; }
    bool isResponse() const { return is_response_; }

    // Setters
    void setId(uint8_t id) { id_ = id; }
    void setInstruction(Instruction instr) { instruction_ = instr; }
    void setError(uint8_t error) { error_ = error; }
    void setParams(const std::vector<uint8_t>& params) { params_ = params; }

private:
    uint8_t id_;
    uint8_t length_;
    Instruction instruction_;
    uint8_t error_;  // For status packets
    std::vector<uint8_t> params_;
    bool is_valid_;
    bool is_response_;  // True for status packets, false for instruction packets

    void updateLength();
    uint8_t calculateChecksum() const;
    bool validateChecksum(uint8_t checksum) const;
};

} // namespace protocol
} // namespace motor_sim
