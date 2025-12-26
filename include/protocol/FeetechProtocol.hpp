#pragma once

#include "protocol/ProtocolPacket.hpp"
#include "transport/ITransport.hpp"
#include <memory>
#include <chrono>

namespace motor_sim {
namespace protocol {

class FeetechProtocol {
public:
    struct Config {
        std::chrono::milliseconds timeout{100};
        uint8_t max_retries{3};
        std::chrono::microseconds response_delay{0};  // Simulated device response delay
    };

    explicit FeetechProtocol(std::shared_ptr<transport::ITransport> transport,
                            const Config& config);
    explicit FeetechProtocol(std::shared_ptr<transport::ITransport> transport);

    /**
     * @brief Send packet and wait for response
     * @return Response packet or nullopt on timeout/error
     */
    std::optional<ProtocolPacket> sendAndReceive(const ProtocolPacket& packet);

    /**
     * @brief Send packet without waiting for response
     */
    bool send(const ProtocolPacket& packet);

    /**
     * @brief Receive packet with timeout
     */
    std::optional<ProtocolPacket> receive();

    /**
     * @brief Ping a motor
     * @return true if motor responds
     */
    bool ping(uint8_t id);

    /**
     * @brief Read data from motor register
     */
    std::optional<std::vector<uint8_t>> readData(uint8_t id, Register start_addr, uint8_t length);

    /**
     * @brief Write data to motor register
     */
    bool writeData(uint8_t id, Register start_addr, const std::vector<uint8_t>& data);

    /**
     * @brief Read 16-bit value (little-endian)
     */
    std::optional<uint16_t> readWord(uint8_t id, Register addr);

    /**
     * @brief Write 16-bit value (little-endian)
     */
    bool writeWord(uint8_t id, Register addr, uint16_t value);

    /**
     * @brief Read 8-bit value
     */
    std::optional<uint8_t> readByte(uint8_t id, Register addr);

    /**
     * @brief Write 8-bit value
     */
    bool writeByte(uint8_t id, Register addr, uint8_t value);

    /**
     * @brief Get last error code
     */
    uint8_t getLastError() const { return last_error_; }

    /**
     * @brief Clear receive buffer
     */
    void clearBuffer();

private:
    std::shared_ptr<transport::ITransport> transport_;
    Config config_;
    uint8_t last_error_;

    bool waitForResponse(std::chrono::milliseconds timeout);
    std::optional<ProtocolPacket> parseResponse();
};

} // namespace protocol
} // namespace motor_sim
