#pragma once

#include "transport/ITransport.hpp"
#include "utils/CircularBuffer.hpp"
#include <memory>
#include <chrono>

namespace motor_sim {
namespace transport {


class MockSerialBuffer : public ITransport, 
                        public std::enable_shared_from_this<MockSerialBuffer> {
public:
    struct Config {
        size_t tx_buffer_size = 1024;
        size_t rx_buffer_size = 1024;
        uint32_t baud_rate = 115200;
        bool simulate_timing = true;
        double error_rate = 0.0;  // Probability of byte corruption (0.0 - 1.0)
    };

    explicit MockSerialBuffer(const Config& config);
    MockSerialBuffer();
    ~MockSerialBuffer() override = default;

    // ITransport interface
    size_t write(const uint8_t* data, size_t size) override;
    size_t read(uint8_t* data, size_t size) override;
    size_t available() const override;
    void flush() override;
    void clear() override;
    bool isOpen() const override { return is_open_; }

    void connectTo(std::shared_ptr<MockSerialBuffer> other);

    /**
     * @brief Get the connected peer buffer
     */
    std::shared_ptr<MockSerialBuffer> getPeer() const { return peer_.lock(); }

    /**
     * @brief Simulate transmission delay based on baud rate
     */
    void simulateDelay(size_t bytes);

    /**
     * @brief Get TX buffer for testing
     */
    const utils::CircularBuffer<uint8_t>& getTxBuffer() const { return tx_buffer_; }

    /**
     * @brief Get RX buffer for testing
     */
    const utils::CircularBuffer<uint8_t>& getRxBuffer() const { return rx_buffer_; }

    /**
     * @brief Inject a byte corruption error
     */
    void injectError() { inject_next_error_ = true; }

private:
    utils::CircularBuffer<uint8_t> tx_buffer_;
    utils::CircularBuffer<uint8_t> rx_buffer_;
    std::weak_ptr<MockSerialBuffer> peer_;
    
    Config config_;
    bool is_open_;
    bool inject_next_error_;
    
    std::chrono::steady_clock::time_point last_write_time_;
    
    uint8_t corruptByte(uint8_t byte);
    bool shouldInjectError();
};


std::pair<std::shared_ptr<MockSerialBuffer>, std::shared_ptr<MockSerialBuffer>>
createSerialPair(const MockSerialBuffer::Config& config = MockSerialBuffer::Config());

} // namespace transport
} // namespace motor_sim
