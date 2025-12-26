#include "transport/MockSerialBuffer.hpp"
#include "utils/Logger.hpp"
#include <thread>
#include <cstring>

namespace motor_sim {
namespace transport {

MockSerialBuffer::MockSerialBuffer(const Config& config)
    : tx_buffer_(config.tx_buffer_size)
    , rx_buffer_(config.rx_buffer_size)
    , config_(config)
    , is_open_(true)
    , inject_next_error_(false)
    , last_write_time_(std::chrono::steady_clock::now()) {
}

MockSerialBuffer::MockSerialBuffer()
    : MockSerialBuffer(Config{}) {
}

size_t MockSerialBuffer::write(const uint8_t* data, size_t size) {
    if (!is_open_) return 0;

    if (config_.simulate_timing) {
        simulateDelay(size);
    }

    size_t written = tx_buffer_.write(data, size);

    if (auto peer = peer_.lock()) {
        std::vector<uint8_t> temp(written);
        tx_buffer_.peek(temp.data(), written);
        
        for (size_t i = 0; i < written; ++i) {
            if (shouldInjectError()) {
                temp[i] = corruptByte(temp[i]);
                LOG_WARN("MockSerial: Injected byte corruption at offset ", i);
            }
        }
        
        peer->rx_buffer_.write(temp.data(), written);
    }

    LOG_TRACE("MockSerial: Wrote ", written, " bytes");
    return written;
}

size_t MockSerialBuffer::read(uint8_t* data, size_t size) {
    if (!is_open_) return 0;

    size_t bytes_read = rx_buffer_.read(data, size);
    LOG_TRACE("MockSerial: Read ", bytes_read, " bytes");
    return bytes_read;
}

size_t MockSerialBuffer::available() const {
    if (!is_open_) return 0;
    return rx_buffer_.available();
}

void MockSerialBuffer::flush() {
    // No-op for mock buffer
}

void MockSerialBuffer::clear() {
    tx_buffer_.clear();
    rx_buffer_.clear();
}

void MockSerialBuffer::connectTo(std::shared_ptr<MockSerialBuffer> other) {
    peer_ = other;
    if (other) {
        other->peer_ = shared_from_this();
        LOG_INFO("MockSerial: Connected to peer");
    }
}

void MockSerialBuffer::simulateDelay(size_t bytes) {
    if (!config_.simulate_timing || config_.baud_rate == 0) return;

    // Calculate transmission time: (bytes * 10 bits/byte) / baud_rate
    // 10 bits = 1 start + 8 data + 1 stop bit
    auto transmission_us = (bytes * 10 * 1000000ULL) / config_.baud_rate;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        now - last_write_time_);
    
    if (transmission_us > static_cast<uint64_t>(elapsed.count())) {
        auto sleep_time = transmission_us - elapsed.count();
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    }
    
    last_write_time_ = std::chrono::steady_clock::now();
}

uint8_t MockSerialBuffer::corruptByte(uint8_t byte) {
    // Flip a random bit
    return byte ^ (1 << (rand() % 8));
}

bool MockSerialBuffer::shouldInjectError() {
    if (inject_next_error_) {
        inject_next_error_ = false;
        return true;
    }
    
    if (config_.error_rate > 0.0) {
        return (static_cast<double>(rand()) / RAND_MAX) < config_.error_rate;
    }
    
    return false;
}

std::pair<std::shared_ptr<MockSerialBuffer>, std::shared_ptr<MockSerialBuffer>>
createSerialPair(const MockSerialBuffer::Config& config) {
    auto buffer1 = std::make_shared<MockSerialBuffer>(config);
    auto buffer2 = std::make_shared<MockSerialBuffer>(config);
    
    buffer1->connectTo(buffer2);
    
    return {buffer1, buffer2};
}

} // namespace transport
} // namespace motor_sim
