#include "protocol/FeetechProtocol.hpp"
#include "utils/Logger.hpp"
#include <thread>

namespace motor_sim {
namespace protocol {

FeetechProtocol::FeetechProtocol(std::shared_ptr<transport::ITransport> transport,
                                const Config& config)
    : transport_(transport)
    , config_(config)
    , last_error_(0) {
}

FeetechProtocol::FeetechProtocol(std::shared_ptr<transport::ITransport> transport)
    : FeetechProtocol(transport, Config{}) {
}

std::optional<ProtocolPacket> FeetechProtocol::sendAndReceive(const ProtocolPacket& packet) {
    clearBuffer();
    
    if (!send(packet)) {
        LOG_ERROR("FeetechProtocol: Failed to send packet");
        return std::nullopt;
    }

    if (config_.response_delay.count() > 0) {
        std::this_thread::sleep_for(config_.response_delay);
    }

    return receive();
}

bool FeetechProtocol::send(const ProtocolPacket& packet) {
    auto data = packet.serialize();
    
    LOG_TRACE("FeetechProtocol: Sending ", data.size(), " bytes to ID ", 
             static_cast<int>(packet.getId()));
    
    size_t written = transport_->write(data.data(), data.size());
    transport_->flush();
    
    return written == data.size();
}

std::optional<ProtocolPacket> FeetechProtocol::receive() {
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        
        if (elapsed > config_.timeout) {
            LOG_WARN("FeetechProtocol: Receive timeout");
            return std::nullopt;
        }

        if (transport_->available() > 0) {
            auto packet = parseResponse();
            if (packet) {
                return packet;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool FeetechProtocol::ping(uint8_t id) {
    auto packet = ProtocolPacket::createInstruction(id, Instruction::PING);
    auto response = sendAndReceive(packet);
    
    if (response && response->getId() == id) {
        last_error_ = response->getError();
        return last_error_ == 0;
    }
    
    return false;
}

std::optional<std::vector<uint8_t>> FeetechProtocol::readData(uint8_t id, 
                                                              Register start_addr, 
                                                              uint8_t length) {
    auto packet = ProtocolPacket::createReadRequest(id, start_addr, length);
    auto response = sendAndReceive(packet);
    
    if (response && response->getId() == id) {
        last_error_ = response->getError();
        if (last_error_ == 0) {
            return response->getParams();
        }
    }
    
    return std::nullopt;
}

bool FeetechProtocol::writeData(uint8_t id, Register start_addr, 
                               const std::vector<uint8_t>& data) {
    auto packet = ProtocolPacket::createWriteRequest(id, start_addr, data);
    auto response = sendAndReceive(packet);
    
    if (response && response->getId() == id) {
        last_error_ = response->getError();
        return last_error_ == 0;
    }
    
    return false;
}

std::optional<uint16_t> FeetechProtocol::readWord(uint8_t id, Register addr) {
    auto data = readData(id, addr, 2);
    if (data && data->size() >= 2) {
        // Little-endian
        return static_cast<uint16_t>((*data)[0]) | 
               (static_cast<uint16_t>((*data)[1]) << 8);
    }
    return std::nullopt;
}

bool FeetechProtocol::writeWord(uint8_t id, Register addr, uint16_t value) {
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };
    return writeData(id, addr, data);
}

std::optional<uint8_t> FeetechProtocol::readByte(uint8_t id, Register addr) {
    auto data = readData(id, addr, 1);
    if (data && data->size() >= 1) {
        return (*data)[0];
    }
    return std::nullopt;
}

bool FeetechProtocol::writeByte(uint8_t id, Register addr, uint8_t value) {
    std::vector<uint8_t> data = {value};
    return writeData(id, addr, data);
}

void FeetechProtocol::clearBuffer() {
    transport_->clear();
}

bool FeetechProtocol::waitForResponse(std::chrono::milliseconds timeout) {
    auto start = std::chrono::steady_clock::now();
    
    while (transport_->available() < ProtocolPacket::MIN_PACKET_SIZE) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        
        if (elapsed > timeout) {
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    return true;
}

std::optional<ProtocolPacket> FeetechProtocol::parseResponse() {
    size_t available = transport_->available();
    if (available < ProtocolPacket::MIN_PACKET_SIZE) {
        return std::nullopt;
    }

    // Read available data
    std::vector<uint8_t> buffer(available);
    size_t read_count = transport_->read(buffer.data(), available);
    
    if (read_count == 0) {
        return std::nullopt;
    }

    auto result = ProtocolPacket::findPacket(buffer.data(), read_count);
    if (result) {
        LOG_TRACE("FeetechProtocol: Received packet from ID ", 
                 static_cast<int>(result->first.getId()));
        return result->first;
    }

    return std::nullopt;
}

} // namespace protocol
} // namespace motor_sim
