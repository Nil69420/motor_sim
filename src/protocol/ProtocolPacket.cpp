#include "protocol/ProtocolPacket.hpp"
#include "utils/Checksum.hpp"
#include "utils/Logger.hpp"
#include <algorithm>

namespace motor_sim {
namespace protocol {

ProtocolPacket::ProtocolPacket()
    : id_(0)
    , length_(2)
    , instruction_(Instruction::PING)
    , error_(0)
    , is_valid_(false)
    , is_response_(false) {
}

ProtocolPacket ProtocolPacket::createInstruction(uint8_t id, Instruction instruction,
                                                 const std::vector<uint8_t>& params) {
    ProtocolPacket packet;
    packet.id_ = id;
    packet.instruction_ = instruction;
    packet.params_ = params;
    packet.error_ = 0;
    packet.is_response_ = false;
    packet.updateLength();
    packet.is_valid_ = true;
    return packet;
}

ProtocolPacket ProtocolPacket::createReadRequest(uint8_t id, Register start_addr, uint8_t length) {
    std::vector<uint8_t> params = {
        static_cast<uint8_t>(start_addr),
        length
    };
    return createInstruction(id, Instruction::READ_DATA, params);
}

ProtocolPacket ProtocolPacket::createWriteRequest(uint8_t id, Register start_addr,
                                                  const std::vector<uint8_t>& data) {
    std::vector<uint8_t> params;
    params.push_back(static_cast<uint8_t>(start_addr));
    params.insert(params.end(), data.begin(), data.end());
    return createInstruction(id, Instruction::WRITE_DATA, params);
}

ProtocolPacket ProtocolPacket::createStatusResponse(uint8_t id, uint8_t error,
                                                    const std::vector<uint8_t>& params) {
    ProtocolPacket packet;
    packet.id_ = id;
    packet.instruction_ = Instruction::READ_DATA;  // Not used in status response
    packet.error_ = error;
    packet.params_ = params;
    packet.is_response_ = true;
    packet.updateLength();
    packet.is_valid_ = true;
    return packet;
}

std::vector<uint8_t> ProtocolPacket::serialize() const {
    std::vector<uint8_t> buffer;
    
    // Headers
    buffer.push_back(HEADER_0);
    buffer.push_back(HEADER_1);
    
    // ID
    buffer.push_back(id_);
    
    // Length
    buffer.push_back(length_);
    
    if (is_response_) {
        // Status packet: FF FF ID LEN ERR PARAMS... CHECKSUM
        buffer.push_back(error_);
    } else {
        // Instruction packet: FF FF ID LEN INS PARAMS... CHECKSUM
        buffer.push_back(static_cast<uint8_t>(instruction_));
    }
    
    // Parameters
    buffer.insert(buffer.end(), params_.begin(), params_.end());
    
    // Checksum calculation: ~(ID + Length + INS/ERR + Params)
    std::vector<uint8_t> checksum_data;
    checksum_data.push_back(id_);
    checksum_data.push_back(length_);
    checksum_data.push_back(is_response_ ? error_ : static_cast<uint8_t>(instruction_));
    checksum_data.insert(checksum_data.end(), params_.begin(), params_.end());
    
    uint8_t checksum = utils::Checksum::calculate8bit(checksum_data.data(), 
                                                      checksum_data.size());
    buffer.push_back(checksum);
    
    return buffer;
}

std::optional<ProtocolPacket> ProtocolPacket::deserialize(const uint8_t* data, size_t length) {
    if (length < MIN_PACKET_SIZE) {
        return std::nullopt;
    }

    // Verify headers
    if (data[0] != HEADER_0 || data[1] != HEADER_1) {
        return std::nullopt;
    }

    ProtocolPacket packet;
    packet.id_ = data[2];
    packet.length_ = data[3];

   
    size_t total_size = 4 + packet.length_;
    if (length < total_size) {
        return std::nullopt;
    }

    size_t idx = 4;
    uint8_t ins_or_err = data[idx++];
    
    
    bool is_response = (ins_or_err != static_cast<uint8_t>(Instruction::PING) &&
                        ins_or_err != static_cast<uint8_t>(Instruction::READ_DATA) &&
                        ins_or_err != static_cast<uint8_t>(Instruction::WRITE_DATA) &&
                        ins_or_err != static_cast<uint8_t>(Instruction::REG_WRITE) &&
                        ins_or_err != static_cast<uint8_t>(Instruction::ACTION) &&
                        ins_or_err != static_cast<uint8_t>(Instruction::SYNC_WRITE) &&
                        ins_or_err != static_cast<uint8_t>(Instruction::RESET));
    
    packet.is_response_ = is_response;
    if (is_response) {
        packet.error_ = ins_or_err;
        packet.instruction_ = Instruction::READ_DATA;  // Default
    } else {
        packet.instruction_ = static_cast<Instruction>(ins_or_err);
        packet.error_ = 0;
    }
    
    // Parameters: length field includes ins/err (1 byte) + params + checksum (1 byte)

    size_t param_count = packet.length_ - 2;
    for (size_t i = 0; i < param_count; ++i) {
        packet.params_.push_back(data[idx++]);
    }

    uint8_t received_checksum = data[idx];
    std::vector<uint8_t> checksum_data;
    checksum_data.push_back(packet.id_);
    checksum_data.push_back(packet.length_);
    checksum_data.push_back(ins_or_err);
    checksum_data.insert(checksum_data.end(), packet.params_.begin(), packet.params_.end());
    
    if (!utils::Checksum::validate8bit(checksum_data.data(), 
                                       checksum_data.size(), 
                                       received_checksum)) {
        LOG_WARN("ProtocolPacket: Checksum mismatch");
        packet.is_valid_ = false;
        return std::nullopt;
    }

    packet.is_valid_ = true;
    return packet;
}

std::optional<std::pair<ProtocolPacket, size_t>>
ProtocolPacket::findPacket(const uint8_t* data, size_t length) {
    for (size_t i = 0; i + 1 < length; ++i) {
        if (data[i] == HEADER_0 && data[i + 1] == HEADER_1) {
            if (i + 4 > length) {
                return std::nullopt;
            }
            
            auto packet = deserialize(data + i, length - i);
            if (packet) {
               
                size_t packet_size = 4 + packet->length_;
                return std::make_pair(*packet, i + packet_size);
            }
        }
    }
    
    return std::nullopt;
}

void ProtocolPacket::updateLength() {
    // Length = Instruction/Error (1) + Parameters (N) + Checksum (1)
    length_ = 2 + params_.size();
}

uint8_t ProtocolPacket::calculateChecksum() const {
    std::vector<uint8_t> data;
    data.push_back(id_);
    data.push_back(length_);
    data.push_back(is_response_ ? error_ : static_cast<uint8_t>(instruction_));
    data.insert(data.end(), params_.begin(), params_.end());
    
    return utils::Checksum::calculate8bit(data.data(), data.size());
}

bool ProtocolPacket::validateChecksum(uint8_t checksum) const {
    return calculateChecksum() == checksum;
}

} // namespace protocol
} // namespace motor_sim
