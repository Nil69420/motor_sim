#include <gtest/gtest.h>
#include "protocol/ProtocolPacket.hpp"
#include "utils/Checksum.hpp"

using namespace motor_sim::protocol;
using namespace motor_sim::utils;

class ProtocolPacketTest : public ::testing::Test {};

TEST_F(ProtocolPacketTest, CreatePingPacket) {
    auto packet = ProtocolPacket::createInstruction(1, Instruction::PING);
    
    EXPECT_EQ(packet.getId(), 1);
    EXPECT_EQ(packet.getInstruction(), Instruction::PING);
    EXPECT_TRUE(packet.isValid());
}

TEST_F(ProtocolPacketTest, SerializeDeserialize) {
    auto packet = ProtocolPacket::createInstruction(5, Instruction::PING);
    auto data = packet.serialize();
    
    EXPECT_GE(data.size(), ProtocolPacket::MIN_PACKET_SIZE);
    EXPECT_EQ(data[0], 0xFF);  // Header
    EXPECT_EQ(data[1], 0xFF);  // Header
    EXPECT_EQ(data[2], 5);     // ID
    
    auto deserialized = ProtocolPacket::deserialize(data.data(), data.size());
    
    ASSERT_TRUE(deserialized.has_value());
    EXPECT_EQ(deserialized->getId(), 5);
    EXPECT_EQ(deserialized->getInstruction(), Instruction::PING);
}

TEST_F(ProtocolPacketTest, ReadRequest) {
    auto packet = ProtocolPacket::createReadRequest(1, Register::PRESENT_POSITION_L, 2);
    auto data = packet.serialize();
    
    EXPECT_EQ(packet.getInstruction(), Instruction::READ_DATA);
    EXPECT_EQ(packet.getParams().size(), 2);
    EXPECT_EQ(packet.getParams()[0], static_cast<uint8_t>(Register::PRESENT_POSITION_L));
    EXPECT_EQ(packet.getParams()[1], 2);
}

TEST_F(ProtocolPacketTest, WriteRequest) {
    std::vector<uint8_t> write_data = {0x00, 0x02};  // Position 512
    auto packet = ProtocolPacket::createWriteRequest(1, Register::GOAL_POSITION_L, write_data);
    
    EXPECT_EQ(packet.getInstruction(), Instruction::WRITE_DATA);
    EXPECT_EQ(packet.getParams().size(), 3);  // Address + 2 bytes data
    EXPECT_EQ(packet.getParams()[0], static_cast<uint8_t>(Register::GOAL_POSITION_L));
}

TEST_F(ProtocolPacketTest, StatusResponse) {
    std::vector<uint8_t> response_data = {0x00, 0x02};
    auto packet = ProtocolPacket::createStatusResponse(1, 0, response_data);
    
    EXPECT_EQ(packet.getId(), 1);
    EXPECT_EQ(packet.getError(), 0);
    EXPECT_EQ(packet.getParams().size(), 2);
}

TEST_F(ProtocolPacketTest, InvalidPacket) {
    uint8_t bad_data[] = {0x00, 0x00, 0x01, 0x02};  // Wrong header
    auto packet = ProtocolPacket::deserialize(bad_data, 4);
    
    EXPECT_FALSE(packet.has_value());
}

TEST_F(ProtocolPacketTest, ChecksumValidation) {
    auto packet = ProtocolPacket::createInstruction(1, Instruction::PING);
    auto data = packet.serialize();
    
    // Corrupt checksum
    data[data.size() - 1] ^= 0xFF;
    
    auto deserialized = ProtocolPacket::deserialize(data.data(), data.size());
    EXPECT_FALSE(deserialized.has_value());
}

TEST_F(ProtocolPacketTest, FindPacketInStream) {
    // Create a packet
    auto packet = ProtocolPacket::createInstruction(1, Instruction::PING);
    auto packet_data = packet.serialize();
    
    // Add some garbage before it
    std::vector<uint8_t> stream = {0x00, 0x11, 0x22};
    stream.insert(stream.end(), packet_data.begin(), packet_data.end());
    
    auto result = ProtocolPacket::findPacket(stream.data(), stream.size());
    
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->first.getId(), 1);
    EXPECT_EQ(result->first.getInstruction(), Instruction::PING);
}

class ChecksumTest : public ::testing::Test {};

TEST_F(ChecksumTest, Calculate8bit) {
    uint8_t data[] = {0x01, 0x02, 0x01};  // ID, Length, Instruction
    uint8_t checksum = Checksum::calculate8bit(data, 3);
    
    // Checksum should be ~(0x01 + 0x02 + 0x01) = ~0x04 = 0xFB
    EXPECT_EQ(checksum, 0xFB);
}

TEST_F(ChecksumTest, Validate8bit) {
    uint8_t data[] = {0x01, 0x02, 0x01};
    uint8_t checksum = 0xFB;
    
    EXPECT_TRUE(Checksum::validate8bit(data, 3, checksum));
    EXPECT_FALSE(Checksum::validate8bit(data, 3, 0x00));
}

TEST_F(ChecksumTest, CRC16) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = Checksum::calculateCRC16(data, 4);
    
    EXPECT_TRUE(Checksum::validateCRC16(data, 4, crc));
    EXPECT_FALSE(Checksum::validateCRC16(data, 4, crc ^ 0xFFFF));
}
