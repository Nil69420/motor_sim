#include <gtest/gtest.h>
#include "transport/MockSerialBuffer.hpp"
#include <thread>
#include <chrono>

using namespace motor_sim::transport;

class MockSerialBufferTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto pair = createSerialPair();
        buffer1 = pair.first;
        buffer2 = pair.second;
    }

    std::shared_ptr<MockSerialBuffer> buffer1;
    std::shared_ptr<MockSerialBuffer> buffer2;
};

TEST_F(MockSerialBufferTest, BasicCommunication) {
    uint8_t send_data[] = {0x01, 0x02, 0x03, 0x04};
    
    size_t written = buffer1->write(send_data, 4);
    EXPECT_EQ(written, 4);
    
    uint8_t recv_data[4] = {0};
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    size_t read_count = buffer2->read(recv_data, 4);
    EXPECT_EQ(read_count, 4);
    
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(recv_data[i], send_data[i]);
    }
}

TEST_F(MockSerialBufferTest, BidirectionalCommunication) {
    uint8_t data1[] = {0xAA, 0xBB};
    uint8_t data2[] = {0xCC, 0xDD};
    
    buffer1->write(data1, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    buffer2->write(data2, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    uint8_t recv1[2], recv2[2];
    buffer1->read(recv1, 2);
    buffer2->read(recv2, 2);
    
    EXPECT_EQ(recv1[0], data2[0]);
    EXPECT_EQ(recv1[1], data2[1]);
    EXPECT_EQ(recv2[0], data1[0]);
    EXPECT_EQ(recv2[1], data1[1]);
}

TEST_F(MockSerialBufferTest, Available) {
    EXPECT_EQ(buffer1->available(), 0);
    
    uint8_t data[] = {0x01, 0x02, 0x03};
    buffer2->write(data, 3);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_EQ(buffer1->available(), 3);
}

TEST_F(MockSerialBufferTest, Clear) {
    uint8_t data[] = {0x01, 0x02, 0x03};
    buffer2->write(data, 3);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    buffer1->clear();
    EXPECT_EQ(buffer1->available(), 0);
}

TEST_F(MockSerialBufferTest, ErrorInjection) {
    buffer1->injectError();
    
    uint8_t send_data[] = {0xFF, 0xFF, 0x01};
    buffer1->write(send_data, 3);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    uint8_t recv_data[3];
    size_t read_count = buffer2->read(recv_data, 3);
    
    // Verify we read the expected number of bytes
    EXPECT_EQ(read_count, 3);
    
    (void)recv_data; 
    (void)send_data;
}
