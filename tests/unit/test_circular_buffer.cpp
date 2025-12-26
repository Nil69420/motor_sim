#include <gtest/gtest.h>
#include "utils/CircularBuffer.hpp"
#include <vector>

using namespace motor_sim::utils;

class CircularBufferTest : public ::testing::Test {
protected:
    CircularBuffer<uint8_t> buffer{10};
};

TEST_F(CircularBufferTest, InitialState) {
    EXPECT_TRUE(buffer.empty());
    EXPECT_FALSE(buffer.full());
    EXPECT_EQ(buffer.available(), 0);
    EXPECT_EQ(buffer.capacity(), 10);
}

TEST_F(CircularBufferTest, WriteRead) {
    uint8_t write_data[] = {1, 2, 3, 4, 5};
    size_t written = buffer.write(write_data, 5);
    
    EXPECT_EQ(written, 5);
    EXPECT_EQ(buffer.available(), 5);
    EXPECT_FALSE(buffer.empty());
    
    uint8_t read_data[5];
    size_t read_count = buffer.read(read_data, 5);
    
    EXPECT_EQ(read_count, 5);
    EXPECT_EQ(buffer.available(), 0);
    
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(read_data[i], write_data[i]);
    }
}

TEST_F(CircularBufferTest, Overflow) {
    uint8_t data[15] = {0};
    size_t written = buffer.write(data, 15);
    
    // Should only write up to capacity
    EXPECT_LE(written, buffer.capacity());
    EXPECT_TRUE(buffer.full());
}

TEST_F(CircularBufferTest, WrapAround) {
    uint8_t data1[] = {1, 2, 3, 4, 5};
    buffer.write(data1, 5);
    
    uint8_t temp[5];
    buffer.read(temp, 5);
    
    uint8_t data2[] = {6, 7, 8, 9, 10};
    buffer.write(data2, 5);
    
    uint8_t result[5];
    buffer.read(result, 5);
    
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(result[i], data2[i]);
    }
}

TEST_F(CircularBufferTest, Peek) {
    uint8_t write_data[] = {1, 2, 3};
    buffer.write(write_data, 3);
    
    uint8_t peek_data[3];
    size_t peeked = buffer.peek(peek_data, 3);
    
    EXPECT_EQ(peeked, 3);
    EXPECT_EQ(buffer.available(), 3);  // Data still in buffer
    
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(peek_data[i], write_data[i]);
    }
}

TEST_F(CircularBufferTest, Clear) {
    uint8_t data[] = {1, 2, 3};
    buffer.write(data, 3);
    
    buffer.clear();
    
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.available(), 0);
}
