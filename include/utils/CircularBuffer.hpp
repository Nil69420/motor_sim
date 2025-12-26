#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace motor_sim {
namespace utils {


template<typename T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t capacity)
        : buffer_(capacity + 1)  // Extra slot to distinguish full from empty
        , capacity_(capacity + 1)
        , head_(0)
        , tail_(0) {
    }

    /**
     * @brief Write data to the buffer
     * @param data Pointer to data
     * @param size Number of elements to write
     * @return Number of elements actually written
     */
    size_t write(const T* data, size_t size) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        size_t available = capacity_ - 1 - used();
        size_t to_write = std::min(size, available);
        
        for (size_t i = 0; i < to_write; ++i) {
            buffer_[head_] = data[i];
            head_ = (head_ + 1) % capacity_;
        }
        
        return to_write;
    }

    /**
     * @brief Read data from the buffer
     * @param data Pointer to destination buffer
     * @param size Maximum number of elements to read
     * @return Number of elements actually read
     */
    size_t read(T* data, size_t size) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        size_t available = used();
        size_t to_read = std::min(size, available);
        
        for (size_t i = 0; i < to_read; ++i) {
            data[i] = buffer_[tail_];
            tail_ = (tail_ + 1) % capacity_;
        }
        
        return to_read;
    }

    /**
     * @brief Peek at data without removing it
     * @param data Pointer to destination buffer
     * @param size Maximum number of elements to peek
     * @return Number of elements peeked
     */
    size_t peek(T* data, size_t size) const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        size_t available = used();
        size_t to_peek = std::min(size, available);
        
        size_t temp_tail = tail_;
        for (size_t i = 0; i < to_peek; ++i) {
            data[i] = buffer_[temp_tail];
            temp_tail = (temp_tail + 1) % capacity_;
        }
        
        return to_peek;
    }

    /**
     * @brief Get number of elements currently in buffer
     */
    size_t available() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return used();
    }

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return head_ == tail_;
    }

    /**
     * @brief Check if buffer is full
     */
    bool full() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ((head_ + 1) % capacity_) == tail_;
    }

    /**
     * @brief Clear the buffer
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_ = 0;
    }

    /**
     * @brief Get buffer capacity
     */
    size_t capacity() const {
        return capacity_ - 1;
    }

private:
    size_t used() const {
        if (head_ >= tail_) {
            return head_ - tail_;
        }
        return capacity_ - tail_ + head_;
    }

    std::vector<T> buffer_;
    size_t capacity_;
    size_t head_;  // Write position
    size_t tail_;  // Read position
    mutable std::mutex mutex_;
};

} // namespace utils
} // namespace motor_sim
