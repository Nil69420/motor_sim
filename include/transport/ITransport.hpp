#pragma once

#include <cstddef>
#include <cstdint>

namespace motor_sim {
namespace transport {

/**
 * @brief Abstract transport interface for motor communication
 * 
 * This interface abstracts the underlying communication mechanism,
 * allowing the motor driver to work with various backends:
 * - Mock serial buffers (for testing)
 * - Virtual serial ports (for simulation)
 * - Real serial ports (for hardware)
 * - File-based I/O (for logging/replay)
 */
class ITransport {
public:
    virtual ~ITransport() = default;

    /**
     * @brief Write data to the transport
     * @param data Pointer to data buffer
     * @param size Number of bytes to write
     * @return Number of bytes actually written
     */
    virtual size_t write(const uint8_t* data, size_t size) = 0;

    /**
     * @brief Read data from the transport
     * @param data Pointer to destination buffer
     * @param size Maximum number of bytes to read
     * @return Number of bytes actually read
     */
    virtual size_t read(uint8_t* data, size_t size) = 0;

    /**
     * @brief Get number of bytes available to read
     * @return Number of bytes in receive buffer
     */
    virtual size_t available() const = 0;

    /**
     * @brief Flush the write buffer
     */
    virtual void flush() = 0;

    /**
     * @brief Clear both read and write buffers
     */
    virtual void clear() = 0;

    /**
     * @brief Check if transport is open/connected
     * @return true if ready for communication
     */
    virtual bool isOpen() const = 0;
};

} // namespace transport
} // namespace motor_sim
