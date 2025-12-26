#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <mutex>

namespace motor_sim {
namespace utils {

enum class LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_level_ = level;
    }

    LogLevel getLevel() const {
        return current_level_;
    }

    template<typename... Args>
    void log(LogLevel level, const Args&... args) {
        if (level < current_level_) return;

        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S")
            << "." << std::setfill('0') << std::setw(3) << ms.count()
            << "] [" << levelToString(level) << "] ";

        ((oss << args), ...);
        oss << std::endl;

        std::cout << oss.str();
    }

    template<typename... Args>
    void trace(const Args&... args) { log(LogLevel::TRACE, args...); }

    template<typename... Args>
    void debug(const Args&... args) { log(LogLevel::DEBUG, args...); }

    template<typename... Args>
    void info(const Args&... args) { log(LogLevel::INFO, args...); }

    template<typename... Args>
    void warn(const Args&... args) { log(LogLevel::WARN, args...); }

    template<typename... Args>
    void error(const Args&... args) { log(LogLevel::ERROR, args...); }

    template<typename... Args>
    void fatal(const Args&... args) { log(LogLevel::FATAL, args...); }

private:
    Logger() : current_level_(LogLevel::INFO) {}
    
    std::string levelToString(LogLevel level) const {
        switch (level) {
            case LogLevel::TRACE: return "TRACE";
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO ";
            case LogLevel::WARN:  return "WARN ";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }

    LogLevel current_level_;
    std::mutex mutex_;
};

// Convenience macros
#define LOG_TRACE(...) motor_sim::utils::Logger::getInstance().trace(__VA_ARGS__)
#define LOG_DEBUG(...) motor_sim::utils::Logger::getInstance().debug(__VA_ARGS__)
#define LOG_INFO(...)  motor_sim::utils::Logger::getInstance().info(__VA_ARGS__)
#define LOG_WARN(...)  motor_sim::utils::Logger::getInstance().warn(__VA_ARGS__)
#define LOG_ERROR(...) motor_sim::utils::Logger::getInstance().error(__VA_ARGS__)
#define LOG_FATAL(...) motor_sim::utils::Logger::getInstance().fatal(__VA_ARGS__)

} // namespace utils
} // namespace motor_sim
