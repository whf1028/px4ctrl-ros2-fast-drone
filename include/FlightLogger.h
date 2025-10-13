#ifndef FLIGHT_LOGGER_H
#define FLIGHT_LOGGER_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <fstream>
#include <memory>
#include <mutex>
#include <map>

namespace px4ctrl {

/**
 * @brief 飞行日志记录器
 * 统一的日志记录接口，支持按类别分类记录
 */
class FlightLogger {
public:
    // 日志级别
    enum class LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        FATAL = 4
    };
    
    // 日志类别
    enum class LogCategory {
        SCRIPT = 0,
        PX4CTRL = 1,
        ROS2 = 2,
        SYSTEM = 3,
        FLIGHT_PHASE = 4,
        CONTROLLER = 5,
        SENSOR = 6,
        MISSION = 7,
        TOPIC = 8
    };
    
    // 单例模式
    static FlightLogger& getInstance();
    
    // 初始化日志系统
    bool initialize(const std::string& log_base_dir, const std::string& session_id);
    
    // 记录日志
    void log(LogLevel level, LogCategory category, const std::string& message);
    void log(LogLevel level, LogCategory category, const char* format, ...);
    
    // 便捷方法
    void debug(LogCategory category, const std::string& message);
    void info(LogCategory category, const std::string& message);
    void warn(LogCategory category, const std::string& message);
    void error(LogCategory category, const std::string& message);
    void fatal(LogCategory category, const std::string& message);
    
    // 设置日志级别
    void setLogLevel(LogLevel level);
    
    // 检查是否启用日志
    bool isEnabled() const { return enabled_; }
    
    // 获取日志文件路径
    std::string getLogFilePath(LogCategory category) const;

private:
    FlightLogger() = default;
    ~FlightLogger() = default;
    FlightLogger(const FlightLogger&) = delete;
    FlightLogger& operator=(const FlightLogger&) = delete;
    
    // 内部方法
    std::string getCurrentTimestamp();
    std::string getLogLevelString(LogLevel level) const;
    std::string getLogCategoryString(LogCategory category) const;
    std::string getLogFileName(LogCategory category) const;
    std::ofstream& getLogFile(LogCategory category);
    
    // 成员变量
    bool enabled_ = false;
    std::string log_base_dir_;
    std::string session_id_;
    LogLevel current_level_ = LogLevel::INFO;
    
    // 日志文件流
    std::map<LogCategory, std::unique_ptr<std::ofstream>> log_files_;
    std::mutex log_mutex_;
};

// 便捷宏定义
#define FLIGHT_LOG_DEBUG(category, ...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::DEBUG, px4ctrl::FlightLogger::LogCategory::category, __VA_ARGS__)

#define FLIGHT_LOG_INFO(category, ...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::INFO, px4ctrl::FlightLogger::LogCategory::category, __VA_ARGS__)

#define FLIGHT_LOG_WARN(category, ...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::WARN, px4ctrl::FlightLogger::LogCategory::category, __VA_ARGS__)

#define FLIGHT_LOG_ERROR(category, ...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::ERROR, px4ctrl::FlightLogger::LogCategory::category, __VA_ARGS__)

#define FLIGHT_LOG_FATAL(category, ...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::FATAL, px4ctrl::FlightLogger::LogCategory::category, __VA_ARGS__)

// 话题监听日志宏定义
#define LISTENER_LOG_DEBUG(...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::DEBUG, px4ctrl::FlightLogger::LogCategory::TOPIC, __VA_ARGS__)

#define LISTENER_LOG_INFO(...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::INFO, px4ctrl::FlightLogger::LogCategory::TOPIC, __VA_ARGS__)

#define LISTENER_LOG_WARN(...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::WARN, px4ctrl::FlightLogger::LogCategory::TOPIC, __VA_ARGS__)

#define LISTENER_LOG_ERROR(...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::ERROR, px4ctrl::FlightLogger::LogCategory::TOPIC, __VA_ARGS__)

#define LISTENER_LOG_FATAL(...) \
    px4ctrl::FlightLogger::getInstance().log(px4ctrl::FlightLogger::LogLevel::FATAL, px4ctrl::FlightLogger::LogCategory::TOPIC, __VA_ARGS__)

} // namespace px4ctrl

#endif // FLIGHT_LOGGER_H
