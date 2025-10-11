#include "FlightLogger.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <iostream>

namespace px4ctrl {

FlightLogger& FlightLogger::getInstance() {
    static FlightLogger instance;
    return instance;
}

bool FlightLogger::initialize(const std::string& log_base_dir, const std::string& session_id) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    log_base_dir_ = log_base_dir;
    session_id_ = session_id;
    
    // 检查环境变量
    const char* log_enabled = std::getenv("FLIGHT_LOG_ENABLED");
    if (log_enabled && std::string(log_enabled) == "true") {
        enabled_ = true;
    } else {
        enabled_ = false;
        return true; // 日志未启用，但初始化成功
    }
    
    // 检查日志级别
    const char* log_level = std::getenv("FLIGHT_LOG_LEVEL");
    if (log_level) {
        std::string level_str = log_level;
        if (level_str == "DEBUG") current_level_ = LogLevel::DEBUG;
        else if (level_str == "INFO") current_level_ = LogLevel::INFO;
        else if (level_str == "WARN") current_level_ = LogLevel::WARN;
        else if (level_str == "ERROR") current_level_ = LogLevel::ERROR;
        else if (level_str == "FATAL") current_level_ = LogLevel::FATAL;
    }
    
    // 创建日志目录
    try {
        std::filesystem::create_directories(log_base_dir_ + "/px4ctrl");
        std::filesystem::create_directories(log_base_dir_ + "/script");
        std::filesystem::create_directories(log_base_dir_ + "/ros2");
        std::filesystem::create_directories(log_base_dir_ + "/system");
        std::filesystem::create_directories(log_base_dir_ + "/flight_phases");
        std::filesystem::create_directories(log_base_dir_ + "/controller");
        std::filesystem::create_directories(log_base_dir_ + "/sensor");
        std::filesystem::create_directories(log_base_dir_ + "/mission");
    } catch (const std::exception& e) {
        std::cerr << "Failed to create log directories: " << e.what() << std::endl;
        return false;
    }
    
    return true;
}

void FlightLogger::log(LogLevel level, LogCategory category, const std::string& message) {
    if (!enabled_ || level < current_level_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    std::string timestamp = getCurrentTimestamp();
    std::string level_str = getLogLevelString(level);
    std::string category_str = getLogCategoryString(category);
    
    std::ofstream& log_file = getLogFile(category);
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] [" << level_str << "] [" << category_str << "] " << message << std::endl;
        log_file.flush();
    }
}

void FlightLogger::log(LogLevel level, LogCategory category, const char* format, ...) {
    if (!enabled_ || level < current_level_) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    log(level, category, std::string(buffer));
}

void FlightLogger::debug(LogCategory category, const std::string& message) {
    log(LogLevel::DEBUG, category, message);
}

void FlightLogger::info(LogCategory category, const std::string& message) {
    log(LogLevel::INFO, category, message);
}

void FlightLogger::warn(LogCategory category, const std::string& message) {
    log(LogLevel::WARN, category, message);
}

void FlightLogger::error(LogCategory category, const std::string& message) {
    log(LogLevel::ERROR, category, message);
}

void FlightLogger::fatal(LogCategory category, const std::string& message) {
    log(LogLevel::FATAL, category, message);
}

void FlightLogger::setLogLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    current_level_ = level;
}

std::string FlightLogger::getLogFilePath(LogCategory category) const {
    std::string category_dir = getLogCategoryString(category);
    std::string filename = getLogFileName(category);
    return log_base_dir_ + "/" + category_dir + "/" + filename;
}

std::string FlightLogger::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

std::string FlightLogger::getLogLevelString(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARN: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

std::string FlightLogger::getLogCategoryString(LogCategory category) const {
    switch (category) {
        case LogCategory::SCRIPT: return "script";
        case LogCategory::PX4CTRL: return "px4ctrl";
        case LogCategory::ROS2: return "ros2";
        case LogCategory::SYSTEM: return "system";
        case LogCategory::FLIGHT_PHASE: return "flight_phases";
        case LogCategory::CONTROLLER: return "controller";
        case LogCategory::SENSOR: return "sensor";
        case LogCategory::MISSION: return "mission";
        default: return "unknown";
    }
}

std::string FlightLogger::getLogFileName(LogCategory category) const {
    std::string category_str = getLogCategoryString(category);
    return session_id_ + "_" + category_str + ".log";
}

std::ofstream& FlightLogger::getLogFile(LogCategory category) {
    if (log_files_.find(category) == log_files_.end()) {
        std::string filepath = getLogFilePath(category);
        log_files_[category] = std::make_unique<std::ofstream>(filepath, std::ios::app);
    }
    return *log_files_[category];
}

} // namespace px4ctrl
