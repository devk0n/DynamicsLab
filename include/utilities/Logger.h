#ifndef LOGGER_H
#define LOGGER_H

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

// ===== ANSI COLOR CODES =====
#define RESET        "\033[0m"
#define WHITE        "\033[97m"
#define GRAY         "\033[38;2;150;150;150m"
#define COLOR_DEBUG  "\033[38;2;180;140;255m"
#define COLOR_INFO   "\033[38;2;92;198;255m"
#define COLOR_WARN   "\033[38;2;255;191;0m"
#define COLOR_ERROR  "\033[38;2;255;92;92m"
#define BOLD         "\033[1m"
#define DIM          "\033[2m"

struct ConsoleConfig {
  bool showTimestamps = true;
  bool showFileNames  = true;
  bool showLevel      = true;
  bool enabled        = true;
};

class Logger {
public:
  enum class Level { Debug, Info, Warning, Error };

  Logger() = delete;

  static bool initialize(const std::string& filename = "") {
    std::lock_guard lock(m_mutex);

    if (m_initialized) {
      return true;
    }

    if (!filename.empty()) {
      m_fileOut.open(filename, std::ios::app);
      if (!m_fileOut.is_open()) {
        std::cerr << "Logger: Failed to open log file: " << filename << "\n";
        return false;
      }
    }

#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (GetConsoleMode(hOut, &mode)) {
      SetConsoleMode(hOut, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    }
#endif

    m_initialized = true;
    return true;
  }

  template<typename... Args>
  static void log(Level level, const std::string& file, int line, Args&&... args) {
#ifndef DISABLE_LOGGING
    if (level < m_logLevel) return;

    const std::string message = formatMessage(level, file, line, std::forward<Args>(args)...);
    const std::string colored = applyColor(level, message);

    std::lock_guard lock(m_mutex);
    if (m_consoleConfig.enabled) {
      std::cout << colored << std::flush;
    }

    if (m_fileOut.is_open()) {
      m_fileOut << message << std::flush;
    }
#endif
  }

  static void setLogLevel(const Level level) {
    std::lock_guard lock(m_mutex);
    m_logLevel = level;
  }

  static void setConsoleConfig(const ConsoleConfig& config) {
    std::lock_guard lock(m_mutex);
    m_consoleConfig = config;
  }

private:
  inline static std::ofstream m_fileOut;
  inline static auto m_logLevel = Level::Info;
  inline static bool m_initialized = false;
  inline static std::mutex m_mutex;
  inline static ConsoleConfig m_consoleConfig = {};

  static std::string levelToString(const Level level) {
    switch (level) {
      case Level::Debug:   return "DEBUG";
      case Level::Info:    return "INFO";
      case Level::Warning: return "WARNING";
      case Level::Error:   return "ERROR";
      default:             return "UNKNOWN";
    }
  }

  static std::string applyColor(const Level level, const std::string& msg) {
    std::string color;
    switch (level) {
      case Level::Debug:   color = COLOR_DEBUG; break;
      case Level::Info:    color = COLOR_INFO;  break;
      case Level::Warning: color = COLOR_WARN;  break;
      case Level::Error:   color = COLOR_ERROR; break;
      default:             color = RESET;       break;
    }

    if (const size_t bracket = msg.find("] ["); bracket != std::string::npos) {
      return WHITE + msg.substr(0, bracket + 2) + color + msg.substr(bracket + 2) + RESET;
    }
    return color + msg + RESET;
  }

  static std::string extractFileName(const std::string& path) {
    return std::filesystem::path(path).filename().string();
  }

  template<typename... Args>
  static std::string formatMessage(const Level level, const std::string& file, const int line, Args&&... args) {
    std::ostringstream ss;

    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    tm timeInfo{};

#if defined(_WIN32)
    localtime_s(&timeInfo, &time);
#else
    localtime_r(&time, &timeInfo);
#endif

    if (m_consoleConfig.showTimestamps) {
      ss << "[" << std::put_time(&timeInfo, "%Y-%m-%d %H:%M:%S") << "] ";
    }
    if (m_consoleConfig.showLevel) {
      ss << "[" << levelToString(level) << "] ";
    }
    if (m_consoleConfig.showFileNames) {
      ss << "[" << extractFileName(file) << ":" << line << "] ";
    }

    (ss << ... << std::forward<Args>(args)) << "\n";
    return ss.str();
  }
};

// ==== LOGGING MACROS ====
#ifndef DISABLE_LOGGING
#define LOG_DEBUG(...)  Logger::log(Logger::Level::Debug,   __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...)   Logger::log(Logger::Level::Info,    __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)   Logger::log(Logger::Level::Warning, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...)  Logger::log(Logger::Level::Error,   __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_DEBUG(...)
#define LOG_INFO(...)
#define LOG_WARN(...)
#define LOG_ERROR(...)
#endif

#endif // LOGGER_H
