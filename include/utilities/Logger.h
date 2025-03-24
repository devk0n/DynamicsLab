#ifndef LOGGER_H
#define LOGGER_H

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

// ANSI Color Codes for Unix/Linux/macOS & Windows 10+
#define RESET "\033[0m"
#define WHITE "\033[97m" // Bright White
#define COLOR_DEBUG "\x1B[95m" // Magenta
#define COLOR_INFO "\x1B[96m" // Cyan
#define COLOR_WARN "\x1B[38;2;255;165;0m" // Orange
#define COLOR_ERROR "\033[31m" // Red

class Logger {
public:
  enum class Level {
    Debug,
    Info,
    Warning,
    Error
  };

  struct ConsoleConfig {
    bool showTimestamps = true; // Show timestamps in logs
    bool showFileNames = true;  // Show file names and line numbers in logs
    bool showLevel = true;      // Show log level (DEBUG, INFO, etc.)
    bool enabled = true;        // Enable/disable console logging
  };

  Logger() = delete;

  static bool initialize(const std::string &filename = "") {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_initialized) {
      std::cerr << "Logger already initialized" << std::endl;
      return false;
    }

    if (!filename.empty()) {
      m_fileOut.open(filename, std::ios::app);
      if (!m_fileOut) {
        std::cerr << "Failed to open log file: " << filename << std::endl;
      }
    }

    #ifdef _WIN32
      // Enable ANSI color support for Windows Console
      HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
      DWORD mode = 0;
      GetConsoleMode(hOut, &mode);
      SetConsoleMode(hOut, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    #endif

    m_initialized = true;
    return m_initialized;
  }

  template<typename... Args>
  static void log(
      const Level level,
      const std::string &file,
      int line,
      Args &&...args) {

    if (level < m_logLevel) return;

    const std::string message =
        formatMessage(level, file, line, std::forward<Args>(args)...);
    const std::string coloredMessage =
        applyColor(level, message);

    std::lock_guard lock(m_mutex);
    if (m_consoleConfig.enabled) {
      std::cout << coloredMessage;
    }

    if (m_fileOut.is_open()) {
      m_fileOut << message; // No color in file output
      m_fileOut.flush();
    }
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
  inline static ConsoleConfig m_consoleConfig = {true, true, true, true}; // Inline initialization

  static std::string levelToString(const Level level) {
    switch (level) {
      case Level::Debug:
        return "DEBUG";
      case Level::Info:
        return "INFO";
      case Level::Warning:
        return "WARNING";
      case Level::Error:
        return "ERROR";
      default:
        return "UNKNOWN";
    }
  }

  static std::string applyColor(const Level level, const std::string &msg) {
    std::string color;
    switch (level) {
      case Level::Debug:
        color = COLOR_DEBUG;
        break;
      case Level::Info:
        color = COLOR_INFO;
        break;
      case Level::Warning:
        color = COLOR_WARN;
        break;
      case Level::Error:
        color = COLOR_ERROR;
        break;
      default:
        color = RESET;
        break;
    }

    // Keep timestamp white, but color the rest (including file name)
    if (const std::size_t firstBracketPos = msg.find("] [");
        firstBracketPos != std::string::npos) {
      return WHITE + msg.substr(0, firstBracketPos + 2) + // White timestamp
             color +
             msg.substr(firstBracketPos +
                        2) + // Colored message (including file name)
             RESET;
    }

    return color + msg + RESET;
  }

  static std::string extractFileName(const std::string &path) {
    // Find the last occurrence of '/' or '\'
    const std::size_t lastSlashPos = path.find_last_of("/\\");
    if (lastSlashPos == std::string::npos) {
      return path; // No path separator found, return the full string
    }
    return path.substr(lastSlashPos + 1); // Extract the file name
  }

  template<typename... Args>
  static std::string formatMessage(const Level level, const std::string &file,
                                   const int line, Args &&...args) {
    std::ostringstream ss;

    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);

    // Extract the file name from the full path
    const std::string fileName = extractFileName(file);

    // White timestamp + log level + file:line + message
    if (m_consoleConfig.showTimestamps) {
      struct tm timeInfo{};
      localtime_s(&timeInfo, &time);
      ss << "[" << std::put_time(&timeInfo, "%Y-%m-%d %H:%M:%S") << "] ";
    }
    if (m_consoleConfig.showLevel) {
      ss << "[" << levelToString(level) << "] ";
    }
    if (m_consoleConfig.showFileNames) {
      ss << "[" << fileName << ":" << line << "] ";
    }

    // Append all arguments to the stream
    (ss << ... << std::forward<Args>(args)) << "\n";

    return ss.str();
  }
};

// Macros for logging with file and line information
#define LOG_DEBUG(...)                                                         \
  Logger::log(Logger::Level::Debug, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...)                                                          \
  Logger::log(Logger::Level::Info, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)                                                          \
  Logger::log(Logger::Level::Warning, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...)                                                         \
  Logger::log(Logger::Level::Error, __FILE__, __LINE__, __VA_ARGS__)

#endif // LOGGER_H