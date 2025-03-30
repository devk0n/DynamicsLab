#include "Logger.h"
#include "Application.h"

int main() {
  // ───────────── Logger Setup ─────────────
  if (!Logger::initialize("log.txt")) {
    std::cerr << "Failed to initialize Logger.\n";
    return EXIT_FAILURE;
  }

  ConsoleConfig config;
  config.showTimestamps = false;
  config.showFileNames  = true;
  config.showLevel      = false;
  config.enabled        = true;
  Logger::setConsoleConfig(config);
  Logger::setLogLevel(Logger::Level::Debug);

  LOG_INFO("Logger initialized.");
  LOG_DEBUG("Logger initialized.");
  LOG_WARN("Logger initialized.");
  LOG_ERROR("Logger initialized.");

  // ───────────── Application Start ─────────────
  Application app;

  if (!app.initialize()) {
    LOG_ERROR("Failed to initialize Application.");
    return EXIT_FAILURE;
  }

  app.run();

  LOG_INFO("Application terminated successfully.");
  return EXIT_SUCCESS;
}
