#include "Application.h"
#include "Logger.h"

int main() {
  int status = EXIT_SUCCESS;

  // ───────────── Logger Setup ─────────────
  if (!Logger::initialize("log.txt")) {
    std::cerr << "Failed to initialize Logger.\n";
    status = EXIT_FAILURE;
  } else {
    ConsoleConfig config;
    config.showTimestamps = false;
    config.showFileNames  = true;
    config.showLevel      = false;
    config.enabled        = true;
    Logger::setConsoleConfig(config);
    Logger::setLogLevel(Logger::Level::Debug);

    LOG_INFO("Logger initialized.");

    // ───────────── Application Start ─────────────
    if (const Application app; !app.initialize()) {
      LOG_ERROR("Failed to initialize Application.");
      status = EXIT_FAILURE;
    } else {
      app.run();
      LOG_INFO("Application terminated successfully.");
    }
  }

  return status;
}
