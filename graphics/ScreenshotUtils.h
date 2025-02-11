#ifndef SCREENSHOTUTILS_H
#define SCREENSHOTUTILS_H
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <GLFW/glfw3.h>
#include "stb_image_write.h"

std::string generateTimestampedFilename() {
  // Define the fixed save path
  const std::string directory = "assets/images/";

  // Get the current time
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  // Format timestamp as YYYYMMDD_HHMMSS
  std::ostringstream oss;
  oss << directory << "screenshot_"
      << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S")
      << ".png";

  return oss.str();
}


void saveScreenshot(GLFWwindow *window) {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);

  std::vector<unsigned char> pixels(width * height * 3); // RGB format

  // Read pixels from OpenGL framebuffer
  glReadBuffer(GL_FRONT);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  // Flip image vertically (OpenGL's origin is bottom-left)
  std::vector<unsigned char> flippedPixels(width * height * 3);
  for (int y = 0; y < height; ++y) {
    memcpy(&flippedPixels[y * width * 3], &pixels[(height - 1 - y) * width * 3], width * 3);
  }

  // Generate timestamped filename
  std::string filename = generateTimestampedFilename();

  // Save as PNG
  if (stbi_write_png(filename.c_str(), width, height, 3, flippedPixels.data(), width * 3)) {
    std::cout << "Screenshot saved: " << filename << std::endl;
  } else {
    std::cerr << "Failed to save screenshot!" << std::endl;
  }
}

#endif // SCREENSHOTUTILS_H
