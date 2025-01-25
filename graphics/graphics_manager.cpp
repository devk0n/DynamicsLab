#include <stdexcept>

#include "graphics_manager.h"

GraphicsManager::GraphicsManager() {
    WINDOW_WIDTH = 1280;
    WINDOW_HEIGHT = 720;
    WINDOW_TITLE = "DynamicsLab";

    initializeGLFW();
    createWindow();
}

GraphicsManager::~GraphicsManager() {
    cleanUp();
}

void GraphicsManager::initializeGLFW() {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW.");
    }
}

void GraphicsManager::createWindow() {
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE, nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window.");
    }
}

void GraphicsManager::run() {
    mainLoop();
}

void GraphicsManager::mainLoop() {
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
    }
}

void GraphicsManager::cleanUp() {
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}