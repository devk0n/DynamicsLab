#include <iostream>

#include <glad/glad.h>

#include "renderer.h"

Renderer::Renderer(GLFWwindow* window)
    : m_Window(window) {
    if (!m_Window) {
        throw std::runtime_error("Renderer: Invalid GLFW window.");
    }
    glfwMakeContextCurrent(m_Window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }
    initOpenGL();

}

Renderer::~Renderer() {
    std::cout << "Renderer destroyed." << std::endl;
}

void Renderer::initOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glViewport(0, 0, 1920, 1080);
    std::cout << "OpenGL Initialized" << std::endl;
}

void Renderer::clearScreen(const glm::vec4& color) {
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw() {
    // Placeholder for rendering logic
}