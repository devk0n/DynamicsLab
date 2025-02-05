#include "InputHandler.h"

#include <utility>

InputHandler::InputHandler(GLFWwindow* window) : m_window(window) {

}

InputHandler::~InputHandler() {
    glfwSetKeyCallback(m_window, nullptr);
}

