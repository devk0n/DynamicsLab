#ifndef DYNAMICSLAB_WINDOW_H
#define DYNAMICSLAB_WINDOW_H

#include <iostream>
#include <memory>
#include <glad/glad.h>

#include "GLFW/glfw3.h"

struct GLFWwindow;

class Window {
public:
    explicit Window(int width, int height, const char* title);
    ~Window();

    [[nodiscard]] bool shouldClose() const;
    static void pollEvents();
    void swapBuffers();

    [[nodiscard]] GLFWwindow* getWindow() const { return m_window.get(); }

private:
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> m_window;

};


#endif //DYNAMICSLAB_WINDOW_H
