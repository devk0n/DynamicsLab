#ifndef DYNAMICSLAB_WINDOW_H
#define DYNAMICSLAB_WINDOW_H
#include <iostream>
#include <memory>

#include "GLFW/glfw3.h"

class Window {
public:
    Window(int width, int height, const char* title);

    [[nodiscard]] bool shouldClose() const;
    static void pollEvents();
    void swapBuffers();

private:
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> m_window;
};


#endif //DYNAMICSLAB_WINDOW_H
