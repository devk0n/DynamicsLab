#ifndef DYNAMICSLAB_WINDOW_H
#define DYNAMICSLAB_WINDOW_H
#include <iostream>

#include <GLFW/glfw3.h>

class Window {
public:
    Window(int width, int height, const char* title);
    ~Window();

    bool shouldClose();
    void pollEvents();
    void swapBuffers();
    GLFWwindow* getWindow() { return m_window; }

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

private:
    GLFWwindow* m_window;
    int m_width, m_height;
    const char* m_title;
};


#endif //DYNAMICSLAB_WINDOW_H
