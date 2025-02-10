#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <string>

class WindowManager {
public:
    WindowManager();
    ~WindowManager();

    bool initialize(int width = 1920, int height = 1280, const std::string& title = "DynamicsLab");
    void pollEvents();
    void swapBuffers();
    bool shouldClose() const;
    void shutdown();

    GLFWwindow* getWindow() const;
    float getAspectRatio() const;
private:
    GLFWwindow* m_window;
    int m_width;
    int m_height;
};


#endif // WINDOWMANAGER_H
