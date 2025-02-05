#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <iostream>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Application {
public:

    explicit Application();
    ~Application();

    void run();

private:

    bool initialize();
    void mainLoop();
    void shutdown();

    std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
    bool isRunning{};
};

#endif // DYNAMICSLAB_APPLICATION_H
