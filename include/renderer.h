#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include "GLFW/glfw3.h"
#include "glm/glm.hpp"

class Renderer {
public:
    Renderer(GLFWwindow* window);
    ~Renderer();

    static void clearScreen(const glm::dvec4& color);
    void draw();

    void handleMouseButton(int button, int action);
    void handleMouseMovement(double xpos, double ypos);
    void handleKeyboardInput(GLFWwindow* window, double deltaTime);

    glm::dvec3 getCameraPosition() const;

    glm::dvec3 getCameraOrientation() const;

private:
    GLFWwindow* m_Window;
    GLuint m_GridShaderProgram;

    glm::dvec3 m_CameraPos = glm::dvec3(0.0, 5.2, 10.5);
    glm::dvec3 m_CameraFront = glm::dvec3(0.0, -0.5, -1.0);
    glm::dvec3 m_CameraUp = glm::dvec3(0.0, 1.0, 0.0);
    double m_Yaw = -90.0;
    double m_Pitch = 0.0;
    double m_CameraSpeed = 5.0;
    bool m_RightMouseHeld = false;
    double m_LastMouseX = 400.0, m_LastMouseY = 300.0;
    bool m_FirstMouse = true;

    static void initOpenGL();

    void drawGrid(double size, int divisions, const glm::dvec3 &color) const;

    void updateViewMatrix();
    void updateProjectionMatrix(double aspectRatio) const;

    static GLuint compileShader(GLenum type, const char *source);
    static GLuint createShaderProgram(const char *vertexSource, const char *fragmentSource);

    static std::string loadShaderFromFile(const std::string &filepath);

};


#endif //DYNAMICSLAB_RENDERER_H
