#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include "GLFW/glfw3.h"
#include "glm/glm.hpp"

class Renderer {
public:
    Renderer(GLFWwindow* window);
    ~Renderer();

    void clearScreen(const glm::vec4& color);
    void draw();

    void handleMouseButton(int button, int action);
    void handleMouseMovement(double xpos, double ypos);
    void handleKeyboardInput(GLFWwindow* window, float deltaTime);

private:
    GLFWwindow* m_Window;
    GLuint m_GridShaderProgram;

    glm::vec3 m_CameraPos = glm::vec3(10.0f, 10.0f, 3.0f);
    glm::vec3 m_CameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 m_CameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
    float m_Yaw = -90.0f;
    float m_Pitch = 0.0f;
    float m_CameraSpeed = 5.0f;
    bool m_RightMouseHeld = false;
    float m_LastMouseX = 400.0f, m_LastMouseY = 300.0f; // Initial cursor position
    bool m_FirstMouse = true;

    void initOpenGL();

    void drawGrid(float size, int divisions, const glm::vec3 &color) const;

    void updateViewMatrix();
    void updateProjectionMatrix(float aspectRatio);

    GLuint compileShader(GLenum type, const char *source);
    GLuint createShaderProgram(const char *vertexSource, const char *fragmentSource);
};


#endif //DYNAMICSLAB_RENDERER_H
