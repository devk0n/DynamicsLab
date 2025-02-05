#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include "glad/glad.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>


class Renderer {
public:
    Renderer();
    ~Renderer();

    bool initialize();
    void clear();
    void render();
    void shutdown();

    // Set these from outside (e.g., main.cpp) if you want a dynamic camera
    void setProjectionMatrix(const glm::mat4& projection);
    void setViewMatrix(const glm::mat4& view);

private:
    GLuint m_gridVAO;
    GLuint m_gridVBO;
    GLuint m_gridShaderProgram;

    glm::mat4 m_projectionMatrix = glm::perspective(
        glm::radians(45.0f),
        static_cast<float>(1920) / static_cast<float>(1280),
        0.01f,
        1000.0f
    );
    glm::mat4 m_viewMatrix;

    // Internal helpers
    bool initializeGrid();
    bool setupGrid();
    GLuint createShaderProgramFromFiles(const std::string& vertexFilePath,
                                        const std::string& fragmentFilePath);
    static std::string readFileContents(const std::string& filePath);


};


#endif //DYNAMICSLAB_RENDERER_H
