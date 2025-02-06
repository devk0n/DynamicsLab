#ifndef DYNAMICSLAB_SHADERUTILS_H
#define DYNAMICSLAB_SHADERUTILS_H


#include "glad/glad.h"
#include <string>

class ShaderUtils {
public:
    static std::string readFileContents(const std::string& filePath);
    static GLuint compileShader(GLenum shaderType, const std::string& source);
    static GLuint createShaderProgram(const std::string& vertexPath, const std::string& fragmentPath);
};


#endif //DYNAMICSLAB_SHADERUTILS_H
