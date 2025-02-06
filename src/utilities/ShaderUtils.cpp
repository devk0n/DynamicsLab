#include "ShaderUtils.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::string ShaderUtils::readFileContents(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::in | std::ios::binary);
    if (!file) {
        std::cerr << "[ShaderUtils] Cannot open file: " << filePath << "\n";
        return {};
    }
    std::stringstream buffer;
    buffer << file.rdbuf();

    return buffer.str();
}


GLuint ShaderUtils::compileShader(GLenum shaderType, const std::string& source) {
    GLuint shader = glCreateShader(shaderType);
    const char* srcPtr = source.c_str();
    glShaderSource(shader, 1, &srcPtr, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
        std::cerr << "[ShaderUtils::compileShader] Error:\n" << infoLog << + "\n";
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint ShaderUtils::createShaderProgram(const std::string& vertexPath, const std::string& fragmentPath) {
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, readFileContents(vertexPath));
    if (!vertexShader) {
        std::cerr << "[ShaderUtils] ERROR: Vertex shader failed to compile!\n";
        return 0;
    }

    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, readFileContents(fragmentPath));
    if (!fragmentShader) {
        std::cerr << "[ShaderUtils] ERROR: Fragment shader failed to compile!\n";
        glDeleteShader(vertexShader);
        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    // Check if linking was successful
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[1024];
        glGetProgramInfoLog(program, 1024, nullptr, infoLog);
        std::cerr << "[ShaderUtils] ERROR: Shader linking failed!\n" << infoLog << "\n";
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(program);
        return 0;
    }

    return program;
}

