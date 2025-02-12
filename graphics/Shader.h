#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h> // Include glad to get OpenGL function pointers
#include <glm/glm.hpp> // Include GLM for math types like glm::mat4, glm::vec3, etc.
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {
public:
  // Constructor reads and builds the shader
  Shader();

  // Use/activate the shader
  void use() const;

  // Utility functions for setting uniforms
  void setBool(const std::string &name, bool value) const;

  void setInt(const std::string &name, int value) const;

  void setFloat(const std::string &name, float value) const;

  void setVec3(const std::string &name, const glm::vec3 &value) const;

  void setVec3(const std::string &name, float x, float y, float z) const;

  void setMat4(const std::string &name, const glm::mat4 &mat) const;

  bool loadShader(const std::string &vertexPath, const std::string &fragmentPath);

  // Cleanup
  void cleanup() const;

private:
  GLuint ID; // Program ID

  // Helper function to read a file into a string
  std::string readFile(const std::string &filePath);

  // Helper function to compile a shader
  GLuint compileShader(const char *shaderCode, GLenum shaderType) {
    GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &shaderCode, nullptr);
    glCompileShader(shader);
    checkCompileErrors(shader, shaderType == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT");
    return shader;
  }

  // Helper function to check for shader compilation/linking errors
  void checkCompileErrors(GLuint shader, const std::string &type);
};

#endif // SHADER_H