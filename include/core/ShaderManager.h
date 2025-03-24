#ifndef SHADER_MANAGER_H
#define SHADER_MANAGER_H

#include <string>
#include <unordered_map>
#include <mutex>
#include <glm/glm.hpp>

#include "OpenGLCore.h"

class ShaderManager {
public:
  ShaderManager() = default;
  ~ShaderManager();

  bool loadShader(const std::string &name,
                  const std::string &vertexPath,
                  const std::string &fragmentPath,
                  const std::string &geometryPath = "");

  bool loadComputeShader(const std::string &name,
                         const std::string &computePath);

  unsigned int getShader(const std::string &name) const;

  // Bind a shader for use
  void useShader(const std::string &name);

  // Set uniforms with shader name
  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  int value) const;

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  float value) const;

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  const glm::mat4 &matrix) const;

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  const glm::vec3 &value) const;

  // Set uniforms for the currently bound shader
  void setUniform(const std::string &uniformName, int value) const;
  void setUniform(const std::string &uniformName, float value) const;
  void setUniform(const std::string &uniformName, const glm::mat4 &matrix) const;
  void setUniform(const std::string &uniformName, const glm::vec3 &value) const;

  // Get the currently bound shader program
  unsigned int getCurrentShader() const;

  void cleanup();

private:
  std::unordered_map<std::string, unsigned int> m_shaders;
  mutable std::mutex m_mutex;
  unsigned int m_currentShader = 0; // Track the currently bound shader

  static std::string readFile(const std::string &filePath);

  static unsigned int compileShader(GLenum type, const std::string &source);

  static bool linkProgram(unsigned int program,
                          unsigned int vertexShader,
                          unsigned int fragmentShader,
                          unsigned int geometryShader = 0);

  static bool checkCompileErrors(unsigned int shader, const std::string &type);
  static bool checkLinkErrors(unsigned int program);
};

#endif // SHADER_MANAGER_H
