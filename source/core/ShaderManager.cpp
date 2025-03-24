#include "ShaderManager.h"
#include "Logger.h"
#include "OpenGLCore.h"

ShaderManager::~ShaderManager() {}

bool ShaderManager::loadShader(const std::string &name,
                               const std::string &vertexPath,
                               const std::string &fragmentPath,
                               const std::string &geometryPath) {
  // std::lock_guard lock(m_mutex);

  // Read shader files
  const std::string vertexSource = readFile(vertexPath);
  const std::string fragmentSource = readFile(fragmentPath);
  const std::string geometrySource = geometryPath.empty() ? "" : readFile(geometryPath);

  if (vertexSource.empty() || fragmentSource.empty()) {
    LOG_ERROR("Failed to read shader files for ", name);
    return false;
  }

  const unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
  const unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
  unsigned int geometryShader = 0;

  if (!geometrySource.empty()) {
    geometryShader = compileShader(GL_GEOMETRY_SHADER, geometrySource);
  }

  const unsigned int program = glCreateProgram();
  if (!linkProgram(program, vertexShader, fragmentShader, geometryShader)) {
    glDeleteProgram(program);
    return false;
  }

  m_shaders[name] = program;
  return true;
}

bool ShaderManager::loadComputeShader(const std::string &name,
                                      const std::string &computePath) {
  std::lock_guard lock(m_mutex);

  const std::string computeSource = readFile(computePath);
  if (computeSource.empty()) {
    LOG_ERROR("Failed to read compute shader file for ", name);
    return false;
  }

  const unsigned int computeShader = compileShader(GL_COMPUTE_SHADER, computeSource);
  const unsigned int program = glCreateProgram();
  glAttachShader(program, computeShader);
  glLinkProgram(program);

  if (!checkLinkErrors(program)) {
    glDeleteProgram(program);
    return false;
  }

  m_shaders[name] = program;
  return true;
}

unsigned int ShaderManager::getShader(const std::string &name) const {
  std::lock_guard lock(m_mutex);
  if (const auto it = m_shaders.find(name); it != m_shaders.end()) {
    return it->second;
  }
  return 0;
}

void ShaderManager::useShader(const std::string &name) {
  std::lock_guard lock(m_mutex);
  if (const auto it = m_shaders.find(name); it != m_shaders.end()) {
    glUseProgram(it->second);
    m_currentShader = it->second;
  } else {
    LOG_ERROR("Shader not found: ", name);
  }
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               const int value) const {
  if (const unsigned int shader = getShader(shaderName)) {
    glUniform1i(glGetUniformLocation(shader, uniformName.c_str()), value);
  }
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               const float value) const {
  if (const unsigned int shader = getShader(shaderName)) {
    glUniform1f(glGetUniformLocation(shader, uniformName.c_str()), value);
  }
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               const glm::mat4 &matrix) const {
  if (const unsigned int shader = getShader(shaderName)) {
    glUniformMatrix4fv(glGetUniformLocation(shader, uniformName.c_str()), 1, GL_FALSE, &matrix[0][0]);
  }
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               const glm::vec3 &value) const {
  if (const unsigned int shader = getShader(shaderName)) {
    glUniform3f(glGetUniformLocation(shader, uniformName.c_str()), value.x, value.y, value.z);
  }
}

void ShaderManager::setUniform(const std::string &uniformName, const int value) const {
  if (m_currentShader) {
    glUniform1i(glGetUniformLocation(m_currentShader, uniformName.c_str()), value);
  }
}

void ShaderManager::setUniform(const std::string &uniformName, const float value) const {
  if (m_currentShader) {
    glUniform1f(glGetUniformLocation(m_currentShader, uniformName.c_str()), value);
  }
}

void ShaderManager::setUniform(const std::string &uniformName, const glm::mat4 &matrix) const {
  if (m_currentShader) {
    glUniformMatrix4fv(glGetUniformLocation(m_currentShader, uniformName.c_str()), 1, GL_FALSE, &matrix[0][0]);
  }
}

void ShaderManager::setUniform(const std::string &uniformName, const glm::vec3 &value) const {
  if (m_currentShader) {
    glUniform3f(glGetUniformLocation(m_currentShader, uniformName.c_str()), value.x, value.y, value.z);
  }
}

unsigned int ShaderManager::getCurrentShader() const {
  return m_currentShader;
}

void ShaderManager::cleanup() {
  std::lock_guard lock(m_mutex);
  for (const auto &shader: m_shaders | std::views::values) {
    glDeleteProgram(shader);
  }
  m_shaders.clear();
}

std::string ShaderManager::readFile(const std::string &filePath) {
  const std::ifstream file(filePath);
  if (!file.is_open()) {
    LOG_ERROR("Failed to open file: ", filePath);
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

unsigned int ShaderManager::compileShader(const GLenum type, const std::string &source) {
  const unsigned int shader = glCreateShader(type);
  const char *src = source.c_str();
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  if (!checkCompileErrors(shader,
      (type == GL_VERTEX_SHADER) ? "VERTEX" :
      (type == GL_FRAGMENT_SHADER) ? "FRAGMENT" :
      (type == GL_GEOMETRY_SHADER) ? "GEOMETRY" : "COMPUTE")) {
    glDeleteShader(shader);
    return 0;
      }
  return shader;
}

bool ShaderManager::linkProgram(const unsigned int program,
                                const unsigned int vertexShader,
                                const unsigned int fragmentShader,
                                const unsigned int geometryShader) {
  glAttachShader(program, vertexShader);
  glAttachShader(program, fragmentShader);
  if (geometryShader != 0) {
    glAttachShader(program, geometryShader);
  }
  glLinkProgram(program);

  return checkLinkErrors(program);
}

bool ShaderManager::checkCompileErrors(const unsigned int shader,
                                       const std::string &type) {
  int success;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[1024];
    glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
    LOG_ERROR("Shader compilation error (", type, "): ", infoLog);
    return false;
  }
  return true;
}

bool ShaderManager::checkLinkErrors(const unsigned int program) {
  int success;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    char infoLog[1024];
    glGetProgramInfoLog(program, 1024, nullptr, infoLog);
    LOG_ERROR("Shader program linking error: ", infoLog);
    return false;
  }
  return true;
}
