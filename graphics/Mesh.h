#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include "Shader.h"

// Vertex structure without texture coordinates
struct Vertex {
  glm::vec3 Position; // Vertex position
  glm::vec3 Normal;   // Vertex normal (for lighting)
};

class Mesh {
public:
  Mesh(const std::vector<Vertex> &vertices, const std::vector<GLuint> &indices);

  void draw(const Shader &shader) const;

  void cleanup();

private:
  std::vector<Vertex> vertices;
  std::vector<GLuint> indices;

  GLuint VAO, VBO, EBO;

  void setupMesh();
};

#endif // MESH_H