#ifndef MESHDATA_H
#define MESHDATA_H

#include "Mesh.h"
#include <vector>

namespace MeshData {

    // === Predefined Cube ===
    const std::vector<Vertex> cubeVertices = {
        {{-0.5f, -0.5f, 0.5f},  {0.0f,  0.0f,  1.0f}},
        {{0.5f,  -0.5f, 0.5f},  {0.0f,  0.0f,  1.0f}},
        {{0.5f,  0.5f,  0.5f},  {0.0f,  0.0f,  1.0f}},
        {{-0.5f, 0.5f,  0.5f},  {0.0f,  0.0f,  1.0f}},

        {{-0.5f, -0.5f, -0.5f}, {0.0f,  0.0f,  -1.0f}},
        {{0.5f,  -0.5f, -0.5f}, {0.0f,  0.0f,  -1.0f}},
        {{0.5f,  0.5f,  -0.5f}, {0.0f,  0.0f,  -1.0f}},
        {{-0.5f, 0.5f,  -0.5f}, {0.0f,  0.0f,  -1.0f}},

        {{-0.5f, -0.5f, -0.5f}, {-1.0f, 0.0f,  0.0f}},
        {{-0.5f, -0.5f, 0.5f},  {-1.0f, 0.0f,  0.0f}},
        {{-0.5f, 0.5f,  0.5f},  {-1.0f, 0.0f,  0.0f}},
        {{-0.5f, 0.5f,  -0.5f}, {-1.0f, 0.0f,  0.0f}},

        {{0.5f,  -0.5f, -0.5f}, {1.0f,  0.0f,  0.0f}},
        {{0.5f,  -0.5f, 0.5f},  {1.0f,  0.0f,  0.0f}},
        {{0.5f,  0.5f,  0.5f},  {1.0f,  0.0f,  0.0f}},
        {{0.5f,  0.5f,  -0.5f}, {1.0f,  0.0f,  0.0f}},

        {{-0.5f, 0.5f,  -0.5f}, {0.0f,  1.0f,  0.0f}},
        {{-0.5f, 0.5f,  0.5f},  {0.0f,  1.0f,  0.0f}},
        {{0.5f,  0.5f,  0.5f},  {0.0f,  1.0f,  0.0f}},
        {{0.5f,  0.5f,  -0.5f}, {0.0f,  1.0f,  0.0f}},

        {{-0.5f, -0.5f, -0.5f}, {0.0f,  -1.0f, 0.0f}},
        {{-0.5f, -0.5f, 0.5f},  {0.0f,  -1.0f, 0.0f}},
        {{0.5f,  -0.5f, 0.5f},  {0.0f,  -1.0f, 0.0f}},
        {{0.5f,  -0.5f, -0.5f}, {0.0f,  -1.0f, 0.0f}}
    };

    // Predefined cube indices for indexed drawing
    const std::vector<unsigned int> cubeIndices = {
        0, 1, 2, 2, 3, 0,  // Front
        4, 5, 6, 6, 7, 4,  // Back
        8, 9, 10, 10, 11, 8,  // Left
        12, 13, 14, 14, 15, 12,  // Right
        16, 17, 18, 18, 19, 16,  // Top
        20, 21, 22, 22, 23, 20   // Bottom
    };

    constexpr float axisVertices[] = {
        // X Axis
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // Red
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // Red

        // Y Axis
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Green
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Green

        // Z Axis
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // Blue
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f   // Blue
    };

    // === Predefined Sphere ===
    inline std::vector<Vertex> generateSphereVertices(float radius, int sectors, int stacks) {
      std::vector<Vertex> vertices;

      const float PI = 3.14159265358979323846f;
      float sectorStep = 2 * PI / sectors;
      float stackStep = PI / stacks;

      for (int i = 0; i <= stacks; ++i) {
        float stackAngle = PI / 2 - i * stackStep; // From pi/2 to -pi/2
        float xy = radius * cosf(stackAngle);     // r * cos(u)
        float z = radius * sinf(stackAngle);      // r * sin(u)

        for (int j = 0; j <= sectors; ++j) {
          float sectorAngle = j * sectorStep;   // From 0 to 2pi

          // Vertex position
          float x = xy * cosf(sectorAngle);    // r * cos(u) * cos(v)
          float y = xy * sinf(sectorAngle);    // r * cos(u) * sin(v)

          // Vertex normal (normalized position)
          glm::vec3 normal = glm::normalize(glm::vec3(x, y, z));

          vertices.push_back({{x,        y,        z},
                              {normal.x, normal.y, normal.z}});
        }
      }

      return vertices;
    }

    inline std::vector<unsigned int> generateSphereIndices(int sectors, int stacks) {
      std::vector<unsigned int> indices;

      for (int i = 0; i < stacks; ++i) {
        int k1 = i * (sectors + 1); // Beginning of current stack
        int k2 = k1 + sectors + 1;  // Beginning of next stack

        for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
          // 2 triangles per sector excluding the first and last stacks
          if (i != 0) {
            indices.push_back(k1);
            indices.push_back(k2);
            indices.push_back(k1 + 1);
          }

          if (i != (stacks - 1)) {
            indices.push_back(k1 + 1);
            indices.push_back(k2);
            indices.push_back(k2 + 1);
          }
        }
      }

      return indices;
    }
}
#endif // MESHDATA_H