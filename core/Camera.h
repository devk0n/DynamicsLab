#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
  // Constructor
  explicit Camera(glm::vec3 position = glm::vec3(0.0f, 3.0f, 0.0f),
                  glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f),
                  float yaw = -90.0f, float pitch = 0.0f);

  // Get view and projection matrices
  [[nodiscard]] glm::mat4 getViewMatrix() const;

  [[nodiscard]] glm::mat4 getProjectionMatrix(float aspectRatio, float near = 0.1f, float far = 1000.0f) const;

  // Camera movement
  void moveForward(float deltaTime);

  void moveBackward(float deltaTime);

  void moveLeft(float deltaTime);

  void moveRight(float deltaTime);

  void moveUp(float deltaTime);

  void moveDown(float deltaTime);

  // Mouse look
  void processMouseMovement(float xOffset, float yOffset, bool constrainPitch = true);

  // Getters
  [[nodiscard]] glm::vec3 getPosition() const;

  [[nodiscard]] glm::vec3 getFront() const;

private:
  // Camera attributes
  glm::vec3 m_position;
  glm::vec3 m_front;
  glm::vec3 m_up;
  glm::vec3 m_right;
  glm::vec3 m_worldUp;

  // Euler angles
  float m_yaw;
  float m_pitch;

  // Camera options
  float m_movementSpeed;
  float m_mouseSensitivity;

  // Update camera vectors based on Euler angles
  void updateCameraVectors();
};


#endif // CAMERA_H