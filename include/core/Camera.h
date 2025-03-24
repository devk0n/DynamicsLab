#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

enum class CameraMovement { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

class Camera {
public:
  Camera();

  // Camera controls
  void processMouseMovement(float xOffset, float yOffset);
  void processKeyboardInput(CameraMovement direction, float deltaTime);
  void processScroll(float yOffset);
  void lookAt(const glm::vec3 &target);

  // Getters
  [[nodiscard]] glm::mat4 getViewMatrix() const;
  [[nodiscard]] glm::mat4 getProjectionMatrix() const;
  [[nodiscard]] glm::vec3 getPosition() const { return m_position; }
  [[nodiscard]] glm::quat getOrientation() const { return m_orientation; }
  [[nodiscard]] glm::vec3 getFront() const { return m_front; }
  [[nodiscard]] glm::vec3 getLeft() const { return m_left; }
  [[nodiscard]] glm::vec3 getUp() const { return m_up; }

  [[nodiscard]] float getFov() const { return m_fov; }
  [[nodiscard]] float getAspectRatio() const { return m_aspectRatio; }
  [[nodiscard]] float getNearClip() const { return m_nearClip; }
  [[nodiscard]] float getFarClip() const { return m_farClip; }
  [[nodiscard]] float getMovementSpeed() const { return m_movementSpeed; }

  // Setters
  void setPosition(const glm::vec3 position) { m_position = position; }
  void setOrientation(const glm::quat orientation) { m_orientation = orientation; }
  void setFront(const glm::vec3 front) { m_front = front; }
  void setLeft(const glm::vec3 left) { m_left = left; }
  void setUp(const glm::vec3 up) { m_up = up; }
  void setFov(const float fov) { m_fov = fov; }
  void setAspectRatio(const float aspectRatio) { m_aspectRatio = aspectRatio; }

  void setMovementSpeed(const float movementSpeed) { m_movementSpeed = movementSpeed; }

  static inline glm::vec3 s_forward = {1.0f, 0.0f, 0.0f};
  static inline glm::vec3 s_left = {0.0f, 1.0f, 0.0f};
  static inline glm::vec3 s_up = {0.0f, 0.0f, 1.0f};

private:
  void updateVectors();

  // Camera state
  glm::vec3 m_position;
  glm::quat m_orientation;

  // Camera vectors
  glm::vec3 m_front = {1.0f, 0.0f, 0.0f};
  glm::vec3 m_left = {0.0f, 1.0f, 0.0f};
  glm::vec3 m_up = {0.0f, 0.0f, 1.0f};

  // Camera options
  float m_movementSpeed;
  float m_mouseSensitivity;

  float m_fov;
  float m_aspectRatio;
  float m_nearClip;
  float m_farClip;
};

#endif // CAMERA_H
