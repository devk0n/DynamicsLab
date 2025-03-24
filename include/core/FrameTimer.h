#ifndef FRAME_TIMER_H
#define FRAME_TIMER_H

#include "OpenGLCore.h"

class FrameTimer {
public:
  FrameTimer() = default;

  void update() {
    const double currentTime = glfwGetTime();
    m_deltaTime = currentTime - m_lastFrameTime;
    m_lastFrameTime = currentTime;
    m_elapsedTime += m_deltaTime;
  }

  [[nodiscard]] double getDeltaTime() const { return m_deltaTime; }
  [[nodiscard]] double getElapsedTime() const { return m_elapsedTime; }

private:
  double m_lastFrameTime{0.0};
  double m_deltaTime{0.0};
  double m_elapsedTime{0.0};

};
#endif // FRAME_TIMER_H
