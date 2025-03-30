#ifndef SCOPED_TIMER_H
#define SCOPED_TIMER_H

#include <string>
#include <utility>
#include <vector>

#include "OpenGLCore.h"

class ScopedTimer {
public:
  ScopedTimer(const std::string& label, FrameTimer& timer)
  : m_label(label), m_records(timer.getTimings()), m_start(glfwGetTime()) {}

  ~ScopedTimer() {
    double end = glfwGetTime();
    m_records.emplace_back(m_label, (end - m_start) * 1000.0); // ms
  }

private:
  std::string m_label;
  std::vector<std::pair<std::string, double>>& m_records;
  double m_start;
};

#endif // SCOPED_TIMER_H
