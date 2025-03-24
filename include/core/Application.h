#ifndef APPLICATION_H
#define APPLICATION_H

#include <memory>

class Application {
public:
  Application();
  ~Application();

  bool initialize() const;
  void run() const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

#endif // APPLICATION_H
