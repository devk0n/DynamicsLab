#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

#include <memory>
#include <stack>

#include "Scene.h"

class SceneManager {
public:
  void pushScene(std::unique_ptr<Scene> scene);
  void popScene();
  void update(float dt);
  void render();

private:
  std::stack<std::unique_ptr<Scene>> m_scenes;
};

#endif // SCENE_MANAGER_H
