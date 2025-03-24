#include "SceneManager.h"
#include "Logger.h"
#include "Scene.h"

void SceneManager::pushScene(
    std::unique_ptr<Scene> scene) {
  if (!scene) {
    LOG_ERROR("Attempted to push a null scene");
    return;
  }

  LOG_DEBUG("Pushing new scene. ");
  if (!scene->load()) {
    LOG_ERROR("Failed to load scene");
    return;
  }

  m_scenes.push(std::move(scene));
  LOG_DEBUG("Scene pushed.");
}

void SceneManager::popScene() {
  if (m_scenes.empty()) {
    LOG_WARN("No scenes to pop.");
    return;
  }

  m_scenes.top()->unload();
  m_scenes.pop();

  LOG_DEBUG("Scene popped. New top: %p",
            m_scenes.empty() ? nullptr : m_scenes.top().get());

  // Ensure new scene is loaded
  if (!m_scenes.empty()) {
    m_scenes.top()->load();
  }
}

void SceneManager::update(const float dt) {
  if (m_scenes.empty()) {
    LOG_WARN("No scenes to update");
    return;
  }

  m_scenes.top()->update(dt);
}

void SceneManager::render() {
  if (m_scenes.empty()) {
    LOG_WARN("No scenes to render");
    return;
  }

  m_scenes.top()->render();
}
