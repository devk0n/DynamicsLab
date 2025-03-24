#ifndef PRIMARY_H
#define PRIMARY_H

#include "Camera.h"
#include "Scene.h"

class Primary final : public Scene {
public:
  explicit Primary(const Context &ctx) : Scene(ctx) {}
  bool load() override;
  void update(double dt) override;
  void render() override;
  void unload() override;


private:
  Camera m_camera;
  float m_displayedFps = 0.0f;
  float m_fpsUpdateTimer = 0.0f;
  static constexpr float FPS_UPDATE_INTERVAL = 1.0f;

  void showUI();
  void handleCameraMovement(double dt);

};

#endif // PRIMARY_H
