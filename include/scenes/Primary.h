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

  void handleCameraMovement(double dt);

private:
  Camera m_camera;
  void showUI();
};

#endif // PRIMARY_H
