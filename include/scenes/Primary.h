#ifndef PRIMARY_H
#define PRIMARY_H

#include "Context.h"
#include "Renderer.h"
#include "Camera.h"
#include "Scene.h"
#include "SystemVisualizer.h"
#include "Dynamics.h"

class Primary final : public Scene {
public:
  explicit Primary(const Context &ctx)
      : Scene(ctx),
        m_systemVisualizer(ctx.renderer->getShaderManager()) {}

  void problemA();

  void problemB();

  void doublePendulum();

  void problemC();

  void vehicle();

  void space();

  bool load() override;
  void update(double dt) override;
  void render() override;
  void unload() override;

private:
  SystemVisualizer m_systemVisualizer;
  Proton::Dynamics m_system;
  Camera m_camera;

  float m_displayedFps = 0.0f;
  float m_fpsUpdateTimer = 0.0f;
  static constexpr float FPS_UPDATE_INTERVAL = 1.0f;

  mutable bool m_run = false;

  void showUI();

  void simulationControls() const;

  void bodyInfo() const;

  static void renderTimings(
    const std::vector<std::pair<std::string, double>> &timings);

  void handleCameraMovement(double dt);

  void setupDynamics();
};

#endif // PRIMARY_H
