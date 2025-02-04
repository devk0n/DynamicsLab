#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <memory>

#include "../window/Window.h"
#include "../graphics/Camera.h"
#include "../graphics/Renderer.h"
#include "../input/InputHandler.h"
#include "../gui/GuiManager.h"

class Application {
public:
    Application(int width, int height, const char* title);

    void run();

private:
    // Core Systems
    std::unique_ptr<Window> m_window;
    // std::unique_ptr<Camera> m_camera;
    std::unique_ptr<Renderer> m_renderer;
    // std::unique_ptr<InputHandler> m_inputHandler;
    std::unique_ptr<GuiManager> m_guiManager;

};

#endif // DYNAMICSLAB_APPLICATION_H
