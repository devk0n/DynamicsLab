//
// Created by devkon on 03/02/2025.
//

#ifndef DYNAMICSLAB_UIMANAGER_H
#define DYNAMICSLAB_UIMANAGER_H

#include "../window/Window.h"

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <GLFW/glfw3.h>


class UIManager{
public:
    UIManager(Window& window);
    ~UIManager();

    void renderUI();

private:

    void beginFrame();

    void endFrame();

    void test();
};


#endif //DYNAMICSLAB_UIMANAGER_H
