#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include <iostream>

#include "glm/glm.hpp"
#include "GLFW/glfw3.h"

#include "Camera.h"

class Renderer {
public:
    Renderer();
    ~Renderer();

    static void clearScreen();
    void draw();

};

#endif // DYNAMICSLAB_RENDERER_H
