#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include <glad/glad.h>

class Renderer {
public:
    Renderer() = default;
    ~Renderer() = default;

    bool initialize();
    void clear();
    void render();

};


#endif //DYNAMICSLAB_RENDERER_H
