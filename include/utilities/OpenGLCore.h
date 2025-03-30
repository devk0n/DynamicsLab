#ifndef OPENGL_CORE_H
#define OPENGL_CORE_H

// Prevent incorrect include order
#ifdef _GLFW3_H
#error "GLFW was included before GLAD. Fix include order by including OpenGLCore.h first in your source files."
#endif

// GLAD must be included before GLFW
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#endif // OPENGL_CORE_H
