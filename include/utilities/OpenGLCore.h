#ifndef OPENGL_CORE_H
#define OPENGL_CORE_H

// Check if GLFW was already included, which would be an error
#ifdef _GLFW3_H
#error "GLFW was included before GLAD! Please include OpenGLCore.h before any GLFW headers."
#endif

// Include GLAD first to load OpenGL function pointers
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#endif // OPENGL_CORE_H
