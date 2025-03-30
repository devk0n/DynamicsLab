#ifndef PCH_H
#define PCH_H

// Avoid conflicts with Windows macros
#ifndef NOMINMAX
#define NOMINMAX
#endif

// Standard Library
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// Third-Party Libraries
// Eigen (Linear Algebra)
#include <Eigen/Core>
#include <Eigen/Dense>

// ImGui
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

// ImGuizmo (Gizmo Manipulation)
#include <ImGuizmo.h>

// ImPlot (Graph Plotting)
#include <implot.h>

// GLM (Math)
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#endif // PCH_H
