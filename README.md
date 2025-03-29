# DynamicsLab

DynamicsLab is a modern C++ application that demonstrates a modular multi-body dynamics simulation engine. It features a custom physics solver with various constraint types, integrated rendering with OpenGL, and an interactive UI powered by ImGui.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Features](#features)
- [Dependencies](#dependencies)
- [Build Instructions](#build-instructions)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

---

## Overview

DynamicsLab simulates rigid body dynamics using a range of constraints (ball joints, distance constraints, revolute joints, spherical joints, and universal joints) combined with an implicit integration scheme and constraint stabilization. The project is built with C++20 and leverages modern libraries for mathematics, graphics, and UI.

---

## Architecture

DynamicsLab is structured into several modular components:

- **Core System:**  
  - **Application:** Manages initialization, the main loop, and shutdown procedures.  
  - **Context:** Provides centralized access to window, input, rendering, scene, UI, and timing systems.

- **Rendering & Input:**  
  - **WindowManager & InputManager:** Create and manage the application window and handle user input.  
  - **Renderer & ShaderManager:** Manage OpenGL state, shader compilation, and drawing of simulation elements.  
  - **ImGuiManager:** Integrates ImGui for UI overlays and debugging information.

- **Scene Management:**  
  - **SceneManager & Primary Scene:** Organize simulation scenes and handle per-frame updates, camera controls, and UI interactions.

- **Physics Engine:**  
  - **Dynamics & Body:** Represent and simulate rigid bodies with mass, inertia, and state updates.  
  - **Constraints:** Implement various constraints (ball joint, distance, revolute, spherical, universal) for realistic motion.  
  - **Force Generators:** Apply forces such as gravity, spring forces, and torque to the bodies.

---

## Features

- **Multi-Body Dynamics Simulation:**  
  Solve rigid body dynamics with constraint projection and stabilization techniques.

- **Constraint Support:**  
  Implement various joint types (ball, distance, revolute, spherical, universal) using analytical Jacobians.

- **Modular and Extensible Design:**  
  Separate systems for rendering, input, physics, and UI for easy maintenance and extension.

- **Real-Time Rendering:**  
  Leverage OpenGL with GLFW, GLAD, and GLM for high-performance graphics.

- **Interactive UI:**  
  Integrated ImGui interface for runtime diagnostics and simulation control.

---

## Dependencies

- **Programming Language:** C++20  
- **Libraries:**
  - **Graphics:** OpenGL, GLFW, GLAD, GLM  
  - **Math:** Eigen  
  - **UI:** ImGui, ImPlot, ImGuizmo  
  - **Build System:** CMake (v3.30+)

Dependencies are fetched automatically via CMake’s FetchContent module.

---

## Build Instructions

1. **Clone the Repository:**

   ```bash
   git clone https://your.repo.url/DynamicsLab.git
   cd DynamicsLab
