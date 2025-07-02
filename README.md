

# DynamicsLab

**DynamicsLab** is a modern C++ simulation engine for multi-body dynamics. It features a custom physics solver supporting a variety of constraints for realistic mechanical connections between rigid bodies.

---

## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [Camera & Controls](#camera--controls)
- [Visual Media](#visual-media)
- [Architecture](#architecture)
- [Supported Constraints](#supported-constraints)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

---

## Overview

DynamicsLab simulates rigid body dynamics by solving the equations of motion with constraints. The engine is modular and extensible—making it easy to add new constraint types or force generators.

---

## Getting Started

Clone the repository and build with CMake:

```sh
git clone https://github.com/devk0n/DynamicsLab.git
cd DynamicsLab
mkdir build && cd build
cmake ..
make
./DynamicsLab
```

---

## Camera & Controls

Navigate and interact with the simulation using these controls:

| Action           | Key/Mouse            |
| ---------------- | -------------------- |
| Move Forward     | **W**                |
| Move Backward    | **S**                |
| Move Left        | **A**                |
| Move Right       | **D**                |
| Move Up          | **Left Shift**       |
| Move Down        | **Left Control**     |
| Look Around      | **Hold Right Mouse** |
| Zoom             | **Mouse Scroll**     |
| Pause/Resume Sim | **Space**            |
| Look at Body 0   | **Hold L**           |

---

## Visual Media



- **Screenshots & Diagrams**: The repository includes screenshots and visualizations of constraints in action (see `assets/images`).
- **GIFs**: Short animations demonstrate constraint behavior, joint limits, and projection stabilization.

---

## Architecture

- **Core System**:
  - *Application & Context*: Manages simulation loop and subsystems.
- **Rendering & Input**:
  - *WindowManager, InputManager, Renderer, ShaderManager, ImGuiManager*: Handles windowing (GLFW), input, OpenGL rendering, and UI overlay.
- **Physics Engine**:
  - *Dynamics, Body, Constraints, Force Generators*: Implements multi-body dynamics and a variety of constraints.

---

## Supported Constraints

- **Ball Joint (Spherical Joint)**\
  Allows full rotation while enforcing a common anchor point.

- **Distance Constraint**\
  Maintains a fixed distance between two points on separate bodies.

- **Revolute Joint**\
  Allows rotation about a single axis—perfect for hinges.

- **Universal Joint**\
  Enables rotation about two perpendicular axes (e.g., drive shafts).

---

## Contributing

Pull requests and issues are welcome!

- Fork the repository, create a feature branch, and submit a PR with a clear description.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

Questions or feedback? [Open an issue](https://github.com/devk0n/DynamicsLab/issues).

