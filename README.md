# DynamicsLab

**DynamicsLab** is a real-time simulation tool for multibody dynamics systems. It is designed to help users visualize, analyze, and interact with complex system data efficiently. By combining powerful simulation capabilities with an intuitive interface, **DynamicsLab** supports researchers and engineers in better understanding dynamic systems.

We are solving the system:

$$
\begin{bmatrix}
M^* & P^T \\
P & 0
\end{bmatrix}
\begin{bmatrix}
\ddot{q} \\
\sigma
\end{bmatrix}
\+\
\begin{bmatrix}
b^* \\
c
\end{bmatrix}
\=\
\begin{bmatrix}
g^* \\
0
\end{bmatrix}
$$


## Interface
![Alt text](./assets/images/screenshot_20250130_185815.png)

## Features
- **Real-time Simulation**: Simulates rigid body dynamics with position, velocity, and acceleration updates.
- **Interactive Camera**: Provides intuitive camera controls for navigation and inspection of the simulation environment.
- **Modular Project Structure**: Designed for extensibility, making it easier to integrate new features or modify existing functionality.
- **User Interface**: Built using Dear ImGui, providing real-time control and visualization of simulation and camera data.
- **Cross-platform Support**: Based on GLFW and OpenGL, the project is compatible with major OS platforms.

| ![Screenshot 1](./assets/images/screenshot_20250130_104201.png) | ![Screenshot 2](./assets/images/screenshot_20250130_104001.png) | ![Screenshot 3](./assets/images/screenshot_20250130_094453.png) |
|:---------------------------------:|:---------------------------------:|:---------------------------------:|
| [View Full](./assets/images/screenshot_20250130_104201.png) | [View Full](./assets/images/screenshot_20250130_104001.png) | [View Full](./assets/images/screenshot_20250130_094453.png) |


## Camera Movement
The application enables camera navigation to explore the simulation from any viewpoint. Users can control the camera position and orientation via:
- **Mouse Input**:
    - Movement: Adjusts the camera's rotation.
    - Scrolling: Typically configured for zoom functionality.

- **Keyboard Input**:
    - W/A/S/D keys for navigation along the axes.
    - Space/Shift for upward/downward movement.

## Project Tree
```bash
DynamicsLab
├───assets
│   ├───images
│   └───shaders
│       ├───grid.frag.glsl
│       └───grid.vert.glsl
├───include
│   ├───glad
│   ├───KHR
│   ├───application.h
│   ├───dynamics.h
│   ├───imgui_layer.h
│   ├───renderer.h
│   └───rigid_body.h
└───src
    ├───core
    │   ├───main.cpp
    │   └───application.cpp
    ├───graphics
    │   ├───glad.c
    │   └───renderer.cpp
    ├───simulation
    │   ├───rigid_body.cpp
    │   └───dynamics.cpp
    └───ui
        └───imgui_layer.cpp
```

# Variable Mapping Table

| **Variable from Book**     | **Description**                                   | **Corresponding Name in Code**      | **Additional Notes**                       |
|:---------------------------:|:-----------------------------------------------:|:------------------------------------:|:------------------------------------------:|
| **M\***                    | System mass-inertia matrix                        | `SystemMassInertiaMatrix`           | Represents the mass and inertia properties of the system. |
| **P**                      | Quaternion constraint matrix                      | `QuaternionConstraintMatrix`        | Handles constraints related to quaternion representation. |
| **q**                      | Generalized position coordinates of the system    | `GeneralizedCoordinates`            | Position variables describing the configuration of the system. |
| **q̇** (q dot)             | Generalized velocity coordinates                  | `GeneralizedVelocities`             | Velocity variables associated with the system coordinates. |
| **q̈** (q double dot)       | Generalized accelerations                         | `GeneralizedAccelerations`          | Acceleration variables for the system's motion. |
| **b\***                    | Velocity-dependent term                          | `VelocityDependentTerm`             | Represents effects dependent on the velocity, such as Coriolis or damping forces. |
| **c**                      | Quaternion norm squared value                     | `QuaternionNormSquared`             | Constraint ensuring quaternions remain normalized. |
| **g\***                    | Generalized external forces                       | `GeneralizedExternalForces`         | External forces acting on the system. |
## References

1. Nikravesh, Parviz E. *Computer-Aided Analysis of Mechanical Systems*. Prentice-Hall, Inc., USA, 1988. ISBN: 0131642200.

2. GLFW - A multi-platform library for OpenGL, OpenGL ES, and Vulkan development.
   - Website: [https://www.glfw.org](https://www.glfw.org)

3. GLM - OpenGL Mathematics, a header-only C++ library for graphics software.
   - GitHub: [https://github.com/g-truc/glm](https://github.com/g-truc/glm)

4. Dear ImGui - A bloat-free graphical user interface library for C++.
   - GitHub: [https://github.com/ocornut/imgui](https://github.com/ocornut/imgui)

5. Eigen - A C++ template library for linear algebra.
   - Website: [https://eigen.tuxfamily.org](https://eigen.tuxfamily.org)
