![Alt text](./assets/images/DynamicsLab.png)

DynamicsLab is a modern C++ simulation engine for multi-body dynamics. It features a custom physics solver supporting a variety of constraints that enforce realistic mechanical connections between rigid bodies.

---

## Table of Contents

- [Overview](#overview)
- [Visual Media](#visual-media)
- [Architecture](#architecture)
- [Supported Constraints](#supported-constraints)
- [License](#license)
- [Contact](#contact)

---

## Overview

DynamicsLab simulates rigid body dynamics by solving the equations of motion with constraints. The engine is designed with modular components for physics, rendering, and user interaction, making it easy to extend with additional constraint types or force generators.

---

## Visual Media

### Triple Pendulum
![Demo](assets/files/TriplePendulum.gif)

### Carpet Stress Test
![Demo](assets/files/carpetDemo.gif)

### Movement Demo
![Demo](assets/files/movementDemo.gif)


---

## Architecture

DynamicsLab is organized into several key modules:

- **Core System:**  
  - *Application & Context:* Initialize and manage the overall simulation loop, providing shared access to subsystems.
  
- **Rendering & Input:**  
  - *WindowManager & InputManager:* Handle window creation and user input via GLFW.  
  - *Renderer & ShaderManager:* Manage OpenGL rendering, including shader compilation and drawing routines.
  - *ImGuiManager:* Provide an interactive UI overlay for debugging and simulation control.

- **Physics Engine:**  
  - *Dynamics & Body:* Represent physical bodies with mass, inertia, and state variables.
  - *Constraints & Force Generators:* Implement a variety of mechanical constraints and external force applications, ensuring realistic motion and joint behavior.

---

## Contributing

Contributions are welcome. To contribute:
- Fork the repository and create a feature branch.
- Follow the existing code conventions and document your changes.
- Submit a pull request with a clear description of your changes or additions.

---

## Supported Constraints

DynamicsLab currently supports the following constraints:

### Ball Joint (Spherical Joint)
- **Description:**  
  Allows rotation about any axis while constraining the connected bodies to share a common anchor point.
- **Usage:**  
  Ideal for simulating joints like shoulder or hip joints where only translational separation is restricted.
- **Implementation Highlights:**  
  Uses analytical Jacobians to enforce the position constraint between two bodies and calculates acceleration corrections to counteract drift.

### Distance Constraint
- **Description:**  
  Maintains a fixed distance between two points on separate bodies.
- **Usage:**  
  Useful for simulating objects connected by rigid rods or springs (when used with damping).
- **Implementation Highlights:**  
  The constraint is expressed as a quadratic equation, and its Jacobian is computed to relate changes in body positions to changes in distance.

### Revolute Joint
- **Description:**  
  Constrains two bodies to rotate relative to each other about a single, fixed axis.
- **Usage:**  
  Commonly used for hinge mechanisms such as doors or robotic arm joints.
- **Implementation Highlights:**  
  In addition to enforcing the coincidence of anchor points, extra constraints align the rotation axes. The Jacobian is partitioned into translational and rotational components.

### Spherical Joint (Alternative Formulation)
- **Description:**  
  Similar to the ball joint, this constraint ensures that two bodies remain connected at a common point while allowing full rotational freedom.
- **Usage:**  
  Used when a fixed distance is required between connection points, as in certain joint designs or closed-loop mechanisms.
- **Implementation Highlights:**  
  Incorporates both the position error and its derivative into the constraint correction, maintaining the proper separation even under dynamic conditions.

### Universal Joint
- **Description:**  
  Allows rotation about two perpendicular axes while constraining translation between connected bodies.
- **Usage:**  
  Suitable for applications like drive shafts or gimbal systems where complex rotational behavior is needed.
- **Implementation Highlights:**  
  The constraint enforces both the spherical connection and the alignment of two rotational axes through an extended Jacobian formulation.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

For questions, feedback, or further information, please open an issue [GitHub issues](https://github.com/devk0n/DynamicsLab/issues).

## Support DynamicsLab

If you find DynamicsLab useful or interesting, consider buying me a coffee—or helping keep the project going—with a small Bitcoin donation! Your support means a lot and helps me keep improving this project for everyone.

**BTC Address:**  
`bc1q7tj9hmr703kq63hdjxz8vu4meqavduwp8suxts`

Or scan the QR code:

![Bitcoin QR Code](https://www.bitcoinqrcodemaker.com/api/?style=bitcoin&address=bc1q7tj9hmr703kq63hdjxz8vu4meqavduwp8suxts)

Thank you for supporting open-source software! 🙏
