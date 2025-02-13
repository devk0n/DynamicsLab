#include "ImGuiManager.h"

void ImGuiManager::showPhysicsControls(PhysicsEngine &physicsEngine) {
  ImGui::Begin("Physics Controls");

  auto &rigidBodies = physicsEngine.getRigidBodies();

  for (size_t i = 0; i < rigidBodies.size(); ++i) {
    auto &body = rigidBodies[i];

    std::string label = "RigidBody " + std::to_string(i);

    if (ImGui::CollapsingHeader(label.c_str())) {
      // Position controls
      Eigen::Vector3d position = body.getPosition();
      float pos[3] = {
        static_cast<float>(position.x()),
        static_cast<float>(position.y()),
        static_cast<float>(position.z())
      };

      if (ImGui::DragFloat3((label + " Position").c_str(), pos, 0.1f)) {
        body.setPosition(Eigen::Vector3d(pos[0], pos[1], pos[2]));
      }

      // Rotation controls (around local x, y, z axes)
      float rotationAngles[3] = {0.0f, 0.0f, 0.0f}; // Angles in radians
      if (ImGui::DragFloat3(
        (label + " Rotation (X, Y, Z)").c_str(),
        rotationAngles,
        0.01f,
        -3.14f,
        3.14f,
        "%.2f rad"
      )) {
        // Get the current orientation (w, x, y, z)
        Eigen::Vector4d currentOrientation = body.getOrientation();

        // Create quaternions for each axis rotation
        auto createQuaternion = [](double angle, const Eigen::Vector3d &axis) -> Eigen::Vector4d {
          double halfAngle = angle * 0.5;
          double sinHalfAngle = sin(halfAngle);
          double cosHalfAngle = cos(halfAngle);
          return Eigen::Vector4d(
            cosHalfAngle,
            axis.x() * sinHalfAngle,
            axis.y() * sinHalfAngle,
            axis.z() * sinHalfAngle
          );
        };

        // Quaternions for rotations around x, y, z axes
        Eigen::Vector4d rotX = createQuaternion(rotationAngles[0], Eigen::Vector3d::UnitX());
        Eigen::Vector4d rotY = createQuaternion(rotationAngles[1], Eigen::Vector3d::UnitY());
        Eigen::Vector4d rotZ = createQuaternion(rotationAngles[2], Eigen::Vector3d::UnitZ());

        // Combine the rotations (order matters: Z -> Y -> X)
        auto multiplyQuaternions = [](const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) -> Eigen::Vector4d {
          double w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
          double w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);
          return Eigen::Vector4d(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, // w
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, // x
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, // y
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2 // z
          );
        };

        // Apply rotations in Z -> Y -> X order
        Eigen::Vector4d newQuaternion = multiplyQuaternions(
          rotZ,
          multiplyQuaternions(
            rotY,
            multiplyQuaternions(
              rotX,
              currentOrientation
            )
          )
        );

        // Normalize the quaternion
        double norm = sqrt(newQuaternion(0) * newQuaternion(0) +
                           newQuaternion(1) * newQuaternion(1) +
                           newQuaternion(2) * newQuaternion(2) +
                           newQuaternion(3) * newQuaternion(3));
        newQuaternion /= norm;

        // Update the body's orientation
        body.setOrientation(newQuaternion);
      }
    }
  }
  ImGui::End();
}