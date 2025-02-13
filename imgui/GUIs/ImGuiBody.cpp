#include "../ImGuiManager.h"

void ImGuiManager::showBodyControls(PhysicsEngine &physicsEngine) {
    ImGui::Begin("Body Data and Control");

    auto &rigidBodies = physicsEngine.getRigidBodies();

    for (size_t i = 0; i < rigidBodies.size(); ++i) {
        auto &body = rigidBodies[i];

        std::string label = "RigidBody " + std::to_string(i);

        if (ImGui::CollapsingHeader(label.c_str())) {
            // Summary section
            Eigen::Vector3d position = body.getPosition();
            Eigen::Vector3d linearVelocity = body.getLinearVelocity();
            double mass = body.getMass();

            ImGui::Text("Position: (%.2f, %.2f, %.2f)", position.x(), position.y(), position.z());
            ImGui::Text("Linear Velocity: (%.2f, %.2f, %.2f)", linearVelocity.x(), linearVelocity.y(),
                        linearVelocity.z());
            ImGui::Text("Mass: %.2f kg", mass);

            // Organize controls into tabs
            if (ImGui::BeginTabBar("BodyProperties")) {
                // Position and Rotation Tab
                if (ImGui::BeginTabItem("Position/Rotation")) {
                    // Position controls
                    float pos[3] = {
                        static_cast<float>(position.x()),
                        static_cast<float>(position.y()),
                        static_cast<float>(position.z())
                    };
                    if (ImGui::DragFloat3((label + " Position").c_str(), pos, 0.1f)) {
                        body.setPosition(Eigen::Vector3d(pos[0], pos[1], pos[2]));
                    }
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Drag to change the position of the rigid body.");
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
                        auto multiplyQuaternions = [](const Eigen::Vector4d &q1,
                                                      const Eigen::Vector4d &q2) -> Eigen::Vector4d {
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
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Drag to rotate the rigid body around its local axes.");
                    }

                    ImGui::EndTabItem();
                }

                // Velocities Tab
                if (ImGui::BeginTabItem("Velocities")) {
                    Eigen::Vector3d linearVelocity = body.getLinearVelocity();
                    Eigen::Vector4d angularVelocity = body.getAngularVelocity();

                    float linVel[3] = {
                        static_cast<float>(linearVelocity.x()),
                        static_cast<float>(linearVelocity.y()),
                        static_cast<float>(linearVelocity.z())
                    };
                    float angVel[4] = {
                        static_cast<float>(angularVelocity.w()),
                        static_cast<float>(angularVelocity.x()),
                        static_cast<float>(angularVelocity.y()),
                        static_cast<float>(angularVelocity.z())
                    };

                    ImGui::DragFloat3("Linear Velocity", linVel, 0.1f, -100.0f, 100.0f, "%.2f m/s");
                    ImGui::DragFloat3("Angular Velocity", angVel, 0.1f, -10.0f, 10.0f, "%.2f rad/s");

                    ImGui::EndTabItem();
                }

                // Forces Tab
                if (ImGui::BeginTabItem("Forces")) {
                    Eigen::Vector3d force = body.getAppliedForce();
                    Eigen::Vector3d torque = body.getAppliedTorque();

                    float forceData[3] = {
                        static_cast<float>(force.x()), static_cast<float>(force.y()), static_cast<float>(force.z())
                    };
                    float torqueData[3] = {
                        static_cast<float>(torque.x()), static_cast<float>(torque.y()), static_cast<float>(torque.z())
                    };

                    ImGui::DragFloat3("Applied Force", forceData, 0.1f, -1000.0f, 1000.0f, "%.2f N");
                    ImGui::DragFloat3("Applied Torque", torqueData, 0.1f, -1000.0f, 1000.0f, "%.2f Nm");

                    ImGui::EndTabItem();
                }

                // Mass and Inertia Tab
                if (ImGui::BeginTabItem("Mass and Inertia")) {
                    double mass = body.getMass();
                    if (ImGui::InputDouble("Mass", &mass, 0.1, 1.0, "%.2f kg")) {
                        body.setMass(mass);
                    }

                    Eigen::Matrix3d inertia = body.getLocalInertiaTensor();
                    float inertiaData[3][3] = {
                        {
                            static_cast<float>(inertia(0, 0)), static_cast<float>(inertia(0, 1)),
                            static_cast<float>(inertia(0, 2))
                        },
                        {
                            static_cast<float>(inertia(1, 0)), static_cast<float>(inertia(1, 1)),
                            static_cast<float>(inertia(1, 2))
                        },
                        {
                            static_cast<float>(inertia(2, 0)), static_cast<float>(inertia(2, 1)),
                            static_cast<float>(inertia(2, 2))
                        }
                    };

                    if (ImGui::InputFloat3("Inertia Row 1", inertiaData[0], "%.2f")) {
                        body.setLocalInertiaTensor(Eigen::Matrix3d{
                            {inertiaData[0][0], inertiaData[0][1], inertiaData[0][2]},
                            {inertiaData[1][0], inertiaData[1][1], inertiaData[1][2]},
                            {inertiaData[2][0], inertiaData[2][1], inertiaData[2][2]}
                        });
                    }
                    ImGui::InputFloat3("Inertia Row 2", inertiaData[1], "%.2f");
                    ImGui::InputFloat3("Inertia Row 3", inertiaData[2], "%.2f");

                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
    }
    ImGui::End();
}
