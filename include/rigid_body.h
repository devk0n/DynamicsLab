#ifndef DYNAMICSLAB_RIGID_BODY_H
#define DYNAMICSLAB_RIGID_BODY_H

#include <Eigen/Dense>

class RigidBody {
public:
    explicit RigidBody(
            Eigen::Vector3d position,
            Eigen::Vector4d orientation,
            Eigen::Matrix3d massMatrix,
            Eigen::Matrix3d inertiaTensor);

    Eigen::Vector3d getPosition();
    Eigen::Vector4d getOrientation();
    Eigen::Matrix3d getMassMatrix();
    Eigen::Matrix3d getGlobalInertiaTensor();

private:
    double m_Mass;

    Eigen::Vector3d m_Position;
    Eigen::Vector3d m_Velocity;

    Eigen::Vector4d m_Orientation;
    Eigen::Vector4d m_AngularVelocity;

    Eigen::Matrix3d m_GlobalInertiaTensor;

    Eigen::Matrix3d m_MassMatrix;
    Eigen::Matrix4d m_InertiaTensor;

    Eigen::Matrix<double, 3, 4> m_LTransformationMatrix();
    Eigen::Matrix<double, 3, 4> m_GTransformationMatrix();


};


#endif //DYNAMICSLAB_RIGID_BODY_H
