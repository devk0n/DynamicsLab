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
    Eigen::Vector3d getVelocity();
    Eigen::Vector4d getAngularVelocity();


    Eigen::Matrix<double, 3, 4> getLTransformationMatrix(Eigen::Vector4d transformationMatrix);
    Eigen::Matrix<double, 3, 4> getGTransformationMatrix(Eigen::Vector4d transformationMatrix);

private:
    Eigen::Vector3d m_Position;
    Eigen::Vector3d m_Velocity;

    Eigen::Vector4d m_Orientation;
    Eigen::Vector4d m_AngularVelocity;

    Eigen::Matrix3d m_GlobalInertiaTensor;

    Eigen::Matrix3d m_MassMatrix;
    Eigen::Matrix4d m_InertiaTensor;


};


#endif //DYNAMICSLAB_RIGID_BODY_H
