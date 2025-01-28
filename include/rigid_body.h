#ifndef DYNAMICSLAB_RIGID_BODY_H
#define DYNAMICSLAB_RIGID_BODY_H

#include <Eigen/Dense>

class RigidBody {
public:
    RigidBody(double mass, const Eigen::Matrix3d& inertia, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation);

    Eigen::Vector3d getPosition() const;
    Eigen::Vector4d getOrientation() const;

    Eigen::Vector3d setPosition() const;
    Eigen::Vector4d setOrientation() const;

private:
    double m_Mass;
    Eigen::Matrix3d m_Inertia;
    Eigen::Vector3d m_Position;
    Eigen::Vector3d m_Velocity;
    Eigen::Vector4d m_Orientation;
    Eigen::Vector4d m_AngularVelocity;
};


#endif //DYNAMICSLAB_RIGID_BODY_H
