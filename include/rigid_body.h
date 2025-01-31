#ifndef DYNAMICSLAB_RIGID_BODY_H
#define DYNAMICSLAB_RIGID_BODY_H

#include <Eigen/Dense>
#include <memory>

using namespace Eigen;

class RigidBody {
public:
    explicit RigidBody(
            Vector3d position,
            Vector4d orientation,
            Matrix3d massMatrix,
            Matrix3d globalInertiaTensor);

    void normalizeOrientation();

    Vector3d getPosition();
    Vector4d getOrientation();

    Matrix3d getMassMatrix();
    Matrix3d getGlobalInertiaTensor();
    Matrix4d getInertiaTensor();

    Vector3d getVelocity();
    Vector4d getAngularVelocity();

    double getQuaternionNormSquared();

    Vector3d getInitialPosition();
    Vector4d getInitialOrientation();

    Vector3d getInitialVelocity();
    Vector4d getInitialAngularVelocity();

    void setPosition(Vector3d position);
    void setOrientation(Vector4d orientation);

    void setVelocity(Vector3d velocity);
    void setAngularVelocity(Vector4d angularVelocity);

private:
    Vector3d m_Position;
    Vector3d m_Velocity;

    Vector4d m_Orientation;
    Vector4d m_AngularVelocity;

    Matrix3d m_GlobalInertiaTensor;

    Matrix3d m_MassMatrix;
    Matrix4d m_InertiaTensor;

    Vector3d m_initialPosition;
    Vector3d m_initialVelocity;

    Vector4d m_initialOrientation;
    Vector4d m_initialAngularVelocity;

};


#endif //DYNAMICSLAB_RIGID_BODY_H
