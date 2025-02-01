#ifndef DYNAMICSLAB_RIGID_BODY_H
#define DYNAMICSLAB_RIGID_BODY_H

#include <Eigen/Dense>
#include <memory>

using namespace Eigen;

class RigidBody {
public:
    explicit RigidBody(double mass, Matrix3d globalInertiaTensor, Vector3d position, Vector4d orientation);

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

    void setInertiaTensor(Matrix4d inertiaTensor);

private:
    double m_mass;

    Vector3d m_position;
    Vector3d m_velocity;

    Vector4d m_orientation;
    Vector4d m_angularVelocity;

    Matrix3d m_globalInertiaTensor;

    Matrix3d m_massMatrix;
    Matrix4d m_inertiaTensor;

    Vector3d m_initialPosition;
    Vector3d m_initialVelocity;

    Vector4d m_initialOrientation;
    Vector4d m_initialAngularVelocity;
};


#endif //DYNAMICSLAB_RIGID_BODY_H
