#ifndef DYNAMICSLAB_RIGIDBODY_H
#define DYNAMICSLAB_RIGIDBODY_H

#include <Eigen/Dense>

#include <utilities/MatrixUtilities.h>

using namespace Eigen;

class RigidBody {
public:
    RigidBody(Vector3d position, Vector4d orientation, Matrix3d massMatrix, Matrix3d localInertiaTensor);

    // Getters
    Vector3d getPosition();
    Vector3d getVelocity();
    Vector3d getAcceleration();

    Vector4d getOrientation();
    Vector4d getAngularVelocity();
    Vector4d getAngularAcceleration();

    Matrix3d getMassMatrix();
    Matrix4d getInertiaTensor();

    Matrix3d getLocalInertiaTensor();

    double getMass();

    // Setters
    void setPosition(Vector3d position);
    void setVelocity(Vector3d velocity);
    void setAcceleration(Vector3d acceleration);

    void setOrientation(Vector4d orientation);
    void setAngularVelocity(Vector4d angularVelocity);
    void setAngularAcceleration(Vector4d angularAcceleration);

    void setMassMatrix(Matrix3d massMatrix);
    void setInertiaTensor(Matrix4d inertiaTensor);

private:
    Vector3d m_position;                        // (r) is for position. Yeah, I know it should have been "r" for rotation, but Nikravesh's book uses this.
    Vector3d m_velocity;                        // (rd) velocity in xv, yv, zv.
    Vector3d m_acceleration;                    // (rdd) acceleration in xa, ya, za.

    Vector4d m_orientation;                     // (p) orientation in quaternions w, x, y, z.
    Vector4d m_angularVelocity;                 // (pd)
    Vector4d m_angularAcceleration;             // (pdd)

    Matrix4d m_inertiaTensor;                   // (J*) Generalized inertia tensor from quaternions.

    Matrix3d m_massMatrix;                      // (N) Mass matrix
    Matrix3d m_localInertiaTensor;              // (Jm) Local inertia tensor kgm^2.
};


#endif //DYNAMICSLAB_RIGIDBODY_H
