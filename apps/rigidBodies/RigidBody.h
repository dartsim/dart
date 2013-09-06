#ifndef _RIGIDBODY_
#define _RIGIDBODY_

#include <Eigen/Dense>
#include "kinematics/Shape.h"
#include "kinematics/ShapeBox.h"
#include "kinematics/ShapeEllipsoid.h"

namespace renderer {
    class RenderInterface;
}

class RigidBody {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RigidBody(kinematics::Shape::ShapeType _type, Eigen::Vector3d _dim) {
        // Create a default rigid body
        mMass = 1.0;
        mPosition.setZero();
        mOrientation.setIdentity();
        mColor << 0.9, 0.2, 0.2, 1.0; // Red
        if (_type == kinematics::Shape::P_BOX) {
            mShape = new kinematics::ShapeBox(_dim);
        } else if (_type == kinematics::Shape::P_ELLIPSOID) {
            mShape = new kinematics::ShapeEllipsoid(_dim);
        }
    }
    virtual ~RigidBody() {}

    void draw(renderer::RenderInterface* _ri);
    
    double mMass;
    Eigen::Vector3d mPosition;
    Eigen::Matrix3d mOrientation;
    kinematics::Shape* mShape;

    Eigen::Vector4d mColor;
};

#endif
