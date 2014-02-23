#include "RigidBody.h"
#include "renderer/RenderInterface.h"
#include <iostream>

using namespace Eigen;

void RigidBody::draw(dart::renderer::RenderInterface* _ri) {
    if (!_ri)
        return;

    _ri->setPenColor(mColor);

    _ri->pushMatrix();
	Eigen::Isometry3d m;
    m.translation() = mPosition;
    m.linear() = mOrientation;
    _ri->transform(m);
	if (mShape->getShapeType() == dart::dynamics::Shape::BOX)
        _ri->drawCube(mShape->getDim());
	else if (mShape->getShapeType() == dart::dynamics::Shape::ELLIPSOID)
        _ri->drawEllipsoid(mShape->getDim());
    else
        std::cout << "Unknown shape" << std::endl;
    _ri->popMatrix();
}
