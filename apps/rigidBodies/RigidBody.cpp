#include "RigidBody.h"
#include "renderer/RenderInterface.h"
#include <iostream>

using namespace Eigen;

void RigidBody::draw(renderer::RenderInterface* _ri) {
    if (!_ri)
        return;

    _ri->setPenColor(mColor);

    _ri->pushMatrix();
    _ri->translate(mPosition);    
    _ri->drawCube(mShape->getDim());
    _ri->popMatrix();
}
