#include "Particle.h"
#include "dart/renderer/RenderInterface.h"

using namespace Eigen;

void Particle::draw(dart::renderer::RenderInterface* _ri) {
  if (!_ri)
    return;

  _ri->setPenColor(mColor);

  _ri->pushMatrix();
  _ri->translate(mPosition);
  _ri->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
  _ri->popMatrix();
}
