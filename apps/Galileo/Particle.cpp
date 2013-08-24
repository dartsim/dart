#include "Particle.h"
#include "yui/GLFuncs.h"

void Particle::draw(renderer::RenderInterface* _ri) const
{
    if (!_ri)
        return;

    _ri->setPenColor(mColor);

    _ri->pushMatrix();
    _ri->translate(mPosition);
    _ri->drawSphere();
    _ri->popMatrix();
}
