#include "worldnode.hpp"

WorldNode::WorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {
}

WorldNode::~WorldNode() {}

void WorldNode::customPreStep() {
}
