#ifndef KRWORLDNODE_H
#define KRWORLDNODE_H

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/osg/osg.hpp>

class WorldNode : public dart::gui::osg::WorldNode
{
private:

public:
    WorldNode(const dart::simulation::WorldPtr & world);
    virtual ~WorldNode();

    void customPreStep() override;
};

#endif /* KRWORLDNODE_H */
