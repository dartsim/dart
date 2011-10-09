/**
 * @file World.h
 * @brief Class that contains robot + object descriptions
 * @author A. Huaman
 * @date 2011-10-07
 */

#ifndef PLANNING_WORLD_H
#define PLANNING_WORLD_H

#include <vector>
#include <stdio.h>
#include "Robot.h"
#include "Object.h"

namespace planning {

    class Robot;
    class Object;

    class World {
    public:

        World();
        virtual ~World();

        int addRobot( Robot *_robot );
        int addObject( Object *_object );
        void printInfo();


    protected:
        std::vector< Robot* > mRobots;
        std::vector< Object* > mObjects;

    };

} // namespace planning

#endif /** PLANNING_WORLD_H_ */

