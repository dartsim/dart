/**
 * @file World.cpp
 */

#include <iostream>
#include "World.h"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"

using namespace std;
using namespace Eigen;

namespace planning {

    /**
     * @function World
     * @brief Constructor
     */
    World::World() {
        mRobots.resize(0); mObjects.resize(0);
    }

    /**
     * @function ~World
     * @brief Destructor
     */
    World::~World() {
        for( unsigned int i = 0; i < mRobots.size(); i++ ) delete mRobots[i];
        mRobots.clear();
        for( unsigned int i = 0; i < mObjects.size(); i++ ) delete mObjects[i];
        mObjects.clear();      
    }

    /**
     * @function addRobot
     */
    int World::addRobot( Robot *_robot ) {
        mRobots.push_back( _robot );
        return mRobots.size();
    }

    /**
     * @function addObject
     */
    int World::addObject( Object *_object ) {
        mObjects.push_back( _object );
        return mObjects.size();
    } 

    /**
     * @function printInfo
     */
    void World::printInfo() {

        /// Robots
        std::cout<<"-- World has " << mRobots.size() << " robots "<< std::endl;
        for( unsigned int i = 0; i < mRobots.size(); i++ )
        {    std::cout<<"* Robot["<<i<<"]: " << mRobots[i]->mName << " with "<< mRobots[i]->getNumNodes() << " nodes:" << std::endl;
             std::cout<<"  -- DOF: " << mRobots[i]->getNumDofs() << std::endl;         
        }

        /// Objects
        std::cout<<"-- World has " << mObjects.size() << " objects "<< std::endl;
        for( unsigned int i = 0; i < mObjects.size(); i++ )
        { std::cout<<"* Object["<<i<<"]: " << mObjects[i]->mName << std::endl; }

    }   

}  // namespace planning
