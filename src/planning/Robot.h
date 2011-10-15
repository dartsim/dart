/**
 * @file Robot.h
 * @brief Class that describe Robot, a subclass of Skeleton
 * @author A. Huaman
 * @date 2011-10-07
 */

#ifndef PLANNING_ROBOT_H
#define PLANNING_ROBOT_H

#include <vector>
#include <string>
#include "kinematics/Skeleton.h"

namespace planning {
#define MAX_ROBOT_NAME 128

    class Robot : public kinematics::Skeleton {
    public:

        std::string mName; ///< Name
        std::string mPathName; ///< PathName
        int mGripID; /// THIS HAS TO BE REMOVED

        Robot();
        virtual ~Robot();

        inline std::string getName() { return mName; }
    };

} // namespace planning 

#endif // #ifndef PLANNING_ROBOT_H
