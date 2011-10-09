/**
 * @file Object.h
 * @brief Class that describe Object, a subclass of Skeleton
 * @author A. Huaman
 * @date 2011-10-07
 */

#ifndef PLANNING_OBJECT_H
#define PLANNING_OBJECT_H

#include <vector>
#include <string>
#include "kinematics/Skeleton.h"

namespace planning {
#define MAX_Object_NAME 128


    class Object : public kinematics::Skeleton {
    public:

        std::string mName; ///< Name
        std::string mPathName; ///< PathName
        bool mMovable;
        
        Object();
        virtual ~Object();

        inline std::string getName() { return mName; }
        void loadModel( std::string _filename );
    };

} // namespace planning 

#endif // #ifndef PLANNING_OBJECT_H
