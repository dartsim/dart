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
        int mGripID; /// THIS HAS TO BE REMOVED
        bool mMovable;
        
        Object();
        virtual ~Object();
        inline std::string getName() { return mName; }

        void setPositionX( double _pos );
        void getPositionX( double &_pos );

        void setPositionY( double _pos );
        void getPositionY( double &_pos );

        void setPositionZ( double _pos );
        void getPositionZ( double &_pos );

        void setRotationRPY( double _roll, double _pitch, double _yaw );
        void getRotationRPY( double &_roll, double &_pitch, double &_yaw );


    };

} // namespace planning 

#endif // #ifndef PLANNING_OBJECT_H
