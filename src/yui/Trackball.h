/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#ifndef YUI_TRACKBALL
#define YUI_TRACKBALL

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace yui{

    class Trackball
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Trackball(): mCenter(0.0,0.0), mRadius(1.0), mStartPos(0.0,0.0,0.0),mCurrQuat(1.0,0.0,0.0,0.0) {}
        Trackball(const Eigen::Vector2d& center, double radius)
            : mCenter(center), mRadius(radius), mStartPos(0.0,0.0,0.0), mCurrQuat(1.0,0.0,0.0,0.0) {}

        // set the starting position to the project of (x,y) on the trackball
        void startBall(double x, double y);
        // update the current rotation to rotate from mStartPos to the projection of (x,y) on trackball, then update mStartPos
        void updateBall(double x, double y);

        // apply the current rotation to openGL env
        void applyGLRotation();
        // draw the trackball on screen
        void draw(int winWidth, int winHeight);

        void setTrackball(const Eigen::Vector2d& center, const double radius){ mCenter = center, mRadius = radius; }
        void setCenter(const Eigen::Vector2d& center){ mCenter = center; }
        void setRadius( const double radius ){ mRadius = radius; }
        void setQuaternion(const Eigen::Quaterniond& q){ mCurrQuat = q; }
        Eigen::Quaterniond getCurrQuat() const { return mCurrQuat; }
        Eigen::Matrix3d getRotationMatrix() const { return mCurrQuat.toRotationMatrix(); }
        Eigen::Vector2d getCenter() const { return mCenter; }
        double getRadius() const { return mRadius; }

    private:
        // project screen coordinate (x,y) to the trackball
        Eigen::Vector3d mouseOnSphere(double mouseX, double mouseY) const;
        // compute the quaternion that rotates from vector "from" to vector "to"
        Eigen::Quaterniond quatFromVectors(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

        Eigen::Vector2d mCenter;
        double mRadius;
        Eigen::Vector3d mStartPos;
        Eigen::Quaterniond mCurrQuat;
    };

}   // namespace yui

#endif
