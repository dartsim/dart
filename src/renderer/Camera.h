#ifndef CAMERA_H
#define CAMERA_H

namespace Renderer
{

#define DEFAULT_NEAR_PLANE 0.001
#define DEFAULT_FAR_PLANE 15

    enum AXIS
    {
        AXIS_X,
        AXIS_Y,
        AXIS_Z
    };

    class Camera
    {


    public:
        Camera() {}

        void set (const Vector3d& Eye, const Vector3d& look, const Vector3d& up);
        void slide (double delX, double delY, double delZ, bool bLocal = false);
        void setFrustum (float vAng, float asp, float nearD, float farD);
        void setOrtho(float width, float height, float nearD, float farD);


        void roll (float angle);
        void pitch (float angle);
        void yaw (float angle);
        void localRotate (float angle, AXIS axis);
        void globalRotate (float angle, AXIS axis);

        Vector3d getEye (void);
        Vector3d getLookAtDir(void);
        Vector3d getUpDir(void);


        bool isOrthogonal(void) const
        {
            return mIsOrthognal;
        }

        float getVerticalViewAngle(void) const
        {
            return mViewAngle;
        }
        float getNearDistance() const
        {
            return mNearDist;
        }
        float getFarDistance() const
        {
            return mFarDist;
        }
        float getWidth() const
        {
            return mWidth;
        }
        float getHeight() const
        {
            return mHeight;
        }

    private:
        float mViewAngle, mAspect, mNearDist, mFarDist, mWidth, mHeight;
        bool mIsOrthognal;
    };
}


#endif
