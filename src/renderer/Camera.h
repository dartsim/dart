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
		virtual ~Camera() {};
        virtual void set (const Vector3d& Eye, const Vector3d& look, const Vector3d& up);
        virtual void slide (double delX, double delY, double delZ, bool bLocal = false);
        virtual void setFrustum (float vAng, float asp, float nearD, float farD);
        virtual void setOrtho(float width, float height, float nearD, float farD);


        virtual void roll (float angle);
        virtual void pitch (float angle);
        virtual void yaw (float angle);
        virtual void localRotate (float angle, AXIS axis);
        virtual void globalRotate (float angle, AXIS axis);

        virtual Vector3d getEye (void) const
		{
		}
        virtual Vector3d getLookAtDir(void) const
		{
		}
        virtual Vector3d getUpDir(void) const
		{

		}
        virtual bool isOrthogonal(void) const
        {
            return mIsOrthognal;
        }

        virtual float getVerticalViewAngle(void) const
        {
            return mViewAngle;
        }
        virtual float getNearDistance() const
        {
            return mNearDist;
        }
        virtual float getFarDistance() const
        {
            return mFarDist;
        }
        virtual float getWidth() const
        {
            return mWidth;
        }
        virtual float getHeight() const
        {
            return mHeight;
        }

    protected:
        float mViewAngle, mAspect, mNearDist, mFarDist, mWidth, mHeight;
        bool mIsOrthognal;
    };
}


#endif
