#ifndef SRC_MODEL3D_PRIMITIVE_CUBE_H
#define SRC_MODEL3D_PRIMITIVE_CUBE_H

#include "Primitive.h"

namespace model3d {

    class PrimitiveCube : public Primitive {
    public:
        PrimitiveCube(Eigen::Vector3d _dim, double _mass);
	
        virtual void draw(Renderer::OpenGLRenderInterface* RI, const Eigen::Vector4d& _col, bool _default = true);
    private:
        void calMassTensor();
        void calVolume();
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_PRIMITIVE_CUBE_H

