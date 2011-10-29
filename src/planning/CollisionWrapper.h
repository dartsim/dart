#ifndef COLLISIONWRAPPER_H_
#define COLLISIONWRAPPER_H_

#include <fcl/BVH_model.h>
#include <fcl/vec_3f.h>
#include <Eigen/Eigen>
#include "Model3DS.h"

// Conversion of data strucutres from Model3DS to fcl

namespace CollisionWrapper {

void getModel_3DSToFCL(Model3DS _model3DS,
		std::vector<fcl::Triangle> *_triangles,
		std::vector<fcl::Vec3f> *_vertices);

fcl::Vec3f getTrans_EigenToFCL(Eigen::Transform<double, 3, Eigen::Affine> &_tf);

fcl::Vec3f getRotation_EigenToFCL(Eigen::Transform<double, 3, Eigen::Affine> &_tf, int col);

}

#endif /* COLLISIONWRAPPER_H_ */
