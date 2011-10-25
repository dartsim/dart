#ifndef COLLISIONWRAPPER_H_
#define COLLISIONWRAPPER_H_

#include <fcl/BVH_model.h>
#include <fcl/vec_3f.h>
#include <planning/Model3DS.h>

// Conversion of data strucutres from Model3DS to fcl

namespace CollisionWrapper {

void getModel_3DSToFCL(Model3DS _model3DS,
		std::vector<fcl::Triangle> *_triangles,
		std::vector<fcl::Vec3f> *_vertices) {
	int numVertices;
	double* vertices;
	int numIndices;
	short int* indicesTriangles;

	_model3DS.Get(&numVertices, vertices, &numIndices, indicesTriangles);

	for (int i = 0; i < numVertices; i += 3)
		_vertices->push_back(fcl::Vec3f(vertices[i], vertices[i + 1], vertices[i + 2]));

	for (int i = 0; i < numIndices; i += 3)
		_triangles->push_back(fcl::Triangle(indicesTriangles[i], indicesTriangles[i + 1],
						indicesTriangles[i + 2]));

}

fcl::Vec3f getTrans_EigenToFCL(Eigen::Transform<double, 3, Eigen::Affine> _tf) {
	return fcl::Vec3f(_tf.translation()(0), _tf.translation()(1), _tf.translation()(2));
}

fcl::Vec3f getRotation_EigenToFCL(Eigen::Transform<double, 3, Eigen::Affine> _tf, int col) {
	return fcl::Vec3f(_tf.rotation()(col, 0), _tf.rotation()(col, 1), _tf.rotation()(col, 2));
}

}

#endif /* COLLISIONWRAPPER_H_ */
