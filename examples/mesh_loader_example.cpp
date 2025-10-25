/*
 * Example demonstrating the new MeshLoader API
 *
 * This example shows how to use the new mesh loader abstraction
 * to load meshes without depending on assimp types in application code.
 */

#include <dart/utils/AssimpMeshLoader.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/common/LocalResourceRetriever.hpp>

#include <iostream>
#include <memory>

int main()
{
  // Example 1: Using the new AssimpMeshLoader directly
  {
    std::cout << "Example 1: Using AssimpMeshLoader\n";
    std::cout << "===================================\n";

    // Create a mesh loader
    auto loader = std::make_unique<dart::utils::AssimpMeshLoaderd>();

    // Create a resource retriever
    auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

    // Load a mesh (replace with actual mesh path)
    std::string meshPath = "path/to/mesh.dae";
    auto mesh = loader->load(meshPath, retriever);

    if (mesh) {
      std::cout << "Mesh loaded successfully!\n";
      std::cout << "  Vertices: " << mesh->getVertices().size() << "\n";
      std::cout << "  Triangles: " << mesh->getTriangles().size() << "\n";
    } else {
      std::cout << "Failed to load mesh\n";
    }

    std::cout << "\n";
  }

  // Example 2: Using the new MeshShape constructor with TriMesh
  {
    std::cout << "Example 2: Using MeshShape with TriMesh\n";
    std::cout << "========================================\n";

    // Load mesh using loader
    auto loader = std::make_unique<dart::utils::AssimpMeshLoaderd>();
    auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

    std::string meshPath = "path/to/mesh.dae";
    auto mesh = loader->load(meshPath, retriever);

    if (mesh) {
      // Create MeshShape using the new TriMesh-based constructor
      Eigen::Vector3d scale(1.0, 1.0, 1.0);
      auto meshShape = std::make_shared<dart::dynamics::MeshShape>(
          scale,
          std::move(mesh),
          dart::common::Uri(meshPath));

      std::cout << "MeshShape created with TriMesh!\n";
      std::cout << "  Scale: " << meshShape->getScale().transpose() << "\n";

      // Access the TriMesh
      auto triMesh = meshShape->getTriMesh();
      if (triMesh) {
        std::cout << "  Vertices: " << triMesh->getVertices().size() << "\n";
        std::cout << "  Triangles: " << triMesh->getTriangles().size() << "\n";
      }
    }

    std::cout << "\n";
  }

  // Example 3: Old API still works (deprecated but functional)
  {
    std::cout << "Example 3: Old API (deprecated but still works)\n";
    std::cout << "================================================\n";

    auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

    // Old way: loadMesh returns aiScene*
    const aiScene* scene = dart::dynamics::MeshShape::loadMesh(
        "path/to/mesh.dae", retriever);

    if (scene) {
      // Old way: MeshShape constructor with aiScene
      Eigen::Vector3d scale(1.0, 1.0, 1.0);
      auto meshShape = std::make_shared<dart::dynamics::MeshShape>(
          scale,
          scene,
          dart::common::Uri("path/to/mesh.dae"),
          retriever);

      std::cout << "MeshShape created with old API\n";

      // Both old and new API work
      const aiScene* oldAPI = meshShape->getMesh();  // deprecated
      auto newAPI = meshShape->getTriMesh();          // preferred

      std::cout << "  Old API (aiScene): "
                << (oldAPI ? "available" : "null") << "\n";
      std::cout << "  New API (TriMesh): "
                << (newAPI ? "available" : "null") << "\n";
    }

    std::cout << "\n";
  }

  std::cout << "Examples complete!\n";
  return 0;
}
