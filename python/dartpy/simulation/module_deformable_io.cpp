/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "simulation/module_detail.hpp"

namespace dart::python_nb {

// deformable material/mesh/body types and io (skeleton/scene) loaders
void defSimPartDeformableIo(nb::module_& m)
{
  nb::class_<sim::DeformableMaterialProperties>(
      m, "DeformableMaterialProperties")
      .def(nb::init<>())
      .def_rw("density", &sim::DeformableMaterialProperties::density)
      .def_rw(
          "youngs_modulus", &sim::DeformableMaterialProperties::youngsModulus)
      .def_rw("poisson_ratio", &sim::DeformableMaterialProperties::poissonRatio)
      .def_rw(
          "friction_coefficient",
          &sim::DeformableMaterialProperties::frictionCoefficient)
      .def_rw(
          "use_finite_element_elasticity",
          &sim::DeformableMaterialProperties::useFiniteElementElasticity)
      .def_rw(
          "use_fixed_corotational_elasticity",
          &sim::DeformableMaterialProperties::useFixedCorotationalElasticity)
      .def_rw(
          "use_adaptive_barrier_stiffness",
          &sim::DeformableMaterialProperties::useAdaptiveBarrierStiffness)
      .def_rw(
          "use_iterative_linear_solver",
          &sim::DeformableMaterialProperties::useIterativeLinearSolver)
      .def_rw(
          "use_matrix_free_linear_solver",
          &sim::DeformableMaterialProperties::useMatrixFreeLinearSolver);

  nb::class_<sim::DeformableEdge>(m, "DeformableEdge")
      .def(
          "__init__",
          [](sim::DeformableEdge* self,
             std::size_t nodeA,
             std::size_t nodeB,
             double restLength) {
            new (self) sim::DeformableEdge{nodeA, nodeB, restLength};
          },
          nb::arg("node_a") = 0,
          nb::arg("node_b") = 0,
          nb::arg("rest_length") = -1.0)
      .def_rw("node_a", &sim::DeformableEdge::nodeA)
      .def_rw("node_b", &sim::DeformableEdge::nodeB)
      .def_rw("rest_length", &sim::DeformableEdge::restLength);

  nb::class_<sim::DeformableSurfaceTriangle>(m, "DeformableSurfaceTriangle")
      .def(
          "__init__",
          [](sim::DeformableSurfaceTriangle* self,
             std::size_t nodeA,
             std::size_t nodeB,
             std::size_t nodeC) {
            new (self) sim::DeformableSurfaceTriangle{nodeA, nodeB, nodeC};
          },
          nb::arg("node_a") = 0,
          nb::arg("node_b") = 0,
          nb::arg("node_c") = 0)
      .def_rw("node_a", &sim::DeformableSurfaceTriangle::nodeA)
      .def_rw("node_b", &sim::DeformableSurfaceTriangle::nodeB)
      .def_rw("node_c", &sim::DeformableSurfaceTriangle::nodeC);

  nb::class_<sim::DeformableTetrahedron>(m, "DeformableTetrahedron")
      .def(
          "__init__",
          [](sim::DeformableTetrahedron* self,
             std::size_t nodeA,
             std::size_t nodeB,
             std::size_t nodeC,
             std::size_t nodeD) {
            new (self) sim::DeformableTetrahedron{nodeA, nodeB, nodeC, nodeD};
          },
          nb::arg("node_a") = 0,
          nb::arg("node_b") = 0,
          nb::arg("node_c") = 0,
          nb::arg("node_d") = 0)
      .def_rw("node_a", &sim::DeformableTetrahedron::nodeA)
      .def_rw("node_b", &sim::DeformableTetrahedron::nodeB)
      .def_rw("node_c", &sim::DeformableTetrahedron::nodeC)
      .def_rw("node_d", &sim::DeformableTetrahedron::nodeD);

  nb::class_<sim::DeformableDirichletBoundaryCondition>(
      m, "DeformableDirichletBoundaryCondition")
      .def(nb::init<>())
      .def_rw("nodes", &sim::DeformableDirichletBoundaryCondition::nodes)
      .def_rw(
          "linear_velocity",
          &sim::DeformableDirichletBoundaryCondition::linearVelocity)
      .def_rw(
          "angular_velocity",
          &sim::DeformableDirichletBoundaryCondition::angularVelocity)
      .def_rw("center", &sim::DeformableDirichletBoundaryCondition::center)
      .def_rw(
          "start_time", &sim::DeformableDirichletBoundaryCondition::startTime)
      .def_rw("end_time", &sim::DeformableDirichletBoundaryCondition::endTime);

  nb::class_<sim::DeformableNeumannBoundaryCondition>(
      m, "DeformableNeumannBoundaryCondition")
      .def(nb::init<>())
      .def_rw("nodes", &sim::DeformableNeumannBoundaryCondition::nodes)
      .def_rw(
          "acceleration",
          &sim::DeformableNeumannBoundaryCondition::acceleration)
      .def_rw("start_time", &sim::DeformableNeumannBoundaryCondition::startTime)
      .def_rw("end_time", &sim::DeformableNeumannBoundaryCondition::endTime);

  nb::class_<sim::DeformableBodyOptions>(m, "DeformableBodyOptions")
      .def(nb::init<>())
      .def_rw("positions", &sim::DeformableBodyOptions::positions)
      .def_rw("velocities", &sim::DeformableBodyOptions::velocities)
      .def_rw("masses", &sim::DeformableBodyOptions::masses)
      .def_rw("edges", &sim::DeformableBodyOptions::edges)
      .def_rw(
          "surface_triangles", &sim::DeformableBodyOptions::surfaceTriangles)
      .def_rw("tetrahedra", &sim::DeformableBodyOptions::tetrahedra)
      .def_rw(
          "dirichlet_boundary_conditions",
          &sim::DeformableBodyOptions::dirichletBoundaryConditions)
      .def_rw(
          "neumann_boundary_conditions",
          &sim::DeformableBodyOptions::neumannBoundaryConditions)
      .def_rw("fixed_nodes", &sim::DeformableBodyOptions::fixedNodes)
      .def_rw("edge_stiffness", &sim::DeformableBodyOptions::edgeStiffness)
      .def_rw("damping", &sim::DeformableBodyOptions::damping)
      .def_rw("material", &sim::DeformableBodyOptions::material);

  nb::class_<sim::DeformableSolverOptions>(m, "DeformableSolverOptions")
      .def(nb::init<>())
      .def_rw("iterations", &sim::DeformableSolverOptions::iterations)
      .def_rw(
          "convergence_tolerance",
          &sim::DeformableSolverOptions::convergenceTolerance)
      .def_rw(
          "use_acceleration", &sim::DeformableSolverOptions::useAcceleration)
      .def_rw(
          "acceleration_spectral_radius",
          &sim::DeformableSolverOptions::accelerationSpectralRadius)
      .def_rw(
          "stiffness_damping", &sim::DeformableSolverOptions::stiffnessDamping)
      .def_rw(
          "ground_contact_stiffness",
          &sim::DeformableSolverOptions::groundContactStiffness);

  nb::class_<sim::DeformableBody>(m, "DeformableBody")
      .def_prop_ro("is_valid", &sim::DeformableBody::isValid)
      .def_prop_ro("name", &sim::DeformableBody::getName)
      .def_prop_ro("node_count", &sim::DeformableBody::getNodeCount)
      .def_prop_ro("edge_count", &sim::DeformableBody::getEdgeCount)
      .def_prop_ro(
          "surface_triangle_count",
          &sim::DeformableBody::getSurfaceTriangleCount)
      .def_prop_ro(
          "tetrahedron_count", &sim::DeformableBody::getTetrahedronCount)
      .def("node_position", &sim::DeformableBody::getPosition, nb::arg("node"))
      .def(
          "set_node_position",
          &sim::DeformableBody::setPosition,
          nb::arg("node"),
          nb::arg("position"))
      .def("node_velocity", &sim::DeformableBody::getVelocity, nb::arg("node"))
      .def(
          "set_node_velocity",
          &sim::DeformableBody::setVelocity,
          nb::arg("node"),
          nb::arg("velocity"))
      .def("node_mass", &sim::DeformableBody::getMass, nb::arg("node"))
      .def("is_fixed_node", &sim::DeformableBody::isFixedNode, nb::arg("node"))
      .def("edge", &sim::DeformableBody::getEdge, nb::arg("edge"))
      .def(
          "surface_triangle",
          &sim::DeformableBody::getSurfaceTriangle,
          nb::arg("triangle"))
      .def(
          "tetrahedron",
          &sim::DeformableBody::getTetrahedron,
          nb::arg("tetrahedron"))
      .def(
          "tetrahedron_rest_volume",
          &sim::DeformableBody::getTetrahedronRestVolume,
          nb::arg("tetrahedron"))
      .def_prop_ro(
          "material_properties", &sim::DeformableBody::getMaterialProperties);

  nb::class_<sim::io::SkeletonLoadOptions>(m, "SkeletonLoadOptions")
      .def(nb::init<>())
      .def_rw(
          "root_anchor_prefix",
          &sim::io::SkeletonLoadOptions::rootAnchorPrefix);

  nb::class_<sim::io::DeformableSceneLoadOptions>(
      m, "DeformableSceneLoadOptions")
      .def(nb::init<>())
      .def_rw("asset_root", &sim::io::DeformableSceneLoadOptions::assetRoot)
      .def_rw(
          "body_name_prefix",
          &sim::io::DeformableSceneLoadOptions::bodyNamePrefix)
      .def_rw(
          "add_structural_springs",
          &sim::io::DeformableSceneLoadOptions::addStructuralSprings)
      .def_rw(
          "structural_spring_stiffness",
          &sim::io::DeformableSceneLoadOptions::structuralSpringStiffness)
      .def_rw("damping", &sim::io::DeformableSceneLoadOptions::damping)
      .def_rw(
          "ignore_contact_directives",
          &sim::io::DeformableSceneLoadOptions::ignoreContactDirectives);

  nb::class_<sim::io::DeformableSceneBodyInfo>(m, "DeformableSceneBodyInfo")
      .def_ro("name", &sim::io::DeformableSceneBodyInfo::name)
      .def_ro("body", &sim::io::DeformableSceneBodyInfo::body)
      .def_ro("node_count", &sim::io::DeformableSceneBodyInfo::nodeCount)
      .def_ro(
          "tetrahedron_count",
          &sim::io::DeformableSceneBodyInfo::tetrahedronCount)
      .def_ro(
          "surface_triangle_count",
          &sim::io::DeformableSceneBodyInfo::surfaceTriangleCount)
      .def_ro(
          "dirichlet_condition_count",
          &sim::io::DeformableSceneBodyInfo::dirichletConditionCount)
      .def_ro(
          "neumann_condition_count",
          &sim::io::DeformableSceneBodyInfo::neumannConditionCount);

  nb::class_<sim::io::DeformableSceneInfo>(m, "DeformableSceneInfo")
      .def_ro("duration", &sim::io::DeformableSceneInfo::duration)
      .def_ro("time_step", &sim::io::DeformableSceneInfo::timeStep)
      .def_ro("gravity_enabled", &sim::io::DeformableSceneInfo::gravityEnabled)
      .def_ro("bodies", &sim::io::DeformableSceneInfo::bodies)
      .def_ro("warnings", &sim::io::DeformableSceneInfo::warnings);

  nb::class_<sim::io::DeformableSceneDiagnostics>(
      m, "DeformableSceneDiagnostics")
      .def_ro("frame", &sim::io::DeformableSceneDiagnostics::frame)
      .def_ro("time", &sim::io::DeformableSceneDiagnostics::time)
      .def_ro("body_count", &sim::io::DeformableSceneDiagnostics::bodyCount)
      .def_ro("node_count", &sim::io::DeformableSceneDiagnostics::nodeCount)
      .def_ro(
          "tetrahedron_count",
          &sim::io::DeformableSceneDiagnostics::tetrahedronCount)
      .def_ro(
          "surface_triangle_count",
          &sim::io::DeformableSceneDiagnostics::surfaceTriangleCount)
      .def_ro(
          "dirichlet_condition_count",
          &sim::io::DeformableSceneDiagnostics::dirichletConditionCount)
      .def_ro(
          "neumann_condition_count",
          &sim::io::DeformableSceneDiagnostics::neumannConditionCount)
      .def_ro("total_mass", &sim::io::DeformableSceneDiagnostics::totalMass)
      .def_ro(
          "max_displacement",
          &sim::io::DeformableSceneDiagnostics::maxDisplacement)
      .def_ro("min_z", &sim::io::DeformableSceneDiagnostics::minZ)
      .def_ro("max_z", &sim::io::DeformableSceneDiagnostics::maxZ);

  nb::class_<sim::io::SkeletonToMultibodyOptions>(
      m, "SkeletonToMultibodyOptions")
      .def(nb::init<>())
      .def_rw("name", &sim::io::SkeletonToMultibodyOptions::name)
      .def_rw(
          "base_link_name", &sim::io::SkeletonToMultibodyOptions::baseLinkName)
      .def_rw("copy_state", &sim::io::SkeletonToMultibodyOptions::copyState)
      .def_rw(
          "copy_joint_properties",
          &sim::io::SkeletonToMultibodyOptions::copyJointProperties)
      .def_rw(
          "load_collision_shapes",
          &sim::io::SkeletonToMultibodyOptions::loadCollisionShapes);

  m.def(
      "build_multibody_from_skeleton",
      [](sim::World& world,
         const dart::dynamics::Skeleton& skeleton,
         const sim::io::SkeletonToMultibodyOptions& options) {
        return sim::io::buildMultibodyFromSkeleton(world, skeleton, options);
      },
      nb::arg("world"),
      nb::arg("skeleton"),
      nb::arg("options") = sim::io::SkeletonToMultibodyOptions{},
      // The returned Multibody handle holds a raw World*, so keep the World
      // alive as long as the handle lives, matching World.add_multibody.
      nb::keep_alive<0, 1>());

  m.def(
      "add_skeleton",
      [](sim::World& world,
         const std::shared_ptr<dart::dynamics::Skeleton>& skeleton,
         const sim::io::SkeletonLoadOptions& options) {
        DART_SIMULATION_THROW_T_IF(
            !skeleton,
            sim::InvalidArgumentException,
            "Skeleton must not be null");
        return sim::io::addSkeleton(world, *skeleton, options);
      },
      nb::arg("world"),
      nb::arg("skeleton"),
      nb::arg("options") = sim::io::SkeletonLoadOptions{},
      nb::keep_alive<0, 1>());

  m.def(
      "add_skeleton",
      [](sim::World& world,
         const std::string& uri,
         const sim::io::SkeletonLoadOptions& options) {
        return sim::io::addSkeleton(world, dart::common::Uri(uri), options);
      },
      nb::arg("world"),
      nb::arg("uri"),
      nb::arg("options") = sim::io::SkeletonLoadOptions{},
      nb::keep_alive<0, 1>());

  m.def(
      "add_skeleton",
      [](sim::World& world,
         const std::string& uri,
         const dart::io::ReadOptions& readOptions,
         const sim::io::SkeletonLoadOptions& options) {
        return sim::io::addSkeleton(
            world, dart::common::Uri(uri), readOptions, options);
      },
      nb::arg("world"),
      nb::arg("uri"),
      nb::arg("read_options"),
      nb::arg("options") = sim::io::SkeletonLoadOptions{},
      nb::keep_alive<0, 1>());

  m.def(
      "load_deformable_scene",
      [](sim::World& world,
         const std::filesystem::path& scenePath,
         const sim::io::DeformableSceneLoadOptions& options) {
        return sim::io::loadDeformableScene(world, scenePath, options);
      },
      nb::arg("world"),
      nb::arg("scene_path"),
      nb::arg("options") = sim::io::DeformableSceneLoadOptions{},
      // The returned DeformableSceneInfo carries DeformableBody handles that
      // hold a raw World*, so keep the World alive as long as the info (and the
      // body handles read from it) lives, matching the keep-alive edge the
      // World.add_deformable_body / get_deformable_body bindings use.
      nb::keep_alive<0, 1>());

  m.def(
      "collect_deformable_scene_diagnostics",
      [](const sim::World& world) {
        return sim::io::collectDeformableSceneDiagnostics(world);
      },
      nb::arg("world"));

  m.def(
      "load_gmsh_tet_mesh",
      [](const std::filesystem::path& path) {
        const auto mesh = sim::io::loadGmshTetMeshFile(path);
        sim::DeformableBodyOptions options;
        options.positions = mesh.positions;
        options.tetrahedra.reserve(mesh.tetrahedra.size());
        for (const auto& tet : mesh.tetrahedra) {
          options.tetrahedra.push_back(
              sim::DeformableTetrahedron{tet[0], tet[1], tet[2], tet[3]});
        }
        return options;
      },
      nb::arg("path"),
      "Load a GMSH ASCII .msh (format 2.x) tetrahedral mesh into a "
      "DeformableBodyOptions (positions + tetrahedra). Set the material "
      "(e.g. use_finite_element_elasticity) and fixed nodes on the result.");

  m.def(
      "load_obj_triangle_mesh",
      [](const std::filesystem::path& path) {
        const auto mesh = sim::io::loadObjTriangleMeshFile(path);
        sim::DeformableBodyOptions options;
        options.positions = mesh.positions;
        options.surfaceTriangles.reserve(mesh.triangles.size());
        for (const auto& tri : mesh.triangles) {
          options.surfaceTriangles.push_back(
              sim::DeformableSurfaceTriangle{tri[0], tri[1], tri[2]});
        }
        return options;
      },
      nb::arg("path"),
      "Load a Wavefront .obj triangle surface mesh into a "
      "DeformableBodyOptions (positions + surface_triangles). Add spring edges "
      "and masses (or tetrahedra + material) to make it a simulable body.");

  m.def(
      "load_seg_line_mesh",
      [](const std::filesystem::path& path) {
        const auto mesh = sim::io::loadSegLineMeshFile(path);
        sim::DeformableBodyOptions options;
        options.positions = mesh.positions;
        options.edges.reserve(mesh.segments.size());
        for (const auto& segment : mesh.segments) {
          // restLength <= 0 asks the body builder to use the initial distance.
          options.edges.push_back(
              sim::DeformableEdge{segment[0], segment[1], -1.0});
        }
        return options;
      },
      nb::arg("path"),
      "Load a .seg segment mesh into a DeformableBodyOptions (positions + "
      "spring edges, rest lengths taken from the initial layout). Add masses "
      "to "
      "make it a simulable mass-spring strand.");

  m.def(
      "load_point_set",
      [](const std::filesystem::path& path) {
        const auto points = sim::io::loadPointSetFile(path);
        sim::DeformableBodyOptions options;
        options.positions = points.positions;
        return options;
      },
      nb::arg("path"),
      "Load a .pt point set into a DeformableBodyOptions (positions only). Add "
      "masses to make it a cloud of free deformable particles.");
}

} // namespace dart::python_nb
