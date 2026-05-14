#include "gui/experimental.hpp"

#include "dart/collision/collision_result.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/gui/experimental/scene.hpp"
#include "dart/simulation/world.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <limits>
#include <stdexcept>
#include <string>

#include <cstdint>

namespace nb = nanobind;

namespace dart::python_nb {

void defGuiExperimentalModule(nb::module_& m)
{
  namespace gui = dart::gui::experimental;

  nb::enum_<gui::ShapeKind>(m, "ShapeKind")
      .value("Box", gui::ShapeKind::Box)
      .value("Sphere", gui::ShapeKind::Sphere)
      .value("Ellipsoid", gui::ShapeKind::Ellipsoid)
      .value("Cylinder", gui::ShapeKind::Cylinder)
      .value("Capsule", gui::ShapeKind::Capsule)
      .value("Cone", gui::ShapeKind::Cone)
      .value("Pyramid", gui::ShapeKind::Pyramid)
      .value("MultiSphere", gui::ShapeKind::MultiSphere)
      .value("LineSegments", gui::ShapeKind::LineSegments)
      .value("ConvexMesh", gui::ShapeKind::ConvexMesh)
      .value("PointCloud", gui::ShapeKind::PointCloud)
      .value("Heightmap", gui::ShapeKind::Heightmap)
      .value("SoftMesh", gui::ShapeKind::SoftMesh)
      .value("VoxelGrid", gui::ShapeKind::VoxelGrid)
      .value("Mesh", gui::ShapeKind::Mesh)
      .value("Plane", gui::ShapeKind::Plane)
      .value("Unsupported", gui::ShapeKind::Unsupported);

  nb::class_<gui::MeshMaterialDescriptor>(m, "MeshMaterialDescriptor")
      .def(nb::init<>())
      .def_rw("ambient", &gui::MeshMaterialDescriptor::ambient)
      .def_rw("diffuse", &gui::MeshMaterialDescriptor::diffuse)
      .def_rw("specular", &gui::MeshMaterialDescriptor::specular)
      .def_rw("emissive", &gui::MeshMaterialDescriptor::emissive)
      .def_rw("shininess", &gui::MeshMaterialDescriptor::shininess)
      .def_rw("metallic_factor", &gui::MeshMaterialDescriptor::metallicFactor)
      .def_rw("roughness_factor", &gui::MeshMaterialDescriptor::roughnessFactor)
      .def_rw(
          "base_color_texture_path",
          &gui::MeshMaterialDescriptor::baseColorTexturePath)
      .def_rw(
          "metallic_texture_path",
          &gui::MeshMaterialDescriptor::metallicTexturePath)
      .def_rw(
          "roughness_texture_path",
          &gui::MeshMaterialDescriptor::roughnessTexturePath)
      .def_rw(
          "metallic_roughness_texture_path",
          &gui::MeshMaterialDescriptor::metallicRoughnessTexturePath)
      .def_rw(
          "normal_texture_path",
          &gui::MeshMaterialDescriptor::normalTexturePath)
      .def_rw(
          "occlusion_texture_path",
          &gui::MeshMaterialDescriptor::occlusionTexturePath)
      .def_rw(
          "emissive_texture_path",
          &gui::MeshMaterialDescriptor::emissiveTexturePath)
      .def_rw(
          "texture_image_paths",
          &gui::MeshMaterialDescriptor::textureImagePaths);

  nb::class_<gui::MeshPartDescriptor>(m, "MeshPartDescriptor")
      .def(nb::init<>())
      .def_rw("vertex_offset", &gui::MeshPartDescriptor::vertexOffset)
      .def_rw("vertex_count", &gui::MeshPartDescriptor::vertexCount)
      .def_rw("triangle_offset", &gui::MeshPartDescriptor::triangleOffset)
      .def_rw("triangle_count", &gui::MeshPartDescriptor::triangleCount)
      .def_rw("material_index", &gui::MeshPartDescriptor::materialIndex);

  nb::class_<gui::GeometryDescriptor>(m, "GeometryDescriptor")
      .def(nb::init<>())
      .def_rw("kind", &gui::GeometryDescriptor::kind)
      .def_rw("size", &gui::GeometryDescriptor::size)
      .def_rw("scale", &gui::GeometryDescriptor::scale)
      .def_rw("normal", &gui::GeometryDescriptor::normal)
      .def_rw("local_bounds_min", &gui::GeometryDescriptor::localBoundsMin)
      .def_rw("local_bounds_max", &gui::GeometryDescriptor::localBoundsMax)
      .def_rw("sphere_centers", &gui::GeometryDescriptor::sphereCenters)
      .def_rw("sphere_radii", &gui::GeometryDescriptor::sphereRadii)
      .def_rw("line_vertices", &gui::GeometryDescriptor::lineVertices)
      .def_rw("line_connections", &gui::GeometryDescriptor::lineConnections)
      .def_rw("point_cloud_points", &gui::GeometryDescriptor::pointCloudPoints)
      .def_rw("point_cloud_colors", &gui::GeometryDescriptor::pointCloudColors)
      .def_rw("voxel_centers", &gui::GeometryDescriptor::voxelCenters)
      .def_rw("radius", &gui::GeometryDescriptor::radius)
      .def_rw("height", &gui::GeometryDescriptor::height)
      .def_rw("offset", &gui::GeometryDescriptor::offset)
      .def_rw("line_thickness", &gui::GeometryDescriptor::lineThickness)
      .def_rw("point_size", &gui::GeometryDescriptor::pointSize)
      .def_rw("voxel_size", &gui::GeometryDescriptor::voxelSize)
      .def_rw("has_local_bounds", &gui::GeometryDescriptor::hasLocalBounds)
      .def_rw(
          "mesh_uses_material_colors",
          &gui::GeometryDescriptor::meshUsesMaterialColors)
      .def_rw(
          "mesh_texture_coord_components",
          &gui::GeometryDescriptor::meshTextureCoordComponents)
      .def_rw("mesh_uri", &gui::GeometryDescriptor::meshUri)
      .def_rw("shape_type", &gui::GeometryDescriptor::shapeType)
      .def_rw("unsupported_reason", &gui::GeometryDescriptor::unsupportedReason)
      .def_rw("mesh_materials", &gui::GeometryDescriptor::meshMaterials)
      .def_rw("mesh_parts", &gui::GeometryDescriptor::meshParts);

  nb::class_<gui::MaterialDescriptor>(m, "MaterialDescriptor")
      .def(nb::init<>())
      .def_rw("rgba", &gui::MaterialDescriptor::rgba)
      .def_rw("visible", &gui::MaterialDescriptor::visible)
      .def_rw("casts_shadows", &gui::MaterialDescriptor::castsShadows)
      .def_rw("receives_shadows", &gui::MaterialDescriptor::receivesShadows);

  nb::class_<gui::RenderableDescriptor>(m, "RenderableDescriptor")
      .def(nb::init<>())
      .def_rw("id", &gui::RenderableDescriptor::id)
      .def_rw("skeleton_name", &gui::RenderableDescriptor::skeletonName)
      .def_rw("body_name", &gui::RenderableDescriptor::bodyName)
      .def_rw("shape_frame_name", &gui::RenderableDescriptor::shapeFrameName)
      .def_rw("shape_node_name", &gui::RenderableDescriptor::shapeNodeName)
      .def_rw("geometry", &gui::RenderableDescriptor::geometry)
      .def_rw("material", &gui::RenderableDescriptor::material)
      .def_rw("world_transform", &gui::RenderableDescriptor::worldTransform)
      .def_rw(
          "shape_frame_version", &gui::RenderableDescriptor::shapeFrameVersion)
      .def_rw(
          "shape_node_version", &gui::RenderableDescriptor::shapeNodeVersion)
      .def_rw("shape_version", &gui::RenderableDescriptor::shapeVersion);

  nb::class_<gui::PickRay>(m, "PickRay")
      .def(nb::init<>())
      .def_rw("origin", &gui::PickRay::origin)
      .def_rw("direction", &gui::PickRay::direction);

  nb::class_<gui::RunOptions>(m, "RunOptions")
      .def(nb::init<>())
      .def_rw("width", &gui::RunOptions::width)
      .def_rw("height", &gui::RunOptions::height)
      .def_rw("max_frames", &gui::RunOptions::maxFrames)
      .def_rw("headless", &gui::RunOptions::headless)
      .def_rw("screenshot_path", &gui::RunOptions::screenshotPath);

  nb::class_<gui::ViewerLifecycleState>(m, "ViewerLifecycleState")
      .def(nb::init<>())
      .def_rw("rendered_frames", &gui::ViewerLifecycleState::renderedFrames)
      .def_rw("skipped_frames", &gui::ViewerLifecycleState::skippedFrames)
      .def_rw("paused", &gui::ViewerLifecycleState::paused)
      .def_rw("step_once", &gui::ViewerLifecycleState::stepOnce)
      .def_rw(
          "screenshot_requested",
          &gui::ViewerLifecycleState::screenshotRequested);

  nb::class_<gui::OrbitCamera>(m, "OrbitCamera")
      .def(nb::init<>())
      .def_rw("target", &gui::OrbitCamera::target)
      .def_rw("yaw", &gui::OrbitCamera::yaw)
      .def_rw("pitch", &gui::OrbitCamera::pitch)
      .def_rw("distance", &gui::OrbitCamera::distance);

  nb::class_<gui::OrbitCameraBasis>(m, "OrbitCameraBasis")
      .def(nb::init<>())
      .def_rw("eye", &gui::OrbitCameraBasis::eye)
      .def_rw("forward", &gui::OrbitCameraBasis::forward)
      .def_rw("right", &gui::OrbitCameraBasis::right)
      .def_rw("up", &gui::OrbitCameraBasis::up);

  nb::class_<gui::OrbitCameraUpdate>(m, "OrbitCameraUpdate")
      .def(nb::init<>())
      .def_rw("delta_x", &gui::OrbitCameraUpdate::deltaX)
      .def_rw("delta_y", &gui::OrbitCameraUpdate::deltaY)
      .def_rw("scroll_delta", &gui::OrbitCameraUpdate::scrollDelta)
      .def_rw("orbit", &gui::OrbitCameraUpdate::orbit)
      .def_rw("pan", &gui::OrbitCameraUpdate::pan)
      .def_rw("orbit_scale", &gui::OrbitCameraUpdate::orbitScale)
      .def_rw("pan_scale", &gui::OrbitCameraUpdate::panScale)
      .def_rw("scroll_scale", &gui::OrbitCameraUpdate::scrollScale)
      .def_rw("min_distance", &gui::OrbitCameraUpdate::minDistance)
      .def_rw("max_distance", &gui::OrbitCameraUpdate::maxDistance)
      .def_rw("min_pitch", &gui::OrbitCameraUpdate::minPitch)
      .def_rw("max_pitch", &gui::OrbitCameraUpdate::maxPitch);

  nb::class_<gui::PickHit>(m, "PickHit")
      .def(nb::init<>())
      .def_rw("id", &gui::PickHit::id)
      .def_rw("renderable_index", &gui::PickHit::renderableIndex)
      .def_rw("distance", &gui::PickHit::distance)
      .def_rw("point", &gui::PickHit::point);

  nb::class_<gui::DebugLineDescriptor>(m, "DebugLineDescriptor")
      .def(nb::init<>())
      .def_rw("from_point", &gui::DebugLineDescriptor::from)
      .def_rw("to_point", &gui::DebugLineDescriptor::to)
      .def_rw("rgba", &gui::DebugLineDescriptor::rgba)
      .def_rw("label", &gui::DebugLineDescriptor::label);

  nb::class_<gui::DebugDrawOptions>(m, "DebugDrawOptions")
      .def(nb::init<>())
      .def_rw("draw_grid", &gui::DebugDrawOptions::drawGrid)
      .def_rw("draw_world_frame", &gui::DebugDrawOptions::drawWorldFrame)
      .def_rw("draw_body_frames", &gui::DebugDrawOptions::drawBodyFrames)
      .def_rw("draw_centers_of_mass", &gui::DebugDrawOptions::drawCentersOfMass)
      .def_rw("draw_contacts", &gui::DebugDrawOptions::drawContacts)
      .def_rw(
          "draw_contact_normals", &gui::DebugDrawOptions::drawContactNormals)
      .def_rw("draw_contact_forces", &gui::DebugDrawOptions::drawContactForces)
      .def_rw("grid_half_extent", &gui::DebugDrawOptions::gridHalfExtent)
      .def_rw("grid_spacing", &gui::DebugDrawOptions::gridSpacing)
      .def_rw("grid_z", &gui::DebugDrawOptions::gridZ)
      .def_rw(
          "world_frame_axis_length",
          &gui::DebugDrawOptions::worldFrameAxisLength)
      .def_rw(
          "body_frame_axis_length", &gui::DebugDrawOptions::bodyFrameAxisLength)
      .def_rw(
          "center_of_mass_marker_radius",
          &gui::DebugDrawOptions::centerOfMassMarkerRadius)
      .def_rw(
          "contact_marker_half_extent",
          &gui::DebugDrawOptions::contactMarkerHalfExtent)
      .def_rw(
          "contact_normal_length", &gui::DebugDrawOptions::contactNormalLength)
      .def_rw("contact_force_scale", &gui::DebugDrawOptions::contactForceScale)
      .def_rw(
          "contact_force_min_length",
          &gui::DebugDrawOptions::contactForceMinLength)
      .def_rw(
          "contact_force_max_length",
          &gui::DebugDrawOptions::contactForceMaxLength);

  m.def("describe_shape", &gui::describeShape, nb::arg("shape"));
  m.def("extract_renderables", &gui::extractRenderables, nb::arg("world"));
  m.def(
      "intersect_renderable",
      &gui::intersectRenderable,
      nb::arg("renderable"),
      nb::arg("ray"));
  m.def(
      "pick_nearest_renderable",
      &gui::pickNearestRenderable,
      nb::arg("renderables"),
      nb::arg("ray"),
      nb::arg("max_distance") = std::numeric_limits<double>::infinity());
  m.def(
      "intersect_plane",
      &gui::intersectPlane,
      nb::arg("ray"),
      nb::arg("plane_point"),
      nb::arg("plane_normal"));
  m.def(
      "compute_plane_drag_translation",
      &gui::computePlaneDragTranslation,
      nb::arg("previous_ray"),
      nb::arg("current_ray"),
      nb::arg("plane_point"),
      nb::arg("plane_normal"));
  m.def(
      "translate_free_joint_renderable",
      &gui::translateFreeJointRenderable,
      nb::arg("renderable"),
      nb::arg("world_translation"));
  m.def(
      "translate_simple_frame_renderable",
      &gui::translateSimpleFrameRenderable,
      nb::arg("renderable"),
      nb::arg("world_translation"));
  m.def(
      "translate_frame_renderable",
      &gui::translateFrameRenderable,
      nb::arg("renderable"),
      nb::arg("world_translation"));
  m.def("normalize_run_options", &gui::normalizeRunOptions, nb::arg("options"));
  m.def(
      "should_request_screenshot",
      [](const gui::RunOptions& options,
         int renderedFrames,
         bool screenshotRequested) {
        return gui::shouldRequestScreenshot(
            options, renderedFrames, screenshotRequested);
      },
      nb::arg("options"),
      nb::arg("rendered_frames"),
      nb::arg("screenshot_requested"));
  m.def(
      "should_stop_after_frame",
      [](const gui::RunOptions& options, int renderedFrames) {
        return gui::shouldStopAfterFrame(options, renderedFrames);
      },
      nb::arg("options"),
      nb::arg("rendered_frames"));
  m.def("toggle_paused", &gui::togglePaused, nb::arg("state"));
  m.def(
      "request_single_step",
      &gui::requestSingleStep,
      nb::arg("state"),
      nb::arg("pause") = true);
  m.def(
      "should_advance_simulation",
      &gui::shouldAdvanceSimulation,
      nb::arg("state"));
  m.def(
      "mark_simulation_advanced",
      &gui::markSimulationAdvanced,
      nb::arg("state"));
  m.def(
      "should_request_screenshot",
      [](const gui::RunOptions& options,
         const gui::ViewerLifecycleState& state) {
        return gui::shouldRequestScreenshot(options, state);
      },
      nb::arg("options"),
      nb::arg("state"));
  m.def(
      "mark_screenshot_requested",
      &gui::markScreenshotRequested,
      nb::arg("state"));
  m.def("mark_frame_rendered", &gui::markFrameRendered, nb::arg("state"));
  m.def("mark_frame_skipped", &gui::markFrameSkipped, nb::arg("state"));
  m.def(
      "should_stop_after_frame",
      [](const gui::RunOptions& options,
         const gui::ViewerLifecycleState& state) {
        return gui::shouldStopAfterFrame(options, state);
      },
      nb::arg("options"),
      nb::arg("state"));
  m.def(
      "write_rgba_ppm",
      [](const std::string& path,
         std::uint32_t width,
         std::uint32_t height,
         const std::vector<std::uint8_t>& rgbaPixels,
         bool originBottomLeft) {
        std::string error;
        if (!gui::writeRgbaPpm(
                path, width, height, rgbaPixels, originBottomLeft, &error)) {
          throw std::runtime_error(error);
        }
      },
      nb::arg("path"),
      nb::arg("width"),
      nb::arg("height"),
      nb::arg("rgba_pixels"),
      nb::arg("origin_bottom_left") = false);
  m.def(
      "make_orbit_camera_basis", &gui::makeOrbitCameraBasis, nb::arg("camera"));
  m.def("camera_eye", &gui::cameraEye, nb::arg("camera"));
  m.def(
      "update_orbit_camera",
      &gui::updateOrbitCamera,
      nb::arg("camera"),
      nb::arg("update"));
  m.def(
      "make_perspective_pick_ray",
      &gui::makePerspectivePickRay,
      nb::arg("camera"),
      nb::arg("cursor_x"),
      nb::arg("cursor_y"),
      nb::arg("width"),
      nb::arg("height"),
      nb::arg("vertical_fov_radians") = 0.7853981633974483);
  m.def(
      "make_grid_debug_lines",
      &gui::makeGridDebugLines,
      nb::arg("options") = gui::DebugDrawOptions{});
  m.def(
      "make_frame_debug_lines",
      &gui::makeFrameDebugLines,
      nb::arg("transform"),
      nb::arg("axis_length"),
      nb::arg("label_prefix") = std::string{});
  m.def(
      "make_selection_debug_lines",
      [](const gui::RenderableDescriptor& renderable,
         const Eigen::Vector4d& rgba,
         const std::string& labelPrefix) {
        return gui::makeSelectionDebugLines(renderable, rgba, labelPrefix);
      },
      nb::arg("renderable"),
      nb::arg("rgba") = Eigen::Vector4d(1.0, 0.84, 0.18, 1.0),
      nb::arg("label_prefix") = std::string{});
  m.def(
      "extract_contact_debug_lines",
      &gui::extractContactDebugLines,
      nb::arg("result"),
      nb::arg("options") = gui::DebugDrawOptions{});
  m.def(
      "extract_debug_lines",
      &gui::extractDebugLines,
      nb::arg("world"),
      nb::arg("options") = gui::DebugDrawOptions{});
}

} // namespace dart::python_nb
