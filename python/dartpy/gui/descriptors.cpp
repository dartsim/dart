#include "gui/descriptors.hpp"

#include "dart/collision/collision_result.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/gui/debug.hpp"
#include "dart/gui/scene.hpp"
#include "dart/gui/view_quality.hpp"
#include "dart/gui/viewer.hpp"
#include "dart/simulation/body/contact.hpp"
#include "dart/simulation/world.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <cstdint>

namespace nb = nanobind;

namespace dart::python_nb {

void defGuiDescriptors(nb::module_& m)
{
  namespace gui = dart::gui;

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

  nb::enum_<gui::MeshAlphaMode>(m, "MeshAlphaMode")
      .value("Blend", gui::MeshAlphaMode::Blend)
      .value("Auto", gui::MeshAlphaMode::Auto)
      .value("ShapeAlpha", gui::MeshAlphaMode::ShapeAlpha);

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
      .def_rw("triangle_vertices", &gui::GeometryDescriptor::triangleVertices)
      .def_rw("triangle_indices", &gui::GeometryDescriptor::triangleIndices)
      .def_rw("triangle_normals", &gui::GeometryDescriptor::triangleNormals)
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
      .def_rw("mesh_alpha_mode", &gui::GeometryDescriptor::meshAlphaMode)
      .def_rw(
          "mesh_texture_coord_components",
          &gui::GeometryDescriptor::meshTextureCoordComponents)
      .def_rw(
          "mesh_texture_coordinates",
          &gui::GeometryDescriptor::meshTextureCoordinates)
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
      .def_rw("receives_shadows", &gui::MaterialDescriptor::receivesShadows)
      .def_rw("metallic", &gui::MaterialDescriptor::metallic)
      .def_rw("roughness", &gui::MaterialDescriptor::roughness)
      .def_rw("reflectance", &gui::MaterialDescriptor::reflectance);

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
      .def_rw("shape_version", &gui::RenderableDescriptor::shapeVersion)
      .def_rw(
          "render_resource_version",
          &gui::RenderableDescriptor::renderResourceVersion);

  nb::class_<gui::RenderableSetUpdatePlan>(m, "RenderableSetUpdatePlan")
      .def(nb::init<>())
      .def_rw(
          "descriptor_indices_to_add",
          &gui::RenderableSetUpdatePlan::descriptorIndicesToAdd)
      .def_rw(
          "active_renderable_indices_to_remove",
          &gui::RenderableSetUpdatePlan::activeRenderableIndicesToRemove);

  nb::class_<gui::ActiveRenderableState>(m, "ActiveRenderableState")
      .def(nb::init<>())
      .def_rw("id", &gui::ActiveRenderableState::id)
      .def_rw("shape_version", &gui::ActiveRenderableState::shapeVersion)
      .def_rw(
          "render_resource_version",
          &gui::ActiveRenderableState::renderResourceVersion);

  nb::class_<gui::PickRay>(m, "PickRay")
      .def(nb::init<>())
      .def_rw("origin", &gui::PickRay::origin)
      .def_rw("direction", &gui::PickRay::direction);

  nb::class_<gui::RunOptions>(m, "RunOptions")
      .def(nb::init<>())
      .def_rw("width", &gui::RunOptions::width)
      .def_rw("height", &gui::RunOptions::height)
      .def_rw("max_frames", &gui::RunOptions::maxFrames)
      .def_rw("gui_scale", &gui::RunOptions::guiScale)
      .def_rw("headless", &gui::RunOptions::headless)
      .def_rw("screenshot_path", &gui::RunOptions::screenshotPath)
      .def_rw("frame_output_directory", &gui::RunOptions::frameOutputDirectory);

  nb::class_<gui::ViewerLifecycleState>(m, "ViewerLifecycleState")
      .def(nb::init<>())
      .def_rw("rendered_frames", &gui::ViewerLifecycleState::renderedFrames)
      .def_rw("skipped_frames", &gui::ViewerLifecycleState::skippedFrames)
      .def_rw("paused", &gui::ViewerLifecycleState::paused)
      .def_rw("step_once", &gui::ViewerLifecycleState::stepOnce)
      .def_rw(
          "screenshot_requested",
          &gui::ViewerLifecycleState::screenshotRequested)
      .def_rw(
          "scene_switch_requested",
          &gui::ViewerLifecycleState::sceneSwitchRequested)
      .def_rw("requested_scene", &gui::ViewerLifecycleState::requestedScene)
      .def_rw(
          "scene_activation_status",
          &gui::ViewerLifecycleState::sceneActivationStatus)
      .def_rw(
          "scene_activation_pending_scene",
          &gui::ViewerLifecycleState::sceneActivationPendingScene);

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

  nb::class_<gui::OrbitCameraController>(m, "OrbitCameraController")
      .def(nb::init<>())
      .def_rw("camera", &gui::OrbitCameraController::camera)
      .def_rw("last_cursor_x", &gui::OrbitCameraController::lastCursorX)
      .def_rw("last_cursor_y", &gui::OrbitCameraController::lastCursorY)
      .def_rw("scroll_delta", &gui::OrbitCameraController::scrollDelta)
      .def_rw("has_last_cursor", &gui::OrbitCameraController::hasLastCursor);

  nb::class_<gui::OrbitCameraControllerInput>(m, "OrbitCameraControllerInput")
      .def(nb::init<>())
      .def_rw("cursor_x", &gui::OrbitCameraControllerInput::cursorX)
      .def_rw("cursor_y", &gui::OrbitCameraControllerInput::cursorY)
      .def_rw("has_cursor", &gui::OrbitCameraControllerInput::hasCursor)
      .def_rw("orbit", &gui::OrbitCameraControllerInput::orbit)
      .def_rw("pan", &gui::OrbitCameraControllerInput::pan);

  nb::class_<gui::DirectionalNudgeInput>(m, "DirectionalNudgeInput")
      .def(nb::init<>())
      .def_rw("left", &gui::DirectionalNudgeInput::left)
      .def_rw("right", &gui::DirectionalNudgeInput::right)
      .def_rw("forward", &gui::DirectionalNudgeInput::forward)
      .def_rw("backward", &gui::DirectionalNudgeInput::backward)
      .def_rw("up", &gui::DirectionalNudgeInput::up)
      .def_rw("down", &gui::DirectionalNudgeInput::down)
      .def_rw("fast", &gui::DirectionalNudgeInput::fast)
      .def_rw("step_size", &gui::DirectionalNudgeInput::stepSize)
      .def_rw("fast_multiplier", &gui::DirectionalNudgeInput::fastMultiplier);

  nb::class_<gui::ProjectionOptions>(m, "ProjectionOptions")
      .def(nb::init<>())
      .def_rw(
          "vertical_fov_degrees", &gui::ProjectionOptions::verticalFovDegrees)
      .def_rw("near_plane", &gui::ProjectionOptions::nearPlane)
      .def_rw("far_plane", &gui::ProjectionOptions::farPlane)
      .def_rw("near_scale", &gui::ProjectionOptions::nearScale)
      .def_rw("min_near_plane", &gui::ProjectionOptions::minNearPlane)
      .def_rw("max_near_plane", &gui::ProjectionOptions::maxNearPlane)
      .def_rw("min_far_plane", &gui::ProjectionOptions::minFarPlane)
      .def_rw("far_padding", &gui::ProjectionOptions::farPadding);

  nb::class_<gui::PerspectiveProjection>(m, "PerspectiveProjection")
      .def(nb::init<>())
      .def_rw(
          "vertical_fov_degrees",
          &gui::PerspectiveProjection::verticalFovDegrees)
      .def_rw("aspect_ratio", &gui::PerspectiveProjection::aspectRatio)
      .def_rw("near_plane", &gui::PerspectiveProjection::nearPlane)
      .def_rw("far_plane", &gui::PerspectiveProjection::farPlane);

  nb::class_<gui::PickHit>(m, "PickHit")
      .def(nb::init<>())
      .def_rw("id", &gui::PickHit::id)
      .def_rw("renderable_index", &gui::PickHit::renderableIndex)
      .def_rw("distance", &gui::PickHit::distance)
      .def_rw("point", &gui::PickHit::point)
      .def_rw("normal", &gui::PickHit::normal);

  nb::class_<gui::ViewQualityReport>(m, "ViewQualityReport")
      .def_ro("corner_coverage", &gui::ViewQualityReport::cornerCoverage)
      .def_ro("subject_fraction", &gui::ViewQualityReport::subjectFraction)
      .def_ro("center_visible", &gui::ViewQualityReport::centerVisible)
      .def_ro("occlusion_fraction", &gui::ViewQualityReport::occlusionFraction)
      .def_ro("ambiguity_iou", &gui::ViewQualityReport::ambiguityIoU)
      .def_ro("issues", &gui::ViewQualityReport::issues)
      .def_ro("score", &gui::ViewQualityReport::score);

  nb::class_<gui::DebugLineDescriptor>(m, "DebugLineDescriptor")
      .def(nb::init<>())
      .def_rw("from_point", &gui::DebugLineDescriptor::from)
      .def_rw("to_point", &gui::DebugLineDescriptor::to)
      .def_rw("rgba", &gui::DebugLineDescriptor::rgba)
      .def_rw("thickness", &gui::DebugLineDescriptor::thickness)
      .def_rw("label", &gui::DebugLineDescriptor::label);

  nb::class_<gui::DebugTriangleDescriptor>(m, "DebugTriangleDescriptor")
      .def(nb::init<>())
      .def_rw("a", &gui::DebugTriangleDescriptor::a)
      .def_rw("b", &gui::DebugTriangleDescriptor::b)
      .def_rw("c", &gui::DebugTriangleDescriptor::c)
      .def_rw("rgba", &gui::DebugTriangleDescriptor::rgba)
      .def_rw("label", &gui::DebugTriangleDescriptor::label);

  nb::class_<gui::DebugLabelDescriptor>(m, "DebugLabelDescriptor")
      .def(nb::init<>())
      .def_rw("position", &gui::DebugLabelDescriptor::position)
      .def_rw("rgba", &gui::DebugLabelDescriptor::rgba)
      .def_rw("text", &gui::DebugLabelDescriptor::text);

  nb::class_<gui::DebugScene>(m, "DebugScene")
      .def(nb::init<>())
      .def_rw("lines", &gui::DebugScene::lines)
      .def_rw("triangles", &gui::DebugScene::triangles)
      .def_rw("labels", &gui::DebugScene::labels);

  nb::class_<gui::DebugDrawOptions>(m, "DebugDrawOptions")
      .def(nb::init<>())
      .def_rw("draw_grid", &gui::DebugDrawOptions::drawGrid)
      .def_rw("draw_world_frame", &gui::DebugDrawOptions::drawWorldFrame)
      .def_rw("draw_body_frames", &gui::DebugDrawOptions::drawBodyFrames)
      .def_rw("draw_centers_of_mass", &gui::DebugDrawOptions::drawCentersOfMass)
      .def_rw("draw_inertia_boxes", &gui::DebugDrawOptions::drawInertiaBoxes)
      .def_rw(
          "draw_collision_shape_bounds",
          &gui::DebugDrawOptions::drawCollisionShapeBounds)
      .def_rw(
          "draw_support_polygons", &gui::DebugDrawOptions::drawSupportPolygons)
      .def_rw(
          "draw_support_centroids",
          &gui::DebugDrawOptions::drawSupportCentroids)
      .def_rw("draw_contacts", &gui::DebugDrawOptions::drawContacts)
      .def_rw(
          "draw_contact_normals", &gui::DebugDrawOptions::drawContactNormals)
      .def_rw("draw_contact_forces", &gui::DebugDrawOptions::drawContactForces)
      .def_rw("draw_joint_axes", &gui::DebugDrawOptions::drawJointAxes)
      .def_rw(
          "draw_linear_velocities",
          &gui::DebugDrawOptions::drawLinearVelocities)
      .def_rw(
          "draw_angular_velocities",
          &gui::DebugDrawOptions::drawAngularVelocities)
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
      .def_rw("inertia_box_scale", &gui::DebugDrawOptions::inertiaBoxScale)
      .def_rw(
          "collision_bounds_padding",
          &gui::DebugDrawOptions::collisionBoundsPadding)
      .def_rw(
          "support_polygon_elevation",
          &gui::DebugDrawOptions::supportPolygonElevation)
      .def_rw(
          "support_centroid_marker_radius",
          &gui::DebugDrawOptions::supportCentroidMarkerRadius)
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
          &gui::DebugDrawOptions::contactForceMaxLength)
      .def_rw("joint_axis_length", &gui::DebugDrawOptions::jointAxisLength)
      .def_rw(
          "linear_velocity_scale", &gui::DebugDrawOptions::linearVelocityScale)
      .def_rw(
          "angular_velocity_scale",
          &gui::DebugDrawOptions::angularVelocityScale)
      .def_rw("velocity_min_length", &gui::DebugDrawOptions::velocityMinLength)
      .def_rw("velocity_max_length", &gui::DebugDrawOptions::velocityMaxLength)
      .def_rw("line_thickness", &gui::DebugDrawOptions::lineThickness);

  m.def("describe_shape", &gui::describeShape, nb::arg("shape"));
  m.def(
      "plan_renderable_set_update",
      static_cast<gui::RenderableSetUpdatePlan (*)(
          const std::vector<gui::RenderableDescriptor>&,
          const std::vector<gui::RenderableId>&)>(
          &gui::planRenderableSetUpdate),
      nb::arg("descriptors"),
      nb::arg("active_renderable_ids"));
  m.def(
      "plan_renderable_set_update",
      static_cast<gui::RenderableSetUpdatePlan (*)(
          const std::vector<gui::RenderableDescriptor>&,
          const std::vector<gui::ActiveRenderableState>&)>(
          &gui::planRenderableSetUpdate),
      nb::arg("descriptors"),
      nb::arg("active_renderable_states"));
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
      "compute_axis_drag_translation",
      &gui::computeAxisDragTranslation,
      nb::arg("previous_ray"),
      nb::arg("current_ray"),
      nb::arg("axis_point"),
      nb::arg("axis_direction"));
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
      "should_capture_frame_output",
      [](const gui::RunOptions& options) {
        return gui::shouldCaptureFrameOutput(options);
      },
      nb::arg("options"));
  m.def(
      "make_frame_output_path",
      [](const gui::RunOptions& options, int frameNumber) {
        return gui::makeFrameOutputPath(options, frameNumber);
      },
      nb::arg("options"),
      nb::arg("frame_number"));
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
      "request_scene_switch",
      &gui::requestSceneSwitch,
      nb::arg("state"),
      nb::arg("scene_id"));
  m.def(
      "request_scene_replay",
      &gui::requestSceneReplay,
      nb::arg("state"),
      nb::arg("scene_id"));
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
      "add_orbit_camera_scroll",
      &gui::addOrbitCameraScroll,
      nb::arg("controller"),
      nb::arg("scroll_delta"));
  m.def(
      "reset_orbit_camera_tracking",
      &gui::resetOrbitCameraTracking,
      nb::arg("controller"));
  m.def(
      "update_orbit_camera_controller",
      &gui::updateOrbitCameraController,
      nb::arg("controller"),
      nb::arg("input"));
  m.def(
      "compute_camera_relative_nudge",
      &gui::computeCameraRelativeNudge,
      nb::arg("camera"),
      nb::arg("input"));
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
      "make_perspective_projection",
      &gui::makePerspectiveProjection,
      nb::arg("camera"),
      nb::arg("width"),
      nb::arg("height"),
      nb::arg("options") = gui::ProjectionOptions{});
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
      "make_inertia_debug_lines",
      &gui::makeInertiaDebugLines,
      nb::arg("body_node"),
      nb::arg("options") = gui::DebugDrawOptions{},
      nb::arg("label_prefix") = std::string{});
  m.def(
      "make_collision_shape_debug_lines",
      &gui::makeCollisionShapeDebugLines,
      nb::arg("shape_node"),
      nb::arg("options") = gui::DebugDrawOptions{},
      nb::arg("label_prefix") = std::string{});
  m.def(
      "make_support_polygon_debug_lines",
      &gui::makeSupportPolygonDebugLines,
      nb::arg("skeleton"),
      nb::arg("options") = gui::DebugDrawOptions{},
      nb::arg("label_prefix") = std::string{});
  m.def(
      "make_joint_axis_debug_lines",
      &gui::makeJointAxisDebugLines,
      nb::arg("body_node"),
      nb::arg("options") = gui::DebugDrawOptions{},
      nb::arg("label_prefix") = std::string{});
  m.def(
      "make_velocity_debug_lines",
      &gui::makeVelocityDebugLines,
      nb::arg("body_node"),
      nb::arg("options") = gui::DebugDrawOptions{},
      nb::arg("label_prefix") = std::string{});
  m.def(
      "extract_contact_debug_lines",
      static_cast<std::vector<gui::DebugLineDescriptor> (*)(
          const collision::CollisionResult&, const gui::DebugDrawOptions&)>(
          &gui::extractContactDebugLines),
      nb::arg("result"),
      nb::arg("options") = gui::DebugDrawOptions{});
  // Overload for the promoted simulation contact type returned by
  // World::collide(); nanobind tries the registered overloads in order, so a
  // Python list falls through to this one after the CollisionResult overload.
  m.def(
      "extract_contact_debug_lines",
      static_cast<std::vector<gui::DebugLineDescriptor> (*)(
          const std::vector<simulation::Contact>&,
          const gui::DebugDrawOptions&)>(&gui::extractContactDebugLines),
      nb::arg("contacts"),
      nb::arg("options") = gui::DebugDrawOptions{});
  m.def(
      "extract_debug_lines",
      static_cast<std::vector<gui::DebugLineDescriptor> (*)(
          const gui::DebugDrawOptions&)>(&gui::extractDebugLines),
      nb::arg("options") = gui::DebugDrawOptions{});
  // World-aware debug extraction bound under a distinct name so it never shares
  // an overload set with the options-only extract_debug_lines above.
  m.def(
      "extract_world_debug_lines",
      static_cast<std::vector<gui::DebugLineDescriptor> (*)(
          simulation::World&, const gui::DebugDrawOptions&)>(
          &gui::extractDebugLines),
      nb::arg("world"),
      nb::arg("options") = gui::DebugDrawOptions{});
  m.def(
      "make_polyline_debug_lines",
      &gui::makePolylineDebugLines,
      nb::arg("points"),
      nb::arg("rgba"),
      nb::arg("label") = std::string{},
      nb::arg("thickness") = 0.0);
  m.def(
      "project_to_pixels",
      &gui::projectToPixels,
      nb::arg("camera"),
      nb::arg("width"),
      nb::arg("height"),
      nb::arg("point"),
      nb::arg("options") = gui::ProjectionOptions{});
  m.def(
      "assess_view_quality",
      &gui::assessView,
      nb::arg("descriptors"),
      nb::arg("camera"),
      nb::arg("width"),
      nb::arg("height"),
      nb::arg("focus_ids") = std::vector<gui::RenderableId>{},
      nb::arg("options") = gui::ProjectionOptions{});
}

} // namespace dart::python_nb
