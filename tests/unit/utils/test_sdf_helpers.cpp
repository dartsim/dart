#include <dart/config.hpp>

#include <dart/utils/sdf/detail/geometry_parsers.hpp>
#include <dart/utils/sdf/detail/sdf_helpers.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <gtest/gtest.h>
#include <sdf/Geometry.hh>

#include <string>

namespace detail = dart::utils::SdfParser::detail;

namespace {

sdf::ElementPtr loadElement(const std::string& xml)
{
  sdf::Root root;
  sdf::ParserConfig config = sdf::ParserConfig::GlobalConfig();
  sdf::Errors errors = root.LoadSdfString(xml, config);
  EXPECT_TRUE(errors.empty());
  return root.Element();
}

sdf::ElementPtr loadLinkElement(const std::string& linkContents)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          )" + linkContents
                          + R"(
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  if (!sdfElement) {
    return nullptr;
  }

  const auto modelElement = detail::getElement(sdfElement, "model");
  if (!modelElement) {
    return nullptr;
  }

  return detail::getElement(modelElement, "link");
}

sdf::ElementPtr loadGeometryElement(const std::string& geometryContents)
{
  const sdf::ElementPtr linkElement = loadLinkElement(R"(
    <visual name='visual'>
      <geometry>
        )" + geometryContents + R"(
      </geometry>
    </visual>
  )");
  if (!linkElement) {
    return nullptr;
  }

  const auto visualElement = detail::getElement(linkElement, "visual");
  if (!visualElement) {
    return nullptr;
  }

  return detail::getElement(visualElement, "geometry");
}

sdf::Geometry loadGeometry(const std::string& geometryContents)
{
  sdf::Geometry geometry;
  (void)geometry.Load(loadGeometryElement(geometryContents));
  return geometry;
}

} // namespace

TEST(SdfDetailHelpers, ParseVector3d)
{
  const std::string xml = R"(
    <sdf version='1.6'>
      <world name='default'>
        <gravity>0 0 -9.81</gravity>
      </world>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);

  const auto worldElement = detail::getElement(sdfElement, "world");
  ASSERT_TRUE(worldElement);

  const Eigen::Vector3d gravity
      = detail::getValueVector3d(worldElement, "gravity");
  EXPECT_DOUBLE_EQ(gravity.x(), 0.0);
  EXPECT_DOUBLE_EQ(gravity.y(), 0.0);
  EXPECT_DOUBLE_EQ(gravity.z(), -9.81);
}

TEST(SdfDetailHelpers, ReadGeometryShapesAndFailures)
{
  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();
  const dart::common::Uri baseUri("file:///tmp/model.sdf");

  const auto sphereShape = detail::readGeometryShape(
      loadGeometry(R"(
        <sphere>
          <radius>0.25</radius>
        </sphere>
      )"),
      baseUri,
      retriever);
  ASSERT_TRUE(sphereShape);

  const auto sphere
      = std::dynamic_pointer_cast<dart::dynamics::SphereShape>(sphereShape);
  ASSERT_TRUE(sphere);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), 0.25);

  const auto boxShape = detail::readGeometryShape(
      loadGeometry(R"(
        <box>
          <size>1 2 3</size>
        </box>
      )"),
      baseUri,
      retriever);
  const auto box
      = std::dynamic_pointer_cast<dart::dynamics::BoxShape>(boxShape);
  ASSERT_TRUE(box);
  EXPECT_TRUE(box->getSize().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  const auto cylinderShape = detail::readGeometryShape(
      loadGeometry(R"(
        <cylinder>
          <radius>0.4</radius>
          <length>0.8</length>
        </cylinder>
      )"),
      baseUri,
      retriever);
  const auto cylinder
      = std::dynamic_pointer_cast<dart::dynamics::CylinderShape>(cylinderShape);
  ASSERT_TRUE(cylinder);
  EXPECT_DOUBLE_EQ(cylinder->getRadius(), 0.4);
  EXPECT_DOUBLE_EQ(cylinder->getHeight(), 0.8);

  const auto planeShape = detail::readGeometryShape(
      loadGeometry(R"(
        <plane>
          <size>2 3</size>
        </plane>
      )"),
      baseUri,
      retriever);
  const auto plane
      = std::dynamic_pointer_cast<dart::dynamics::BoxShape>(planeShape);
  ASSERT_TRUE(plane);
  EXPECT_TRUE(plane->getSize().isApprox(Eigen::Vector3d(2.0, 3.0, 0.001)));

  EXPECT_EQ(
      detail::readGeometryShape(loadGeometry("<mesh/>"), baseUri, retriever),
      nullptr);
  EXPECT_EQ(
      detail::readGeometryShape(
          loadGeometry("<mesh><uri>missing.obj</uri></mesh>"),
          baseUri,
          retriever),
      nullptr);
  EXPECT_EQ(
      detail::readGeometryShape(sdf::Geometry(), baseUri, retriever), nullptr);
}

TEST(SdfDetailHelpers, ParsesScalarsVectorsAndPose)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <world name='default'>
        <gravity>-1.0 2.0 -9.3</gravity>
        <model name='demo_model'>
          <link name='demo_link'>
            <pose>1 2 3 0 0 1.57079632679</pose>
            <mass>1.5</mass>
            <grid>1 2 3</grid>
          </link>
        </model>
      </world>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto worldElement = detail::getElement(sdfElement, "world");
  ASSERT_TRUE(worldElement);
  const auto modelElement = detail::getElement(worldElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  EXPECT_TRUE(detail::hasElement(linkElement, "mass"));
  EXPECT_FALSE(detail::hasElement(linkElement, "does_not_exist"));

  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "mass"), 1.5);

  const Eigen::Vector3d grid
      = detail::getValueVector3i(linkElement, "grid").cast<double>();
  EXPECT_EQ(grid.x(), 1);
  EXPECT_EQ(grid.y(), 2);
  EXPECT_EQ(grid.z(), 3);

  const Eigen::Vector3d gravity
      = detail::getValueVector3d(worldElement, "gravity");
  EXPECT_DOUBLE_EQ(gravity.x(), -1.0);
  EXPECT_DOUBLE_EQ(gravity.y(), 2.0);
  EXPECT_DOUBLE_EQ(gravity.z(), -9.3);

  const Eigen::Isometry3d pose
      = detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "pose");
  EXPECT_NEAR(pose.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(pose.translation().y(), 2.0, 1e-12);
  EXPECT_NEAR(pose.translation().z(), 3.0, 1e-12);
}

TEST(SdfDetailHelpers, MissingInputsReturnDefaults)
{
  const sdf::ElementPtr linkElement = loadLinkElement("");
  ASSERT_TRUE(linkElement);

  EXPECT_FALSE(detail::hasElement(nullptr, "link"));
  EXPECT_FALSE(detail::hasElement(linkElement, ""));
  EXPECT_EQ(detail::getElement(nullptr, "link"), nullptr);
  EXPECT_EQ(detail::getElement(linkElement, ""), nullptr);
  EXPECT_EQ(detail::getValueUInt(linkElement, "missing"), 0u);
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "missing"), 0.0);
  EXPECT_TRUE(
      detail::getValueVector3d(linkElement, "missing")
          .isApprox(Eigen::Vector3d::Zero()));
  EXPECT_EQ(
      detail::getValueVector3i(linkElement, "missing"),
      Eigen::Vector3i::Zero());
  EXPECT_TRUE(
      detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "missing")
          .isApprox(Eigen::Isometry3d::Identity()));
}

TEST(SdfDetailHelpers, ParsesExtensionValuesWithSdformatTypedParams)
{
  const sdf::ElementPtr linkElement = loadLinkElement(R"(
    <count>7</count>
    <mass>2.5</mass>
    <custom_vector3>1 2 3</custom_vector3>
    <custom_pose>1 2 3 0 0 1.57079632679</custom_pose>
  )");
  ASSERT_TRUE(linkElement);

  EXPECT_EQ(detail::getValueUInt(linkElement, "count"), 7u);
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "mass"), 2.5);

  const Eigen::Vector3d vec3
      = detail::getValueVector3d(linkElement, "custom_vector3");
  EXPECT_TRUE(vec3.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  const Eigen::Isometry3d pose
      = detail::getValueIsometry3dWithExtrinsicRotation(
          linkElement, "custom_pose");
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(SdfDetailHelpers, MalformedScalarVectorAndPoseValuesReturnDefaults)
{
  const sdf::ElementPtr linkElement = loadLinkElement(R"(
    <bad_uint>not_unsigned</bad_uint>
    <bad_double>not_double</bad_double>
    <empty_vector3></empty_vector3>
    <short_vector3>1 2</short_vector3>
    <empty_vector3i></empty_vector3i>
    <short_vector3i>1 2</short_vector3i>
    <empty_pose></empty_pose>
    <short_pose>1 2 3 0.1 0.2</short_pose>
  )");
  ASSERT_TRUE(linkElement);

  EXPECT_EQ(detail::getValueUInt(linkElement, "bad_uint"), 0u);
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "bad_double"), 0.0);

  EXPECT_TRUE(
      detail::getValueVector3d(linkElement, "empty_vector3")
          .isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(
      detail::getValueVector3d(linkElement, "short_vector3")
          .isApprox(Eigen::Vector3d::Zero()));
  EXPECT_EQ(
      detail::getValueVector3i(linkElement, "empty_vector3i"),
      Eigen::Vector3i::Zero());
  EXPECT_EQ(
      detail::getValueVector3i(linkElement, "short_vector3i"),
      Eigen::Vector3i::Zero());
  EXPECT_TRUE(
      detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "empty_pose")
          .isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(
      detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "short_pose")
          .isApprox(Eigen::Isometry3d::Identity()));
}
