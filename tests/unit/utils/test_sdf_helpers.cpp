#include <dart/config.hpp>

#include <dart/utils/sdf/detail/geometry_parsers.hpp>
#include <dart/utils/sdf/detail/sdf_helpers.hpp>

#include <dart/dynamics/sphere_shape.hpp>

#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

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

TEST(SdfDetailHelpers, ReadGeometrySphere)
{
  const std::string xml = R"(
    <sdf version='1.6'>
      <model name='demo'>
        <link name='link'>
          <visual name='visual'>
            <geometry>
              <sphere>
                <radius>0.25</radius>
              </sphere>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);

  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);
  const auto visualElement = detail::getElement(linkElement, "visual");
  ASSERT_TRUE(visualElement);
  const auto geometryElement = detail::getElement(visualElement, "geometry");
  ASSERT_TRUE(geometryElement);

  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();
  const auto shape = detail::readGeometryShape(
      geometryElement, dart::common::Uri("file://test"), retriever);
  ASSERT_TRUE(shape);

  auto sphere = std::dynamic_pointer_cast<dart::dynamics::SphereShape>(shape);
  ASSERT_TRUE(sphere);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), 0.25);
}

TEST(SdfDetailHelpers, ParsesAttributesScalarsAndEnumerators)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <world name='default'>
        <gravity>-1.0 2.0 -9.3</gravity>
        <model name='demo_model'>
          <link name='demo_link'>
            <pose>1 2 3 0 0 1.57079632679</pose>
            <self_collide>true</self_collide>
            <mass>1.5</mass>
            <grid>1 2 3</grid>
        <custom_array>
          4 5 6 7
        </custom_array>
            <plane>0.3 0.4</plane>
            <unknown_vector>0.1 0.2 0.3</unknown_vector>
            <visual name='visual_one'>
              <material>
                <diffuse>0.2 0.4 0.6 0.8</diffuse>
              </material>
            </visual>
            <visual name='visual_two'>
              <pose>0 0 0 0 0 0</pose>
            </visual>
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

  EXPECT_EQ(detail::getAttributeString(linkElement, "name"), "demo_link");
  EXPECT_EQ(detail::getAttributeString(linkElement, "missing"), "");
  EXPECT_TRUE(detail::hasElement(linkElement, "visual"));
  EXPECT_FALSE(detail::hasElement(linkElement, "does_not_exist"));

  EXPECT_TRUE(detail::getValueBool(linkElement, "self_collide"));
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "mass"), 1.5);

  const Eigen::Vector3d grid
      = detail::getValueVector3i(linkElement, "grid").cast<double>();
  EXPECT_EQ(grid.x(), 1);
  EXPECT_EQ(grid.y(), 2);
  EXPECT_EQ(grid.z(), 3);

  const Eigen::VectorXd array
      = detail::getValueVectorXd(linkElement, "custom_array");
  ASSERT_EQ(array.size(), 4);
  EXPECT_DOUBLE_EQ(array[0], 4);
  EXPECT_DOUBLE_EQ(array[3], 7);

  const Eigen::Vector2d plane = detail::getValueVector2d(linkElement, "plane");
  EXPECT_DOUBLE_EQ(plane.x(), 0.3);
  EXPECT_DOUBLE_EQ(plane.y(), 0.4);

  const Eigen::Vector3d gravity
      = detail::getValueVector3d(worldElement, "gravity");
  EXPECT_DOUBLE_EQ(gravity.x(), -1.0);
  EXPECT_DOUBLE_EQ(gravity.y(), 2.0);
  EXPECT_DOUBLE_EQ(gravity.z(), -9.3);

  const Eigen::VectorXd unknown
      = detail::getValueVectorXd(linkElement, "unknown_vector");
  ASSERT_EQ(unknown.size(), 3);

  const Eigen::Isometry3d pose
      = detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "pose");
  EXPECT_NEAR(pose.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(pose.translation().y(), 2.0, 1e-12);
  EXPECT_NEAR(pose.translation().z(), 3.0, 1e-12);

  detail::ElementEnumerator enumerator(linkElement, "visual");
  std::vector<std::string> names;
  while (enumerator.next()) {
    names.emplace_back(detail::getAttributeString(enumerator.get(), "name"));
  }

  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "visual_one");
  EXPECT_EQ(names[1], "visual_two");

  const auto visualOne = detail::getElement(linkElement, "visual");
  ASSERT_TRUE(visualOne);
  const auto material = detail::getElement(visualOne, "material");
  ASSERT_TRUE(material);

  const Eigen::VectorXd color = detail::getValueVectorXd(material, "diffuse");
  ASSERT_EQ(color.size(), 4);
  EXPECT_DOUBLE_EQ(color[0], 0.2);
  EXPECT_DOUBLE_EQ(color[3], 0.8);
}

TEST(SdfDetailHelpers, SanitizesFloatingPointVectors)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <visual name='visual'>
            <material>
              <diffuse>0.200000003 0.400000006 0.600000024 0.800000012</diffuse>
            </material>
          </visual>
          <unknown_vector>0.200000003 0.400000006 0.600000024</unknown_vector>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);

  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const auto visualElement = detail::getElement(linkElement, "visual");
  ASSERT_TRUE(visualElement);
  const auto materialElement = detail::getElement(visualElement, "material");
  ASSERT_TRUE(materialElement);

  const Eigen::VectorXd color
      = detail::getValueVectorXd(materialElement, "diffuse");
  ASSERT_EQ(color.size(), 4);
  EXPECT_DOUBLE_EQ(color[0], 0.2);
  EXPECT_DOUBLE_EQ(color[1], 0.4);
  EXPECT_DOUBLE_EQ(color[2], 0.6);
  EXPECT_DOUBLE_EQ(color[3], 0.8);

  const Eigen::VectorXd vec
      = detail::getValueVectorXd(linkElement, "unknown_vector");
  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 0.2);
  EXPECT_DOUBLE_EQ(vec[1], 0.4);
  EXPECT_DOUBLE_EQ(vec[2], 0.6);
}

TEST(SdfDetailHelpers, StringNormalizationHelpers)
{
  EXPECT_EQ(detail::toLowerCopy("  AbC-XYZ "), "  abc-xyz ");
  EXPECT_EQ(detail::trimCopy("\t \n"), "");
  EXPECT_EQ(detail::trimCopy("  text\r\n"), "text");
}

TEST(SdfDetailHelpers, ElementTextNullReturnsEmpty)
{
  EXPECT_TRUE(detail::getElementText(nullptr).empty());
}

TEST(SdfDetailHelpers, Vector2dInsufficientValuesReturnZero)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <custom_vector2>0.5</custom_vector2>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::Vector2d vec
      = detail::getValueVector2d(linkElement, "custom_vector2");
  EXPECT_TRUE(vec.isApprox(Eigen::Vector2d::Zero()));
}

TEST(SdfDetailHelpers, Vector3iInsufficientValuesReturnZero)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <custom_vector3i>1 2</custom_vector3i>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::Vector3i vec
      = detail::getValueVector3i(linkElement, "custom_vector3i");
  EXPECT_EQ(vec, Eigen::Vector3i::Zero());
}

TEST(SdfDetailHelpers, VectorXdPreservesUnroundedValues)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <custom_vector>0.123456789 0.987654321</custom_vector>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::VectorXd vec
      = detail::getValueVectorXd(linkElement, "custom_vector");
  ASSERT_EQ(vec.size(), 2);
  EXPECT_DOUBLE_EQ(vec[0], 0.123456789);
  EXPECT_DOUBLE_EQ(vec[1], 0.987654321);
}

TEST(SdfDetailHelpers, Vector3dInsufficientValuesReturnZero)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <custom_vector3>1 2</custom_vector3>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::Vector3d vec
      = detail::getValueVector3d(linkElement, "custom_vector3");
  EXPECT_TRUE(vec.isApprox(Eigen::Vector3d::Zero()));
}

TEST(SdfDetailHelpers, ExtrinsicRotationInvalidSizeReturnsIdentity)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <custom_pose>1 2 3 0.1 0.2</custom_pose>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::Isometry3d pose
      = detail::getValueIsometry3dWithExtrinsicRotation(
          linkElement, "custom_pose");
  EXPECT_TRUE(pose.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(SdfDetailHelpers, Vector3dParsesFromTextArray)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <custom_vector3>1 2 3</custom_vector3>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::Vector3d vec
      = detail::getValueVector3d(linkElement, "custom_vector3");
  EXPECT_TRUE(vec.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(SdfDetailHelpers, PoseParamParsesToIsometry)
{
  const std::string xml = R"(
    <sdf version='1.9'>
      <model name='demo'>
        <link name='demo_link'>
          <pose>1 2 3 0 0 1.57079632679</pose>
        </link>
      </model>
    </sdf>)";

  const sdf::ElementPtr sdfElement = loadElement(xml);
  ASSERT_TRUE(sdfElement);
  const auto modelElement = detail::getElement(sdfElement, "model");
  ASSERT_TRUE(modelElement);
  const auto linkElement = detail::getElement(modelElement, "link");
  ASSERT_TRUE(linkElement);

  const Eigen::Isometry3d pose
      = detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "pose");
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}
