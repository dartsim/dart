#include <dart/config.hpp>

#include <dart/utils/sdf/detail/geometry_parsers.hpp>
#include <dart/utils/sdf/detail/sdf_helpers.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
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
      loadGeometryElement(R"(
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
      loadGeometryElement(R"(
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
      loadGeometryElement(R"(
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
      loadGeometryElement(R"(
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

  EXPECT_EQ(detail::readGeometryShape(nullptr, baseUri, retriever), nullptr);
  EXPECT_EQ(
      detail::readGeometryShape(
          loadGeometryElement("<mesh/>"), baseUri, retriever),
      nullptr);
  EXPECT_EQ(
      detail::readGeometryShape(
          loadGeometryElement("<mesh><uri>missing.obj</uri></mesh>"),
          baseUri,
          retriever),
      nullptr);
  EXPECT_EQ(
      detail::readGeometryShape(loadGeometryElement(""), baseUri, retriever),
      nullptr);
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

TEST(SdfDetailHelpers, MissingInputsReturnDefaults)
{
  const sdf::ElementPtr linkElement = loadLinkElement("");
  ASSERT_TRUE(linkElement);

  EXPECT_FALSE(detail::hasElement(nullptr, "link"));
  EXPECT_FALSE(detail::hasElement(linkElement, ""));
  EXPECT_EQ(detail::getElement(nullptr, "link"), nullptr);
  EXPECT_EQ(detail::getElement(linkElement, ""), nullptr);
  EXPECT_TRUE(detail::getChildElementText(nullptr, "link").empty());
  EXPECT_TRUE(detail::getChildElementText(linkElement, "").empty());

  EXPECT_FALSE(detail::hasAttribute(nullptr, "name"));
  EXPECT_FALSE(detail::hasAttribute(linkElement, ""));
  EXPECT_EQ(detail::getAttributeParam(nullptr, "name"), nullptr);
  EXPECT_EQ(detail::getAttributeParam(linkElement, ""), nullptr);

  EXPECT_EQ(detail::getChildValueParam(nullptr, "mass"), nullptr);
  EXPECT_EQ(detail::getChildValueParam(linkElement, ""), nullptr);
  EXPECT_TRUE(detail::getValueString(linkElement, "missing").empty());
  EXPECT_FALSE(detail::getValueBool(linkElement, "missing"));
  EXPECT_EQ(detail::getValueUInt(linkElement, "missing"), 0u);
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "missing"), 0.0);
  EXPECT_TRUE(
      detail::getValueVector2d(linkElement, "missing")
          .isApprox(Eigen::Vector2d::Zero()));
  EXPECT_TRUE(
      detail::getValueVector3d(linkElement, "missing")
          .isApprox(Eigen::Vector3d::Zero()));
  EXPECT_EQ(
      detail::getValueVector3i(linkElement, "missing"),
      Eigen::Vector3i::Zero());
  EXPECT_EQ(detail::getValueVectorXd(linkElement, "missing").size(), 0);
  EXPECT_TRUE(
      detail::getValueIsometry3dWithExtrinsicRotation(linkElement, "missing")
          .isApprox(Eigen::Isometry3d::Identity()));

  detail::ElementEnumerator nullEnumerator(nullptr, "visual");
  EXPECT_FALSE(nullEnumerator.next());
  EXPECT_EQ(nullEnumerator.get(), nullptr);
}

TEST(SdfDetailHelpers, ConvertsGazeboMathTypes)
{
  const Eigen::Vector3d vec3
      = detail::toEigen(gz::math::Vector3d(1.0, 2.0, 3.0));
  EXPECT_TRUE(vec3.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  const Eigen::Vector2d vec2 = detail::toEigen(gz::math::Vector2d(4.0, 5.0));
  EXPECT_TRUE(vec2.isApprox(Eigen::Vector2d(4.0, 5.0)));

  const Eigen::Vector3i vec3i = detail::toEigen(gz::math::Vector3i(6, 7, 8));
  EXPECT_EQ(vec3i, Eigen::Vector3i(6, 7, 8));

  const Eigen::VectorXd color
      = detail::colorToVector(gz::math::Color(0.1f, 0.2f, 0.3f, 0.4f));
  ASSERT_EQ(color.size(), 4);
  EXPECT_FLOAT_EQ(static_cast<float>(color[0]), 0.1f);
  EXPECT_FLOAT_EQ(static_cast<float>(color[1]), 0.2f);
  EXPECT_FLOAT_EQ(static_cast<float>(color[2]), 0.3f);
  EXPECT_FLOAT_EQ(static_cast<float>(color[3]), 0.4f);

  const Eigen::Isometry3d pose = detail::poseToIsometry(
      gz::math::Pose3d(
          gz::math::Vector3d(9.0, 10.0, 11.0),
          gz::math::Quaterniond(0.0, 0.0, 0.5)));
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(9.0, 10.0, 11.0)));
  EXPECT_NEAR(pose.linear().determinant(), 1.0, 1e-12);
}

TEST(SdfDetailHelpers, ParsesScalarVectorAndPoseFallbackText)
{
  const sdf::ParamPtr detachedParam
      = std::make_shared<sdf::Param>("detached", "string", "fallback", false);
  EXPECT_EQ(
      detail::getValueText(nullptr, "detached", detachedParam), "fallback");
  EXPECT_TRUE(detail::getValueText(nullptr, "detached", nullptr).empty());

  const sdf::ElementPtr linkElement = loadLinkElement(R"(
    <label>demo_label</label>
    <enabled>true</enabled>
    <count>7</count>
    <mass>2.5</mass>
    <custom_vector2>1.25 2.5</custom_vector2>
    <custom_vector3>1 2 3</custom_vector3>
    <custom_pose>1 2 3 0 0 1.57079632679</custom_pose>
  )");
  ASSERT_TRUE(linkElement);

  EXPECT_EQ(detail::getValueString(linkElement, "label"), "demo_label");
  EXPECT_EQ(detail::getValueString(linkElement, "count"), "7");
  EXPECT_TRUE(detail::getValueBool(linkElement, "enabled"));
  EXPECT_EQ(detail::getValueUInt(linkElement, "count"), 7u);
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "mass"), 2.5);

  const Eigen::Vector2d vec2
      = detail::getValueVector2d(linkElement, "custom_vector2");
  EXPECT_TRUE(vec2.isApprox(Eigen::Vector2d(1.25, 2.5)));

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
    <bad_bool>maybe</bad_bool>
    <bad_uint>not_unsigned</bad_uint>
    <bad_double>not_double</bad_double>
    <empty_vector2></empty_vector2>
    <short_vector2>0.5</short_vector2>
    <empty_vector3></empty_vector3>
    <short_vector3>1 2</short_vector3>
    <empty_vector3i></empty_vector3i>
    <short_vector3i>1 2</short_vector3i>
    <empty_pose></empty_pose>
    <short_pose>1 2 3 0.1 0.2</short_pose>
  )");
  ASSERT_TRUE(linkElement);

  EXPECT_FALSE(detail::getValueBool(linkElement, "bad_bool"));
  EXPECT_EQ(detail::getValueUInt(linkElement, "bad_uint"), 0u);
  EXPECT_DOUBLE_EQ(detail::getValueDouble(linkElement, "bad_double"), 0.0);

  EXPECT_TRUE(
      detail::getValueVector2d(linkElement, "empty_vector2")
          .isApprox(Eigen::Vector2d::Zero()));
  EXPECT_TRUE(
      detail::getValueVector2d(linkElement, "short_vector2")
          .isApprox(Eigen::Vector2d::Zero()));
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

TEST(SdfDetailHelpers, ElementTextNullReturnsEmpty)
{
  EXPECT_TRUE(detail::getElementText(nullptr).empty());
}
