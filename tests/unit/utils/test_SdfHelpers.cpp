#include <dart/config.hpp>

#if HAVE_SDFORMAT

  #include <dart/utils/sdf/detail/GeometryParsers.hpp>
  #include <dart/utils/sdf/detail/SdfHelpers.hpp>

  #include <dart/dynamics/SphereShape.hpp>

  #include <dart/common/LocalResourceRetriever.hpp>
  #include <dart/common/Uri.hpp>

  #include <gtest/gtest.h>

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

#endif // HAVE_SDFORMAT
