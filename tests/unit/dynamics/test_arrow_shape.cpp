// Copyright (c) 2011, The DART development contributors

#include <dart/all.hpp>

#include <assimp/cimport.h>
#include <gtest/gtest.h>

namespace dart {
namespace utils {
} // namespace utils
} // namespace dart

using namespace dart::dynamics;

namespace {

class DartSampleRetriever final : public dart::common::ResourceRetriever
{
public:
  explicit DartSampleRetriever(std::string dataRoot)
    : mDataRoot(std::move(dataRoot)),
      mDelegate(std::make_shared<dart::common::LocalResourceRetriever>())
  {
  }

  bool exists(const dart::common::Uri& uri) override
  {
    return mDelegate->exists(toLocalUri(uri));
  }

  dart::common::ResourcePtr retrieve(const dart::common::Uri& uri) override
  {
    return mDelegate->retrieve(toLocalUri(uri));
  }

  std::string getFilePath(const dart::common::Uri& uri) override
  {
    DART_SUPPRESS_DEPRECATED_BEGIN
    return mDelegate->getFilePath(toLocalUri(uri));
    DART_SUPPRESS_DEPRECATED_END
  }

private:
  dart::common::Uri toLocalUri(const dart::common::Uri& uri) const
  {
    if (isSampleUri(uri)) {
      dart::common::filesystem::path path = mDataRoot;
      std::string relative = uri.getPath();
      if (!relative.empty() && relative.front() == '/') {
        relative.erase(0, 1);
      }
      path /= relative;
      return dart::common::Uri::createFromPath(path.string());
    }

    return uri;
  }

  bool isSampleUri(const dart::common::Uri& uri) const
  {
    return uri.mScheme.get_value_or("") == "dart" && uri.mAuthority
           && uri.mAuthority.get() == "sample";
  }

  std::string mDataRoot;
  dart::common::LocalResourceRetrieverPtr mDelegate;
};

dart::common::filesystem::path getSampleDataRoot()
{
  dart::common::filesystem::path here(__FILE__);
  return here.parent_path().parent_path().parent_path().parent_path() / "data";
}

class ExposedArrowShape final : public ArrowShape
{
public:
  using ArrowShape::ArrowShape;
  using MeshShape::convertToAssimpMesh;
};

} // namespace

//==============================================================================
TEST(ArrowShapeTest, DefaultConstructor)
{
  ArrowShape arrow;

  EXPECT_EQ(arrow.getType(), MeshShape::getStaticType());
}

//==============================================================================
TEST(ArrowShapeTest, ConstructorWithPositions)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);
  Eigen::Vector4d color(1.0, 0.0, 0.0, 1.0);

  ArrowShape arrow(tail, head, ArrowShape::Properties(), color);

  EXPECT_TRUE(arrow.getTail().isApprox(tail));
  EXPECT_TRUE(arrow.getHead().isApprox(head));
}

//==============================================================================
TEST(ArrowShapeTest, SetPositions)
{
  ArrowShape arrow;

  Eigen::Vector3d newTail(1.0, 2.0, 3.0);
  Eigen::Vector3d newHead(4.0, 5.0, 6.0);
  arrow.setPositions(newTail, newHead);

  EXPECT_TRUE(arrow.getTail().isApprox(newTail));
  EXPECT_TRUE(arrow.getHead().isApprox(newHead));
}

//==============================================================================
TEST(ArrowShapeTest, Properties)
{
  ArrowShape::Properties props(0.02, 2.0, 0.25, 0.01, 0.1, false);

  EXPECT_DOUBLE_EQ(props.mRadius, 0.02);
  EXPECT_DOUBLE_EQ(props.mHeadRadiusScale, 2.0);
  EXPECT_DOUBLE_EQ(props.mHeadLengthScale, 0.25);
  EXPECT_DOUBLE_EQ(props.mMinHeadLength, 0.01);
  EXPECT_DOUBLE_EQ(props.mMaxHeadLength, 0.1);
  EXPECT_FALSE(props.mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, SetProperties)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);
  ArrowShape arrow(tail, head);

  ArrowShape::Properties newProps(0.05, 3.0, 0.3, 0.02, 0.2, true);
  arrow.setProperties(newProps);

  const auto& props = arrow.getProperties();
  EXPECT_DOUBLE_EQ(props.mRadius, 0.05);
  EXPECT_DOUBLE_EQ(props.mHeadRadiusScale, 3.0);
  EXPECT_TRUE(props.mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, DoubleArrow)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(2.0, 0.0, 0.0);
  ArrowShape::Properties props;
  props.mDoubleArrow = true;

  ArrowShape arrow(tail, head, props);

  EXPECT_TRUE(arrow.getProperties().mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, Clone)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 2.0, 3.0);
  ArrowShape::Properties props(0.03, 2.5, 0.2, 0.01, 0.15, false);
  Eigen::Vector4d color(0.5, 0.5, 0.5, 1.0);

  auto original = std::make_shared<ArrowShape>(tail, head, props, color);
  auto cloned = original->clone();
  auto clonedArrow = std::dynamic_pointer_cast<ArrowShape>(cloned);

  ASSERT_NE(clonedArrow, nullptr);
  EXPECT_TRUE(clonedArrow->getTail().isApprox(tail));
  EXPECT_TRUE(clonedArrow->getHead().isApprox(head));
  EXPECT_NE(clonedArrow.get(), original.get());
}

//==============================================================================
TEST(ArrowShapeTest, ColorUpdate)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);
  Eigen::Vector4d initialColor(1.0, 0.0, 0.0, 1.0);

  ArrowShape arrow(tail, head, ArrowShape::Properties(), initialColor);

  Eigen::Vector4d newColor(0.0, 1.0, 0.0, 0.5);
  arrow.notifyColorUpdated(newColor);

  EXPECT_FALSE(arrow.getMaterials().empty());
}

//==============================================================================
TEST(ArrowShapeTest, PropertiesClampedToValidRange)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);

  ArrowShape::Properties props;
  props.mHeadLengthScale = 1.5;
  props.mMinHeadLength = -0.1;
  props.mMaxHeadLength = -0.2;
  props.mHeadRadiusScale = 0.5;

  ArrowShape arrow(tail, head, props);

  const auto& clampedProps = arrow.getProperties();
  EXPECT_LE(clampedProps.mHeadLengthScale, 1.0);
  EXPECT_GE(clampedProps.mHeadLengthScale, 0.0);
  EXPECT_GE(clampedProps.mMinHeadLength, 0.0);
  EXPECT_GE(clampedProps.mMaxHeadLength, 0.0);
  EXPECT_GE(clampedProps.mHeadRadiusScale, 1.0);
}

//==============================================================================
TEST(ArrowShapeTest, SetPropertiesClampsValues)
{
  ArrowShape arrow(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());

  ArrowShape::Properties props;
  props.mHeadLengthScale = -0.4;
  props.mMinHeadLength = -1.0;
  props.mMaxHeadLength = -2.0;
  props.mHeadRadiusScale = 0.2;
  props.mDoubleArrow = true;

  arrow.setProperties(props);

  const auto& clamped = arrow.getProperties();
  EXPECT_GE(clamped.mHeadLengthScale, 0.0);
  EXPECT_GE(clamped.mMinHeadLength, 0.0);
  EXPECT_GE(clamped.mMaxHeadLength, 0.0);
  EXPECT_GE(clamped.mHeadRadiusScale, 1.0);
  EXPECT_TRUE(clamped.mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, SetPositionsVeryShortArrow)
{
  ArrowShape arrow(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());

  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(1e-8, 0.0, 0.0);
  arrow.setPositions(tail, head);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_GT(mesh->getVertices().size(), 0u);
  const auto extents = arrow.getBoundingBox().computeFullExtents();
  EXPECT_TRUE(extents.array().isFinite().all());
}

//==============================================================================
TEST(ArrowShapeTest, SingleArrowMeshGeneration)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(0.0, 0.0, 1.0);
  const std::size_t resolution = 6;

  ArrowShape arrow(
      tail,
      head,
      ArrowShape::Properties(),
      Eigen::Vector4d::Ones(),
      resolution);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getVertices().size(), 4 * resolution + 1);
  EXPECT_EQ(mesh->getTriangles().size(), 5 * resolution);
}

//==============================================================================
TEST(ArrowShapeTest, DoubleArrowMeshGeneration)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(0.0, 0.0, 1.0);
  const std::size_t resolution = 6;

  ArrowShape::Properties props;
  props.mDoubleArrow = true;

  ArrowShape arrow(tail, head, props, Eigen::Vector4d::Ones(), resolution);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getVertices().size(), 6 * resolution + 2);
  EXPECT_EQ(mesh->getTriangles().size(), 8 * resolution);
}

//==============================================================================
TEST(ArrowShapeTest, DoubleArrowGeneratesFullMesh)
{
  ArrowShape::Properties props;
  props.mDoubleArrow = true;
  props.mRadius = 0.05;
  props.mHeadLengthScale = 0.3;
  props.mHeadRadiusScale = 2.0;

  ArrowShape arrow(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 2.0), props);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_GT(mesh->getTriangles().size(), 20u);
  EXPECT_GT(mesh->getVertices().size(), 10u);

  double maxRadius = 0.0;
  for (const auto& vertex : mesh->getVertices()) {
    maxRadius = std::max(maxRadius, vertex.head<2>().norm());
  }
  EXPECT_GT(maxRadius, props.mRadius * 1.5);
  EXPECT_FALSE(arrow.getMaterials().empty());
}

//==============================================================================
TEST(ArrowShapeTest, MeshConversions)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(0.0, 0.0, 1.0);
  const std::size_t resolution = 6;

  ArrowShape::Properties props;
  props.mDoubleArrow = true;

  ExposedArrowShape arrow(
      tail, head, props, Eigen::Vector4d::Ones(), resolution);

  const auto triMesh = arrow.getTriMesh();
  ASSERT_NE(triMesh, nullptr);

  const auto polygonMesh = arrow.getPolygonMesh();
  ASSERT_NE(polygonMesh, nullptr);
  EXPECT_TRUE(polygonMesh->hasVertices());

  const aiScene* scene = arrow.convertToAssimpMesh();
  ASSERT_NE(scene, nullptr);
  aiReleaseImport(const_cast<aiScene*>(scene));
}

//==============================================================================
TEST(ArrowShapeTest, ReconfigureReleasesCachedScene)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(1.0, 0.0, 0.0);
  ArrowShape arrow(tail, head);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = arrow.getMesh();
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_NE(scene, nullptr);

  const Eigen::Vector3d newTail(0.0, 0.0, 0.0);
  const Eigen::Vector3d newHead(0.0, 1.0, 0.0);
  arrow.configureArrow(newTail, newHead, arrow.getProperties());
}

//==============================================================================
TEST(ArrowShapeTest, AntiparallelDirection)
{
  const Eigen::Vector3d tail(0.0, 0.0, 1.0);
  const Eigen::Vector3d head(0.0, 0.0, 0.0);

  ArrowShape arrow(tail, head);

  EXPECT_NE(arrow.getTriMesh(), nullptr);
}

//==============================================================================
TEST(ArrowShapeTest, ZeroLengthArrowDoesNotCrash)
{
  const Eigen::Vector3d tail(1.0, 2.0, 3.0);
  const Eigen::Vector3d head = tail;

  ArrowShape arrow(tail, head);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_GT(mesh->getVertices().size(), 0u);
}

//==============================================================================
TEST(ArrowShapeTest, DoubleArrowAntiparallelMesh)
{
  const Eigen::Vector3d tail(0.0, 0.0, 2.0);
  const Eigen::Vector3d head(0.0, 0.0, 0.0);
  const std::size_t resolution = 5;

  ArrowShape::Properties props;
  props.mDoubleArrow = true;

  ArrowShape arrow(tail, head, props, Eigen::Vector4d::Ones(), resolution);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getVertices().size(), 6 * resolution + 2);
  EXPECT_EQ(mesh->getTriangles().size(), 8 * resolution);
}

//==============================================================================
TEST(ArrowShapeTest, MeshShapeTriMeshConstructorAndConstGetters)
{
  auto triMesh = std::make_shared<dart::math::TriMesh<double>>();
  triMesh->addVertex(0.0, 0.0, 0.0);
  triMesh->addVertex(1.0, 0.0, 0.0);
  triMesh->addVertex(0.0, 1.0, 0.0);
  triMesh->addTriangle(0, 1, 2);

  dart::dynamics::MeshShape shape(Eigen::Vector3d::Ones(), triMesh);
  shape.setScale(Eigen::Vector3d(1.0, 2.0, 3.0));
  shape.setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
  shape.setAlphaMode(dart::dynamics::MeshShape::AUTO);
  shape.setColorIndex(2);
  shape.setDisplayList(7);

  const dart::dynamics::MeshShape* constShape = &shape;
  EXPECT_TRUE(constShape->getScale().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ(constShape->getColorMode(), dart::dynamics::MeshShape::SHAPE_COLOR);
  EXPECT_EQ(constShape->getAlphaMode(), dart::dynamics::MeshShape::AUTO);
  EXPECT_EQ(constShape->getColorIndex(), 2);
  EXPECT_EQ(constShape->getDisplayList(), 7);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = constShape->getMesh();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);
  EXPECT_TRUE(constShape->getBoundingBox().getMin().array().isFinite().all());
  EXPECT_TRUE(constShape->getBoundingBox().getMax().array().isFinite().all());
}

//==============================================================================
TEST(ArrowShapeTest, MeshShapeLoadsDartSampleUri)
{
  const auto dataRoot = getSampleDataRoot();
  auto retriever = std::make_shared<DartSampleRetriever>(dataRoot.string());
  const std::string uriString = "dart://sample/obj/BoxSmall.obj";

  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene
      = dart::dynamics::MeshShape::loadMesh(uriString, retriever);
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_NE(scene, nullptr);

  const dart::common::Uri uri
      = dart::common::Uri::createFromStringOrPath(uriString);
  DART_SUPPRESS_DEPRECATED_BEGIN
  dart::dynamics::MeshShape shape(
      Eigen::Vector3d::Ones(), scene, uri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shape.getMeshUri(), uriString);
  EXPECT_FALSE(shape.getMeshPath().empty());
}
