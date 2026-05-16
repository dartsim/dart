// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/mesh_material.hpp>

#include <gtest/gtest.h>

//==============================================================================
TEST(MeshMaterialTest, DefaultsUseGltfMetallicRoughnessFactors)
{
  const dart::dynamics::MeshMaterial material;

  EXPECT_FLOAT_EQ(material.metallicFactor, 1.0f);
  EXPECT_FLOAT_EQ(material.roughnessFactor, 1.0f);
}
