#include "dartc/simulation.h"

#include <dart/simulation/World.hpp>
#include "SkeletonManager.hpp"

#ifdef __cplusplus
extern "C" {
#endif

using namespace dart::dynamics;
using namespace dart::simulation;

#define CAST_WORLD(name) static_cast<World*>(name)

//==============================================================================
WorldId dart_world_create(const char* name)
{
  World* world = new World(name);
  return world;
}

//==============================================================================
void dart_world_destroy(WorldId world)
{
  delete CAST_WORLD(world);
}

//==============================================================================
const char* dart_world_get_name(WorldId world)
{
  return CAST_WORLD(world)->getName().c_str();
}

//==============================================================================
void dart_world_set_gravity(WorldId world, double x, double y, double z)
{
  CAST_WORLD(world)->setGravity(Eigen::Vector3d(x, y, z));
}

//==============================================================================
const double* dart_world_get_gravity(WorldId world)
{
  return CAST_WORLD(world)->getGravity().data();
}

//==============================================================================
double dart_world_get_gravity_x(WorldId world)
{
  return CAST_WORLD(world)->getGravity().x();
}

//==============================================================================
double dart_world_get_gravity_y(WorldId world)
{
  return CAST_WORLD(world)->getGravity().y();
}

//==============================================================================
double dart_world_get_gravity_z(WorldId world)
{
  return CAST_WORLD(world)->getGravity().z();
}

//==============================================================================
void dart_world_add_skeleton(WorldId world, SkeletonId skeleton)
{
  CAST_WORLD(world)->addSkeleton(SkeletonManager::asShared(skeleton));
}

//==============================================================================
int dart_world_get_num_skeletons(WorldId world)
{
  return static_cast<int>(CAST_WORLD(world)->getNumSkeletons());
}

//==============================================================================
SkeletonId dart_world_get_skeleton(WorldId world, int skeletonIndex)
{
  return CAST_WORLD(world)
      ->getSkeleton(static_cast<std::size_t>(skeletonIndex))
      .get();
}

//==============================================================================
void dart_world_remove_all_skeletons(WorldId world)
{
  CAST_WORLD(world)->removeAllSkeletons();
}

#ifdef __cplusplus
}
#endif
