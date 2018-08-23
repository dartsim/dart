#include "dartc/skeleton.h"

#include <unordered_map>
#include <dart/dynamics/Skeleton.hpp>

#define CAST_SKELETON(name) static_cast<Skeleton*>(name)

#ifdef __cplusplus
extern "C" {
#endif

using namespace dart::dynamics;

static std::unordered_map<Skeleton*, SkeletonPtr> gSkeletonMap;

//==============================================================================
SkeletonId dart_create_skeleton(const char* name)
{
  SkeletonPtr skel = Skeleton::create(name);
  gSkeletonMap[skel.get()] = skel;

  return skel.get();
}

//==============================================================================
void dart_destroy_skeleton(SkeletonId skel)
{
  gSkeletonMap.erase(static_cast<Skeleton*>(skel));
}

//==============================================================================
const char* dart_skeleton_get_name(SkeletonId skel)
{
  assert(skel);
  return CAST_SKELETON(skel)->getName().c_str();
}

//==============================================================================
int dart_skeleton_get_num_dofs(SkeletonId skel)
{
  return static_cast<int>(CAST_SKELETON(skel)->getNumDofs());
}

#ifdef __cplusplus
}
#endif
