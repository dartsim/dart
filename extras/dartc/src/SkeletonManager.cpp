#include "SkeletonManager.hpp"

//==============================================================================
std::unordered_map<dart::dynamics::Skeleton*, dart::dynamics::SkeletonPtr>
    SkeletonManager::mMap;

//==============================================================================
dart::dynamics::Skeleton* SkeletonManager::create(const std::string& name)
{
  auto skel = dart::dynamics::Skeleton::create(name);
  mMap[skel.get()] = skel;
  return skel.get();
}

//==============================================================================
void SkeletonManager::destroy(dart::dynamics::Skeleton* skel)
{
  mMap.erase(skel);
}

//==============================================================================
void SkeletonManager::destroy(void* skel)
{
  destroy(static_cast<dart::dynamics::Skeleton*>(skel));
}

//==============================================================================
dart::dynamics::SkeletonPtr SkeletonManager::asShared(
    dart::dynamics::Skeleton* skel)
{
  return mMap[skel];
}

//==============================================================================
dart::dynamics::SkeletonPtr SkeletonManager::asShared(void* skel)
{
  return mMap[static_cast<dart::dynamics::Skeleton*>(skel)];
}
