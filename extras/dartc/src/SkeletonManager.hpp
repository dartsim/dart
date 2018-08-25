
#include <unordered_map>
#include <dart/dynamics/Skeleton.hpp>

struct SkeletonManager final
{
  static dart::dynamics::Skeleton* create(const std::string& name);
  // TODO(JS): Make as a template

  static void destroy(dart::dynamics::Skeleton* skel);

  static void destroy(void* skel);

  static dart::dynamics::SkeletonPtr asShared(dart::dynamics::Skeleton* skel);

  static dart::dynamics::SkeletonPtr asShared(void* skel);

  static std::unordered_map<dart::dynamics::Skeleton*,
                            dart::dynamics::SkeletonPtr>
      mMap;
};
