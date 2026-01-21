#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/referential_skeleton.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <utility>

namespace dynamics = dart::dynamics;

static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getNumBodyNodes()),
    "Skeleton::getNumBodyNodes should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getNumRigidBodyNodes()),
    "Skeleton::getNumRigidBodyNodes should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getNumSoftBodyNodes()),
    "Skeleton::getNumSoftBodyNodes should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getNumTrees()),
    "Skeleton::getNumTrees should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getTimeStep()),
    "Skeleton::getTimeStep should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getGravity()),
    "Skeleton::getGravity should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::Skeleton&>().getMass()),
    "Skeleton::getMass should be noexcept");

static_assert(
    noexcept(std::declval<const dynamics::BodyNode&>().getGravityMode()),
    "BodyNode::getGravityMode should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::BodyNode&>().isCollidable()),
    "BodyNode::isCollidable should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::BodyNode&>().getMass()),
    "BodyNode::getMass should be noexcept");

static_assert(
    noexcept(
        std::declval<const dynamics::ReferentialSkeleton&>().getNumBodyNodes()),
    "ReferentialSkeleton::getNumBodyNodes should be noexcept");
static_assert(
    noexcept(
        std::declval<const dynamics::ReferentialSkeleton&>().getNumSkeletons()),
    "ReferentialSkeleton::getNumSkeletons should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::ReferentialSkeleton&>().hasSkeleton(
        static_cast<const dynamics::Skeleton*>(nullptr))),
    "ReferentialSkeleton::hasSkeleton should be noexcept");
static_assert(
    noexcept(std::declval<const dynamics::ReferentialSkeleton&>().getMass()),
    "ReferentialSkeleton::getMass should be noexcept");

TEST(Noexcept, AccessorsCompileTime)
{
  SUCCEED();
}
