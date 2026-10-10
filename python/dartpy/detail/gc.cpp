// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"
#include "detail/ik_properties.hpp"
#include "detail/optimizer_properties.hpp"

#include <dart/utils/CompositeResourceRetriever.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/simulation/World.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>

#include <dart/collision/CollisionOption.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ContactInverseDynamics.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Macros.hpp>
#include <dart/common/Uri.hpp>

#include <nanobind/stl/function.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

namespace dartnb {
bool has_gc_wrapper(void* complete);
namespace {
// A control block owns one Python reference, even when several C++ fields copy
// it.
std::unordered_map<const PythonPin*, std::size_t> edge_counts(
    const GcEdges& refs)
{
  std::unordered_map<const PythonPin*, std::size_t> counts;
  for (const auto& edge : refs.edges)
    ++counts[edge.pin];
  return counts;
}

template <class T, class Clear>
void descend(
    const std::shared_ptr<T>& owner,
    GcEdges& edges,
    Clear clear,
    long owned_references = 1)
{
  if (!owner)
    return;
  if (std::get_deleter<PythonPin>(owner)) {
    edges.add(owner, std::move(clear));
  } else if (!has_gc_wrapper(complete_address(owner.get()))) {
    // Shared native state can have owners outside this wrapper's GC graph.
    if (owner.use_count() != owned_references)
      return;
    if (!edges.seen.insert(complete_address(owner.get())).second)
      return;
    edges.native_owners.push_back(owner);
    if constexpr (std::is_same_v<T, dart::common::ResourceRetriever>) {
      if (auto* composite
          = dynamic_cast<dart::utils::CompositeResourceRetriever*>(owner.get()))
        enumerate_gc(*composite, edges);
    } else
      enumerate_gc(*owner, edges);
  }
}
} // namespace

bool gc_owner_is_exclusive(PyObject* self, void* complete)
{
  if (auto owner = native_owner(complete))
    return owner.use_count() == 2; // Wrapper payload and this lookup.
  // shortcut: BodyNodePtr hides aliases, so borrowed proxies stay conservative;
  // upgrade when a GC-visible native ownership graph can prove exclusivity.
  return nb::inst_state(self).second;
}

int GcEdges::traverse(visitproc visit, void* arg) const
{
  auto counts = edge_counts(*this);
  for (const auto& edge : edges) {
    auto& count = counts.at(edge.pin);
    // shortcut: shared native aliases outside this owner stay conservative;
    // upgrade to a GC-visible native ownership graph for cross-owner cycles.
    if (count && edge.owner.use_count() == static_cast<long>(2 * count)) {
      int result = visit(edge.pin->object, arg);
      if (result)
        return result;
      count = 0;
    }
  }
  return 0;
}

void GcEdges::clear()
{
  const auto counts = edge_counts(*this);
  // Snapshot pins before invoking setters: decrefs can destroy other owners.
  std::vector<std::function<void()>> resets;
  for (const auto& edge : edges)
    if (edge.owner.use_count() == static_cast<long>(2 * counts.at(edge.pin)))
      resets.push_back(edge.clear);
  for (const auto& reset : resets)
    reset();
}

void enumerate_gc(dart::optimizer::Problem& owner, GcEdges& edges)
{
  edges.add(owner.getObjective(), [&owner] { owner.clearObjective(); });
  for (std::size_t i = 0; i < owner.getNumEqConstraints(); ++i)
    edges.add(
        owner.getEqConstraint(i), [&owner] { owner.removeAllEqConstraints(); });
  for (std::size_t i = 0; i < owner.getNumIneqConstraints(); ++i)
    edges.add(owner.getIneqConstraint(i), [&owner] {
      owner.removeAllIneqConstraints();
    });
}

void enumerate_gc(dart::optimizer::Solver& owner, GcEdges& edges)
{
  descend(
      owner.getProblem(), edges, [&owner] { owner.setProblem(nullptr); }, 2);
}

void enumerate_problem_gc(
    std::shared_ptr<dart::optimizer::Problem>& problem, GcEdges& edges)
{
  descend(problem, edges, [&problem] { problem.reset(); });
}

void enumerate_retriever_gc(
    std::shared_ptr<dart::common::ResourceRetriever>& retriever, GcEdges& edges)
{
  descend(retriever, edges, [&retriever] { retriever.reset(); });
}

void enumerate_reference_frame_gc(
    std::shared_ptr<dart::dynamics::SimpleFrame>& frame, GcEdges& edges)
{
  descend(frame, edges, [&frame] { frame.reset(); });
}

void enumerate_gc(dart::collision::CollisionOption& owner, GcEdges& edges)
{
  edges.add(owner.collisionFilter, [&owner] { owner.collisionFilter.reset(); });
}

void enumerate_gc(
    dart::utils::CompositeResourceRetriever& owner, GcEdges& edges)
{
  for (const auto& retriever : owner.getDefaultRetrievers())
    descend(retriever, edges, [&owner] { owner.removeAllRetrievers(); });
  for (const auto& entry : owner.getSchemaRetrievers())
    for (const auto& retriever : entry.second)
      descend(retriever, edges, [&owner] { owner.removeAllRetrievers(); });
}

void enumerate_gc(dart::constraint::ConstraintSolver& owner, GcEdges& edges)
{
  for (std::size_t i = 0; i < owner.getNumConstraints(); ++i)
    edges.add(
        owner.getConstraint(i), [&owner] { owner.removeAllConstraints(); });
  for (const auto& skeleton : owner.getSkeletons())
    descend(skeleton, edges, [&owner] { owner.removeAllSkeletons(); });
  enumerate_gc(owner.getCollisionOption(), edges);
  if (auto* boxed
      = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(&owner)) {
    edges.add(boxed->getBoxedLcpSolver(), [boxed] {
      boxed->setBoxedLcpSolver(
          std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
    });
    edges.add(boxed->getSecondaryBoxedLcpSolver(), [boxed] {
      boxed->setSecondaryBoxedLcpSolver(nullptr);
    });
  }
}

void enumerate_gc(dart::utils::DartLoader& owner, GcEdges& edges)
{
  descend(owner.getOptions().mResourceRetriever, edges, [&owner] {
    auto options = owner.getOptions();
    options.mResourceRetriever.reset();
    owner.setOptions(options);
  });
}

void enumerate_gc(
    dart::dynamics::InverseKinematics::ErrorMethod& owner, GcEdges& edges)
{
  if (auto* region
      = dynamic_cast<dart::dynamics::InverseKinematics::TaskSpaceRegion*>(
          &owner)) {
    auto frame = std::const_pointer_cast<dart::dynamics::SimpleFrame>(
        region->getReferenceFrame());
    descend(
        frame, edges, [region] { region->setReferenceFrame(nullptr); }, 2);
  }
}

void enumerate_gc(dart::dynamics::InverseKinematics& owner, GcEdges& edges)
{
  edges.add(owner.getObjective(), [&owner] { owner.setObjective(nullptr); });
  edges.add(owner.getNullSpaceObjective(), [&owner] {
    owner.setNullSpaceObjective(nullptr);
  });
  descend(owner.getSolver(), edges, [&owner] { owner.setSolver(nullptr); });
  descend(owner.getProblem(), edges, [] {});
  auto& error = owner.getErrorMethod();
  if (!has_gc_wrapper(complete_address(&error)))
    enumerate_gc(error, edges);
  edges.add(owner.getTarget(), [&owner] {
    owner.setTarget(std::make_shared<dart::dynamics::SimpleFrame>());
  });
}

void enumerate_gc(dart::dynamics::ShapeFrame& owner, GcEdges& edges)
{
  descend(
      owner.getShape(), edges, [&owner] { owner.setShape(nullptr); }, 2);
}

void enumerate_gc(dart::dynamics::Shape& owner, GcEdges& edges)
{
  // A live native alias must retain its retriever's Python overrides.
  auto native = native_owner(complete_address(&owner));
  if (native && native.use_count() > 2)
    return;
  if (auto* mesh = dynamic_cast<dart::dynamics::MeshShape*>(&owner)) {
    descend(
        mesh->getResourceRetriever(),
        edges,
        [mesh] {
          DART_SUPPRESS_DEPRECATED_BEGIN
          mesh->setMesh(nullptr, dart::common::Uri(), nullptr);
          DART_SUPPRESS_DEPRECATED_END
        },
        2);
  }
}

void enumerate_gc(dart::dynamics::ContactInverseDynamics& owner, GcEdges& edges)
{
  descend(owner.getSkeleton(), edges, [] {});
}

void enumerate_gc(dart::dynamics::BodyNode& owner, GcEdges& edges)
{
  for (std::size_t i = 0; i < owner.getNumShapeNodes(); ++i) {
    auto* node = owner.getShapeNode(i);
    if (!has_gc_wrapper(complete_address(node)))
      enumerate_gc(*node, edges);
  }
  descend(owner.getIK(), edges, [&owner] { owner.clearIK(); });
}

void enumerate_gc(dart::dynamics::Skeleton& owner, GcEdges& edges)
{
  for (std::size_t i = 0; i < owner.getNumBodyNodes(); ++i) {
    auto* body = owner.getBodyNode(i);
    if (!has_gc_wrapper(complete_address(body)))
      enumerate_gc(*body, edges);
  }
}

void enumerate_gc(dart::simulation::World& owner, GcEdges& edges)
{
  // Five World pointer copies, the ConstraintSolver, and the getter temporary.
  for (std::size_t i = 0; i < owner.getNumSkeletons(); ++i)
    descend(
        owner.getSkeleton(i),
        edges,
        [&owner] { owner.removeAllSkeletons(); },
        7);
  for (std::size_t i = 0; i < owner.getNumSimpleFrames(); ++i) {
    auto frame = owner.getSimpleFrame(i);
    descend(
        frame, edges, [&owner] { owner.removeAllSimpleFrames(); }, 5);
    // Vector, raw-pointer map, and both NameManager maps each own a copy.
    for (int copy = 0; copy < 3; ++copy)
      edges.add(owner.getSimpleFrame(frame->getName()), [&owner] {
        owner.removeAllSimpleFrames();
      });
  }
  if (!has_gc_wrapper(complete_address(owner.getConstraintSolver())))
    enumerate_gc(*owner.getConstraintSolver(), edges);
}
} // namespace dartnb
