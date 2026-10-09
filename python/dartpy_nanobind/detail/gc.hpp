#pragma once

#include <functional>
#include <unordered_set>
#include <vector>

namespace dart {
namespace optimizer {
class Problem;
class Solver;
} // namespace optimizer
namespace constraint {
class ConstraintSolver;
}
namespace utils {
class CompositeResourceRetriever;
}
} // namespace dart

namespace dartnb {
// One Python reference per control block, discoverable without nanobind
// internals.
struct PythonPin
{
  PyObject* object;
  template <class T>
  void operator()(T*) const noexcept
  {
    nanobind::gil_scoped_acquire guard;
    Py_DECREF(object);
  }
};

struct GcEdges
{
  struct Edge
  {
    std::shared_ptr<const void> owner;
    const PythonPin* pin;
    std::function<void()> clear;
  };
  std::vector<Edge> edges;
  std::vector<std::shared_ptr<const void>> native_owners;
  std::unordered_set<const void*> seen;
  template <class T, class Clear>
  void add(const std::shared_ptr<T>& owner, Clear clear)
  {
    if (auto* pin = std::get_deleter<PythonPin>(owner))
      edges.push_back({owner, pin, std::move(clear)});
  }
  int traverse(visitproc visit, void* arg) const;
  void clear();
};

void enumerate_gc(dart::optimizer::Problem&, GcEdges&);
void enumerate_gc(dart::optimizer::Solver&, GcEdges&);
void enumerate_problem_gc(std::shared_ptr<dart::optimizer::Problem>&, GcEdges&);

template <class T>
struct GcProperties : std::false_type
{
};
template <class T, std::enable_if_t<GcProperties<T>::value, int> = 0>
void enumerate_gc(T& owner, GcEdges& edges)
{
  enumerate_problem_gc(owner.mProblem, edges);
}
void enumerate_gc(dart::collision::CollisionOption&, GcEdges&);
void enumerate_gc(dart::constraint::ConstraintSolver&, GcEdges&);
void enumerate_gc(dart::utils::CompositeResourceRetriever&, GcEdges&);
void enumerate_gc(dart::dynamics::InverseKinematics&, GcEdges&);
void enumerate_gc(dart::simulation::World&, GcEdges&);
void enumerate_gc(dart::dynamics::Skeleton&, GcEdges&);
void enumerate_gc(dart::dynamics::BodyNode&, GcEdges&);
void enumerate_gc(dart::dynamics::ShapeFrame&, GcEdges&);

// enumerate_gc overloads opt owners in; inherited owners use public base state.
template <class T>
auto& gc_owner(T& value)
{
  if constexpr (std::is_base_of_v<dart::optimizer::Problem, T>)
    return static_cast<dart::optimizer::Problem&>(value);
  else if constexpr (std::is_base_of_v<dart::optimizer::Solver, T>)
    return static_cast<dart::optimizer::Solver&>(value);
  else if constexpr (std::is_base_of_v<dart::constraint::ConstraintSolver, T>)
    return static_cast<dart::constraint::ConstraintSolver&>(value);
  else if constexpr (std::is_base_of_v<
                         dart::utils::CompositeResourceRetriever,
                         T>)
    return static_cast<dart::utils::CompositeResourceRetriever&>(value);
  else if constexpr (std::is_base_of_v<dart::dynamics::ShapeFrame, T>)
    return static_cast<dart::dynamics::ShapeFrame&>(value);
  else if constexpr (std::is_base_of_v<dart::dynamics::BodyNode, T>)
    return static_cast<dart::dynamics::BodyNode&>(value);
  else
    return value;
}

template <class T, class = void>
struct GcOwner : std::false_type
{
};
template <class T>
struct GcOwner<
    T,
    std::void_t<decltype(enumerate_gc(
        gc_owner(std::declval<T&>()), std::declval<GcEdges&>()))>>
  : std::true_type
{
};

template <class T>
struct GcSlots
{
  static int traverse(PyObject* self, visitproc visit, void* arg) noexcept
  {
    Py_VISIT(Py_TYPE(self));
    if (!nanobind::inst_ready(self))
      return 0;
    try {
      GcEdges edges;
      enumerate_gc(gc_owner(*nanobind::inst_ptr<T>(self)), edges);
      return edges.traverse(visit, arg);
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError, "DART GC traversal failed");
      return -1;
    }
  }
  static int clear(PyObject* self) noexcept
  {
    if (!nanobind::inst_ready(self))
      return 0;
    try {
      GcEdges edges;
      enumerate_gc(gc_owner(*nanobind::inst_ptr<T>(self)), edges);
      edges.clear();
      return 0;
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError, "DART GC clearing failed");
      return -1;
    }
  }
};

template <class T>
const PyType_Slot* gc_slots()
{
  if constexpr (GcOwner<T>::value) {
    static const PyType_Slot slots[]
        = {{Py_tp_traverse, reinterpret_cast<void*>(&GcSlots<T>::traverse)},
           {Py_tp_clear, reinterpret_cast<void*>(&GcSlots<T>::clear)},
           {0, nullptr}};
    return slots;
  } else {
    static const PyType_Slot slots[] = {{0, nullptr}};
    return slots;
  }
}
} // namespace dartnb
