// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <dart/collision/CollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Node.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Subject.hpp>

#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace dartnb {
namespace {
struct Path
{
  std::vector<Adjust> up;
  std::vector<Adjust> down;
};
struct Descendant
{
  Key type;
  Path path;
};
struct Identity
{
  Key exact;
  void* complete;
  nb::weakref reference;
};
struct Owner
{
  std::weak_ptr<void> native;
  const void* token;
};
struct Registry
{
  std::unordered_map<Key, Entry> entries;
  std::unordered_map<void*, Owner> owners;
  std::unordered_map<void*, std::unordered_map<Key, Identity*>> identities;
  std::unordered_map<Key, std::unordered_map<Key, Path>> paths;
  std::unordered_map<Key, std::vector<Descendant>> descendants;
};
Registry& registry()
{
  static Registry data;
  return data;
}
bool path_to(Key from, Key to, Path& path)
{
  if (from == to)
    return true;
  auto found = registry().entries.find(from);
  if (found == registry().entries.end())
    return false;
  for (auto edge : found->second.bases) {
    path.up.push_back(edge.up);
    path.down.push_back(edge.down);
    if (path_to(edge.base, to, path))
      return true;
    path.up.pop_back();
    path.down.pop_back();
  }
  return false;
}
void* apply_path(
    const std::vector<Adjust>& path, void* pointer, bool reverse = false)
{
  if (reverse) {
    for (auto it = path.rbegin(); it != path.rend() && pointer; ++it)
      pointer = (*it)(pointer);
  } else {
    for (auto adjust : path)
      pointer = adjust(pointer);
  }
  return pointer;
}
} // namespace

// Entries live until wrapper deallocation, even after GC clears its weakrefs.
bool has_gc_wrapper(void* complete)
{
  return registry().identities.count(complete) != 0;
}

std::shared_ptr<void> native_owner(void* complete)
{
  auto found = registry().owners.find(complete);
  return found == registry().owners.end() ? std::shared_ptr<void>()
                                          : found->second.native.lock();
}
void remember_owner(
    void* complete, const std::shared_ptr<void>& owner, const void* token)
{
  registry().owners[complete] = {owner, token};
}
void forget_owner(void* complete, const void* token) noexcept
{
  auto& owners = registry().owners;
  auto found = owners.find(complete);
  // Weakref callbacks can install a replacement wrapper before old cleanup.
  if (found != owners.end() && found->second.token == token)
    owners.erase(found);
}

void hold_native_owner(
    nb::handle wrapper, std::shared_ptr<void> owner, void* complete)
{
  struct Payload
  {
    std::shared_ptr<void> owner;
    void* complete;
  };
  auto holder = std::make_unique<Payload>(Payload{std::move(owner), complete});
  remember_owner(complete, holder->owner, holder.get());
  nb::keep_alive_cb(wrapper, holder.get(), [](void* q) noexcept {
    auto* payload = static_cast<Payload*>(q);
    forget_owner(payload->complete, payload);
    delete payload;
  });
  holder.release();
}

void retain_shared_owner(
    nb::handle wrapper,
    bool is_new,
    std::shared_ptr<void> owner,
    void* complete)
{
  if (is_new || (!nb::inst_state(wrapper).second && !native_owner(complete)))
    hold_native_owner(wrapper, std::move(owner), complete);
}

namespace {
// The BodyNode whose skeleton owns a graph object, or null for other types.
dart::dynamics::BodyNode* owning_body(Key type, void* pointer)
{
  namespace d = dart::dynamics;
  if (auto* body
      = static_cast<d::BodyNode*>(upcast(type, typeid(d::BodyNode), pointer)))
    return body;
  if (auto* node
      = static_cast<d::Node*>(upcast(type, typeid(d::Node), pointer)))
    return node->getBodyNodePtr();
  if (auto* joint
      = static_cast<d::Joint*>(upcast(type, typeid(d::Joint), pointer)))
    return joint->getChildBodyNode();
  if (auto* dof = static_cast<d::DegreeOfFreedom*>(
          upcast(type, typeid(d::DegreeOfFreedom), pointer)))
    return dof->getJoint()->getChildBodyNode();
  if (auto* subject = static_cast<dart::common::Subject*>(
          upcast(type, typeid(dart::common::Subject), pointer))) {
    // A registered Frame/Entity can wrap an unregistered native graph type.
    if (auto* body = dynamic_cast<d::BodyNode*>(subject))
      return body;
    if (auto* node = dynamic_cast<d::Node*>(subject))
      return node->getBodyNodePtr();
  }
  return nullptr;
}
} // namespace

void hold_body(nb::handle wrapper, Key type, void* pointer)
{
  namespace d = dart::dynamics;
  if (auto* body = owning_body(type, pointer)) {
    nb::keep_alive_cb(wrapper, new d::BodyNodePtr(body), [](void* p) noexcept {
      delete static_cast<d::BodyNodePtr*>(p);
    });
  }
}

std::shared_ptr<void> graph_owner(Key type, void* pointer, void* complete)
{
  auto* body = owning_body(type, pointer);
  if (!body)
    return {};
  // The deleter's BodyNodePtr keeps the owning skeleton alive and follows the
  // body if it moves to another skeleton.
  return std::shared_ptr<void>(
      complete, [holder = dart::dynamics::BodyNodePtr(body)](void*) mutable {
        holder = nullptr;
      });
}

void remember_wrapper(Key exact, void* complete, nb::handle wrapper)
{
  // The payload owns the weakref; the static registry owns no Python
  // references.
  auto* identity = new Identity{exact, complete, nb::weakref(wrapper)};
  registry().identities[complete][exact] = identity;
  nb::keep_alive_cb(wrapper, identity, [](void* p) noexcept {
    auto* entry = static_cast<Identity*>(p);
    auto& identities = registry().identities;
    auto object = identities.find(entry->complete);
    if (object != identities.end()) {
      auto type = object->second.find(entry->exact);
      if (type != object->second.end() && type->second == entry)
        object->second.erase(type);
      if (object->second.empty())
        identities.erase(object);
    }
    delete entry;
  });
}

void register_type(Key key, Entry entry)
{
  auto& data = registry();
  for (const auto& edge : entry.bases)
    if (!data.entries.count(edge.base))
      throw std::runtime_error(
          std::string("dartpy: base of ") + key.name()
          + " registered after the derived type");
  data.entries.emplace(key, std::move(entry));
  // Declared bases are always registered before their derived types (the
  // module keeps pybind11's registration order), so the new type's paths to
  // its ancestors are final now and no existing path can change. Compute
  // only those instead of rebuilding every pair, which made import cubic in
  // the number of registered types.
  auto& own = data.paths[key];
  own.emplace(key, Path{});
  for (const auto& target : data.entries) {
    if (target.first == key)
      continue;
    Path path;
    if (path_to(key, target.first, path)) {
      own.emplace(target.first, path);
      data.descendants[target.first].push_back({key, path});
    }
  }
}

void register_methods(Key key, void (*methods)(nb::handle))
{
  registry().entries.at(key).rebind_methods = methods;
}

void rebind_methods(Key key, nb::handle cls)
{
  auto found = registry().entries.find(key);
  if (found != registry().entries.end() && found->second.rebind_methods)
    found->second.rebind_methods(cls);
}

const Key* find_registered(const char* name)
{
  // Types register at import, so rebuild the index only when the count grows.
  static std::unordered_map<std::string, Key> by_name;
  static std::size_t indexed = 0;
  const auto& entries = registry().entries;
  if (indexed != entries.size()) {
    by_name.clear();
    for (const auto& entry : entries)
      by_name.emplace(entry.first.name(), entry.first);
    indexed = entries.size();
  }
  const auto found = by_name.find(name);
  return found == by_name.end() ? nullptr : &found->second;
}

void* upcast(Key source, Key target, void* pointer)
{
  auto source_it = registry().paths.find(source);
  if (source_it == registry().paths.end())
    return nullptr;
  auto target_it = source_it->second.find(target);
  return target_it == source_it->second.end()
             ? nullptr
             : apply_path(target_it->second.up, pointer);
}

nb::handle wrap(
    Key source,
    Key dynamic_type,
    void* complete,
    void* pointer,
    nb::rv_policy policy,
    nb::handle parent,
    bool* is_new)
{
  auto& data = registry();
  if (is_new)
    *is_new = false;
  auto selected = data.entries.find(dynamic_type);
  if (selected != data.entries.end()) {
    pointer = complete;
  } else {
    selected = data.entries.find(source);
    if (selected == data.entries.end())
      throw std::runtime_error("unregistered source type");
    auto candidates = data.descendants.find(source);
    if (candidates != data.descendants.end()) {
      void* source_pointer = pointer;
      for (const auto& candidate : candidates->second) {
        void* adjusted = apply_path(candidate.path.down, source_pointer, true);
        if (adjusted && data.paths.at(candidate.type).count(selected->first)) {
          selected = data.entries.find(candidate.type);
          pointer = adjusted;
        }
      }
    }
  }
  auto& entry = selected->second;
  nb::object existing;
  auto object = data.identities.find(complete);
  if (object != data.identities.end()) {
    auto type = object->second.find(selected->first);
    if (type != object->second.end())
      existing = type->second->reference();
  }
  // Existing identity retains its original ownership, as in both libraries.
  // Adding reverse parent dependencies here would create BodyNode/ShapeNode
  // cycles.
  if (existing.is_valid() && !existing.is_none())
    return existing.release();
  if (policy == nb::rv_policy::none)
    return {};
  // shared_ptr exports attach their owner in shared_caster. Raw detector
  // exports recover the same native control block even after its wrapper dies.
  std::shared_ptr<dart::collision::CollisionDetector> detector_owner;
  if (!is_new) {
    if (auto* detector
        = static_cast<dart::collision::CollisionDetector*>(upcast(
            selected->first,
            typeid(dart::collision::CollisionDetector),
            pointer)))
      detector_owner = detector->weak_from_this().lock();
  }
  if ((entry.graph_owned || detector_owner)
      && (policy == nb::rv_policy::automatic
          || policy == nb::rv_policy::take_ownership))
    policy = nb::rv_policy::reference;
  nb::object result;
  if (policy == nb::rv_policy::take_ownership
      || policy == nb::rv_policy::automatic)
    result = nb::inst_take_ownership(entry.python_type, pointer);
  else
    result = nb::inst_reference(
        entry.python_type,
        pointer,
        policy == nb::rv_policy::reference_internal ? parent : nb::handle());
  if (detector_owner)
    hold_native_owner(result, detector_owner);
  hold_body(result, selected->first, pointer);
  remember_wrapper(selected->first, complete, result);
  if (is_new)
    *is_new = true;
  return result.release();
}
} // namespace dartnb
