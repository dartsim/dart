// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#if defined(__GNUC__)
  #pragma GCC diagnostic push
#elif defined(_MSC_VER)
  #pragma warning(push)
#endif
#include <nb_internals.h>
#if defined(__GNUC__)
  #pragma GCC diagnostic pop
#elif defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <optional>

#include <cstddef>

namespace dartnb {
namespace {
using namespace nanobind::detail;

// Public inst_replace_* copies/moves the native object. Factory objects can be
// nonmovable and can contain weak self-pointers tied to their shared_ptr
// control block, so attaching one requires this narrowly isolated internal
// operation. A compatible newer nanobind can retain this ABI; changes require
// an audit.
static_assert(
    NB_INTERNALS_VERSION == 22, "audit nanobind factory attachment ABI");
static_assert(sizeof(nb_inst_state) == sizeof(uint32_t));
static_assert(sizeof(nb_inst) == sizeof(PyObject) + 2 * sizeof(uint32_t));
static_assert(offsetof(nb_inst, offset) == sizeof(PyObject));
static_assert(offsetof(nb_inst, state) == sizeof(PyObject) + sizeof(int32_t));
static_assert(nb_inst_state::state_uninitialized == 0);
static_assert(nb_inst_state::state_ready == 2);

struct Mapping
{
  nb_ptr_map::iterator entry;
  nb_inst_seq* alias;

  void replace(PyObject* replacement)
  {
    if (alias)
      alias->inst = replacement;
    else
      entry.value() = replacement;
  }
};

Mapping mapping(nb_shard& shard, void* pointer, PyObject* expected)
{
  auto entry = shard.inst_c2p.find(pointer);
  if (entry != shard.inst_c2p.end()) {
    if (entry->second == expected)
      return {entry, nullptr};
    if (nb_is_seq(entry->second)) {
      for (auto* alias = nb_get_seq(entry->second); alias; alias = alias->next)
        if (alias->inst == expected)
          return {entry, alias};
    }
  }
  throw nb::type_error("inconsistent native factory instance mapping");
}

void pointTo(nb_inst* instance, void* pointer, bool keepAlive)
{
  // Both objects use GC allocation with at least one pointer of payload space.
  // The old inline payload is unconstructed and can serve as the pointer slot.
  *reinterpret_cast<void**>(instance + 1) = pointer;
  instance->offset = static_cast<int32_t>(sizeof(nb_inst));
  auto state = instance->state;
  state.direct = false;
  state.internal = false;
  state.destruct = false;
  state.cpp_delete = false;
  state.clear_keep_alive = keepAlive;
  nb_inst_state_write(instance, state);
}
} // namespace

void attachFactoryInstance(nb::handle self, nb::handle prepared)
{
  auto* destination = reinterpret_cast<nb_inst*>(self.ptr());
  auto* source = reinterpret_cast<nb_inst*>(prepared.ptr());
  auto* destinationType = nb_type_data(Py_TYPE(self.ptr()));
  auto* sourceType = nb_type_data(Py_TYPE(prepared.ptr()));
  if (destinationType->internals != sourceType->internals
      || destinationType->type != sourceType->type
      || !destination->state.internal || source->state.internal
      || destination->state.state != nb_inst_state::state_uninitialized
      || source->state.state != nb_inst_state::state_uninitialized
      || destination->state.destruct || source->state.destruct
      || destination->state.cpp_delete || source->state.cpp_delete
      || !PyType_HasFeature(Py_TYPE(self.ptr()), Py_TPFLAGS_HAVE_GC)
      || !PyType_HasFeature(Py_TYPE(prepared.ptr()), Py_TPFLAGS_HAVE_GC)
      || destinationType->size < sizeof(void*)
      || sourceType->size < sizeof(void*))
    throw nb::type_error("incompatible native factory attachment");

  void* oldPointer = inst_ptr(destination);
  void* newPointer = inst_ptr(source);
  auto* internals = destinationType->internals;
  auto& oldShard = internals->shard(oldPointer);
  auto& newShard = internals->shard(newPointer);
  bool oldFirst = std::less<nb_shard*>{}(&oldShard, &newShard);
  lock_shard first(oldFirst ? oldShard : newShard);
  std::optional<lock_shard> second;
  if (&oldShard != &newShard)
    second.emplace(oldFirst ? newShard : oldShard);

  auto oldMapping = mapping(oldShard, oldPointer, self.ptr());
  auto newMapping = mapping(newShard, newPointer, prepared.ptr());
  auto sourceKeepAlive = newShard.keep_alive.find(prepared.ptr());
  if (!source->state.clear_keep_alive
      || sourceKeepAlive == newShard.keep_alive.end())
    throw nb::type_error("native factory owner is missing");
  auto* owners = static_cast<nb_weakref_seq*>(sourceKeepAlive->second);
  nb_weakref_seq* previous = nullptr;
  if (destination->state.clear_keep_alive) {
    auto entry = oldShard.keep_alive.find(self.ptr());
    if (entry == oldShard.keep_alive.end())
      throw nb::type_error("inconsistent pending factory owner");
    previous = static_cast<nb_weakref_seq*>(entry->second);
  }

  // This insertion is the only potentially throwing mutation. Until it
  // succeeds, both wrappers, mappings, and owner chains remain untouched.
  if (&oldShard != &newShard || !previous) {
    if (!newShard.keep_alive.try_emplace(self.ptr(), owners).second)
      throw nb::type_error("duplicate pending factory owner");
  } else {
    newShard.keep_alive.find(self.ptr()).value() = owners;
  }
  if (previous) {
    auto* tail = owners;
    while (tail->next)
      tail = tail->next;
    tail->next = previous;
    if (&oldShard != &newShard)
      oldShard.keep_alive.erase(self.ptr());
  }
  newShard.keep_alive.erase(prepared.ptr());

  // Swap the registered addresses, preserving any other aliases. Destruction
  // of prepared removes the old inline mapping without destroying native data.
  oldMapping.replace(prepared.ptr());
  newMapping.replace(self.ptr());
  pointTo(destination, newPointer, true);
  pointTo(source, oldPointer, false);
}
} // namespace dartnb
