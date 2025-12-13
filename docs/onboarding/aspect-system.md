# Aspect, State, and Properties System

## Overview

DART uses an **Aspect** system to add optional features (e.g., visualization, collision, bookkeeping) to objects at runtime without exploding inheritance hierarchies. The system also standardizes how those extensions expose:

- **State**: frequently-changing values (often updated every time step)
- **Properties**: rarely-changing configuration (often used for cache invalidation)

This document focuses on the core implementation in `dart/common/` and how it is used throughout the dynamics layer.

## Key Headers (Start Here)

- `dart/common/Aspect.hpp`: Base `Aspect` API (`State`, `Properties`, lifecycle hooks)
- `dart/common/AspectWithVersion.hpp`: Common helpers for Aspects that own State/Properties
- `dart/common/Composite.hpp`: Host container for Aspects (runtime attach/detach, state/property snapshots)
- `dart/common/SpecializedForAspect.hpp`: Constant-time access for selected Aspect types
- `dart/common/RequiresAspect.hpp`: Declares required Aspects (cannot be removed)
- `dart/common/detail/CompositeData.hpp`: Template machinery for typed State/Properties bundles
- `dart/common/EmbeddedAspect.hpp`: “Embedded” Aspects that store data inside the Composite

## Core Concepts

### 1) `Aspect`

An `Aspect` is a runtime-attached extension object:

- It is stored in a `Composite` and keyed by its C++ type (`std::type_index`).
- It can optionally expose `Aspect::State` and/or `Aspect::Properties`.
- It must implement `cloneAspect()` so a Composite can duplicate or transfer it.

Lifecycle hooks:

- `setComposite(Composite* newComposite)`: called after creation and on transfer to a new Composite
- `loseComposite(Composite* oldComposite)`: called before removal from the Composite

### 2) `Composite`

`Composite` is the host container that owns a set of Aspects:

- Storage: `Composite::AspectMap` (`std::map<std::type_index, std::unique_ptr<Aspect>>`)
- API: `has<T>()`, `get<T>()`, `set<T>()`, `createAspect<T>()`, `removeAspect<T>()`, `releaseAspect<T>()`

Because the map is keyed by type, lookups are typically `O(log N)`. If a class needs frequent access to a particular Aspect, it can inherit `SpecializedForAspect<ThatAspect>` for cached access (see below).

### 3) “State” vs “Properties”

Each Aspect may provide:

- **State** (`Aspect::State`): frequently changing; collected by `Composite::State`
- **Properties** (`Aspect::Properties`): configuration; collected by `Composite::Properties`

The separation exists so systems can snapshot/restore “state” cheaply and so caches can invalidate on “property” changes.

### 4) Versioning and cache invalidation

Many DART objects track a version number (see `dart/common/VersionCounter.hpp`). Helpers like `AspectWithVersionedProperties` call `CompositeType::incrementVersion()` when properties change, enabling cache invalidation patterns across the dynamics stack.

## Two Ways to Implement an Aspect

### A) Standalone Aspect (Aspect owns its data)

Use this when the Aspect can store its own State/Properties internally.

Common helpers:

- `common::AspectWithState` (State only)
- `common::AspectWithVersionedProperties` (Properties only + version bump)
- `common::AspectWithStateAndVersionedProperties` (both)

Example in tree: `dart/dynamics/EndEffector.hpp` defines `dynamics::Support` as:

- a standalone Aspect type attached to `dynamics::EndEffector`
- with both State and Properties
- using `DART_COMMON_ASPECT_STATE_PROPERTY_CONSTRUCTORS(Support)` to generate constructors

### B) Embedded Aspect (data lives inside the Composite)

Use this when you want the Composite object to “own” the State/Properties fields directly (e.g., for locality, avoiding extra allocations, or tight coupling).

The pattern is:

- The Composite inherits `EmbedState`, `EmbedProperties`, or `EmbedStateAndProperties` (from `dart/common/EmbeddedAspect.hpp`).
- The corresponding Aspect type (e.g., `EmbeddedPropertiesAspect<CompositeT, PropertiesDataT>`) forwards `get/set` of Aspect data to the Composite.
- When the Aspect is not attached to a Composite yet, it temporarily stores data on the heap; once attached, it pushes the data into the Composite and clears the temporary storage.

This forwarding behavior is implemented in `dart/common/detail/EmbeddedAspect.hpp`.

## Specialization and Required Aspects

### `SpecializedForAspect<SpecAspect>`

If a Composite needs fast access to a specific Aspect type, it can inherit:

- `common::SpecializedForAspect<SpecAspect>` (or multiple via the variadic form)

This caches an iterator for the specialized Aspect and provides constant-time `get<SpecAspect>()`/`has<SpecAspect>()` while still supporting generic access for other Aspect types.

### `RequiresAspect<ReqAspect>`

If an Aspect must always exist for correctness, inherit:

- `common::RequiresAspect<ReqAspect>`

This registers the Aspect type into `Composite::mRequiredAspects` so attempts to remove it are rejected (see checks in `dart/common/detail/Composite.hpp`). Required Aspects are also automatically specialized for access speed.

## Composite-Wide State and Properties Snapshots

### Runtime, type-erased snapshots

`Composite` can export/import type-erased snapshots:

- `Composite::State` (`detail::CompositeState`): map from Aspect type → cloneable `Aspect::State`
- `Composite::Properties` (`detail::CompositeProperties`): map from Aspect type → cloneable `Aspect::Properties`

This is used for copy/clone/serialization-style workflows:

- `setCompositeState(...)`, `getCompositeState()`, `copyCompositeStateTo(...)`
- `setCompositeProperties(...)`, `getCompositeProperties()`, `copyCompositePropertiesTo(...)`

### Typed bundles (`Composite::MakeState` / `Composite::MakeProperties`)

When you *know at compile time* which Aspects you care about, you can use the typed bundling helpers:

- `Composite::MakeState<Aspects...>`
- `Composite::MakeProperties<Aspects...>`

These are implemented in `dart/common/detail/CompositeData.hpp` using a `ComposeData` template that:

- inherits from each requested Aspect’s `State` or `Properties` type (multiple inheritance)
- can be constructed from a `Composite::State` / `Composite::Properties` snapshot (e.g., from `getCompositeState()` / `getCompositeProperties()`) to pull out those fields
- can be converted back into the type-erased `Composite::State` / `Composite::Properties` maps

Example in tree: `dynamics::EndEffector` defines:

- `using BasicProperties = common::Composite::MakeProperties<NameAspect, FixedFrame, EndEffector>;`

This bundles multiple “property aspects” into a single value type that can be passed around during construction and copying.

## Practical Tips / Pitfalls

- Prefer **standalone Aspects** when the Aspect is conceptually independent and can own its own memory.
- Prefer **embedded Aspects** when the data is tightly coupled to the Composite and benefits from being stored directly on the Composite.
- If you add a new required Aspect, inherit `RequiresAspect<YourAspect>` to prevent accidental removal.
- If you add a performance-sensitive Aspect, inherit `SpecializedForAspect<YourAspect>` (or use existing `DART_BAKE_SPECIALIZED_ASPECT(...)` patterns in the dynamics layer) to avoid repeated map lookups.
