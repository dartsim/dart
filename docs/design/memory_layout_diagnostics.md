# Memory Layout Diagnostics

## Status

Accepted cross-branch contract for the DART 7 and DART 6 diagnostics panels.
Implementation is incremental: a panel must label which interaction and
evidence layers it currently implements rather than implying completion of
later filtering, accessibility, or measured-access slices. This document owns
the durable meaning of a memory-layout view. Active branch work and
verification state belong in the corresponding dev task or plan.

## Requirement

The memory diagnostic must be a two-dimensional view of actual host-memory
layout, not a decorative rendering of aggregate counters. A user should be
able to inspect:

- each real contiguous backing region that the diagnostic provider can prove;
- allocated, free, reserved, padding, and allocator-metadata byte ranges in
  address order;
- known simulation-data categories within allocated payloads;
- logical live, inactive, and spare-capacity subranges when that state is
  observable; and
- discontinuities, fragmentation, alignment, page boundaries, and cache-line
  boundaries without mistaking those observations for measured cache behavior.

The contract applies to both DART 7 and DART 6. It does not require the two
branches to pretend they have the same storage architecture. DART 7 can report
World-owned allocator regions and known typed overlays. The classic DART 6
object graph is not stored in its World `MemoryManager`, so DART 6 must present
its real allocator arenas separately from a sparse address atlas of observed
objects and buffers.

This is a CPU host-memory design. GPU or other device residency remains a
separate backend-specific diagnostic governed by
[`scalable_compute_decisions.md`](scalable_compute_decisions.md).

## Current Architecture Reality

| Surface                  | What is true                                                                                                                                                                    | Consequence for the view                                                                                                                              |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| DART 7 World allocator   | A World-owned free-list allocator can own multiple contiguous backing chunks, and the frame allocator owns a primary bump region plus separate overflow allocations.            | Draw every backing chunk as its own region. Do not concatenate chunks or overflow allocations into a fictitious arena.                                |
| DART 7 ECS               | Default EnTT component storage is paged. Packed entity arrays, component pages, and sparse-index storage can be distinct allocations.                                           | A component capacity row is not evidence of one contiguous component byte range. Report only pages and buffers whose extents are directly observable. |
| DART 7 data architecture | The Model/State/Control/Contacts split is not yet one globally stable dense slab.                                                                                               | Categorization describes known ranges in the current implementation; it must not imply that the target dense-slab architecture already exists.        |
| DART 6 World allocator   | The World reserves a free-list region and a resettable frame region, but classic Skeleton, BodyNode, Joint, Shape, collision, and solver allocations do not generally use them. | Label those regions as `World MemoryManager` reservations, not as the DART 6 simulation-data arena.                                                   |
| DART 6 object graph      | The graph is composed primarily of independently allocated objects and nested buffers.                                                                                          | Use an address atlas for observed objects and buffers. Atlas gaps are unobserved process address space, never allocator free space or fragmentation.  |
| Existing demo grid       | The first grid groups active, hole, and reserved counts in that order. It intentionally discards address order.                                                                 | Its cells cannot be renamed or restyled into an actual memory map; the underlying model and schema must change.                                       |

The DART 7 ownership and no-allocation contract is defined in
[`hierarchical_allocator.md`](hierarchical_allocator.md). The incomplete
physical Model/State split is recorded as finding F2 in
[`dart7_architecture_assessment.md`](dart7_architecture_assessment.md).

## Evidence Contract

The UI and exported snapshot must name the evidence layer behind each claim.
Higher-numbered layers cannot be inferred from lower-numbered layers.

| Layer | Name                     | What it establishes                                                                                                                  | What it does not establish                                                                              |
| ----: | ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------- |
|     1 | Actual region            | A contiguous virtual-address extent with a byte-accurate base and size during capture.                                               | Ownership of every byte unless allocator bookkeeping supplies it; physical-page adjacency or residency. |
|     2 | Actual allocation        | Allocated, free, reserved, padding, or metadata subranges observed from allocator bookkeeping or a known pointer-plus-extent buffer. | Which bytes are accessed, how often they are reused, or their cache state.                              |
|     3 | Layout-derived locality  | Virtual-address adjacency, alignment, range gaps, fragmentation, and cache-line or page crossings calculated from layers 1 and 2.    | Access order, working set, reuse distance, TLB behavior, cache misses, NUMA placement, or latency.      |
|     4 | Measured access locality | Sampled or instrumented loads and stores over a named workload and interval.                                                         | Cache behavior unless the collection method also reports the relevant hardware or simulator events.     |
|     5 | Measured cache behavior  | Named cache, TLB, data-source, latency, or NUMA events from a hardware counter or cache simulator.                                   | Causality or end-to-end speedup without a controlled performance comparison.                            |

All current map geometry is in process virtual-address space. A 4 KiB virtual
page bucket is not proof of a resident physical page, and adjacent virtual
pages are not proof of adjacent physical pages. A cache-line overlay shows
alignment against the host-reported line size; it does not show that the line
was fetched or missed.

Process RSS remains a separate whole-process metric. It must not be summed with
World allocator regions, ECS pages, or atlas observations.

## Two Truthful View Kinds

### Exact region map

An exact region map begins with a provider-observed contiguous allocation whose
base and extent are known at capture time. The public snapshot removes the base
address and uses byte offsets from that base. Its state spans partition the
entire region; bytes that the provider cannot classify are `Unknown`, not
silently assigned to a neighboring state.

Fragmentation metrics are valid only for this view and only when the allocator
provides a complete free/allocated partition. Examples are a free-list backing
chunk, a frame arena, an overflow allocation with known size, or a known
contiguous vector payload.

### Address atlas

An address atlas orders independently observed addresses or extents in virtual
address order. It is useful for seeing whether DART 6 objects of related types
cluster or scatter, but the encompassing address window is not an allocation.
The atlas therefore has no `free` state and no fragmentation percentage.

Each observation records whether it is:

- an exact extent, such as a known contiguous buffer pointer and capacity;
- an estimated shallow extent, such as a concrete object address plus a known
  `sizeof` that excludes nested storage; or
- a point observation when no defensible extent is available.

Large gaps may be rendered with an explicit broken-axis marker. The gap remains
`Unobserved` regardless of its size. An atlas must never fill gaps with a color
that is also used for allocator-free or arena-reserved bytes.

## Renderer-Neutral Snapshot Model

The model is branch-neutral even though collection is branch-specific. The
exact C++ spelling may follow local naming conventions, but the following
semantics are required.

```text
MemoryLayoutSnapshot
  schema                         dart.memory-diagnostics.v2
  generation                     scene/World generation
  capture_sequence               monotonic within the generation
  host_page_size_bytes           optional measured host value
  cache_line_size_bytes          optional measured host value
  regions[]                      exact contiguous region maps
  atlases[]                      sparse address-order observations
  guidance[]                     snapshot-wide limits and unavailable evidence

MemoryLayoutRegion
  key                            generation-local opaque key
  label, scope, source
  size_bytes, alignment_bytes
  base_mod_page_size_bytes       optional low-bit alignment only
  base_mod_cache_line_size_bytes optional low-bit alignment only
  region_kind                    free-list, frame, overflow, known buffer, ...
  completeness                   complete partition or partial observation
  ranges[]                       sorted byte-range partition

MemoryLayoutRange
  offset_bytes, size_bytes
  storage_state                  allocated, free, reserved, metadata, padding,
                                 or unknown
  logical_use                    active, inactive, spare capacity, unknown,
                                 or not applicable
  data_category                  branch-neutral semantic category
  diagnostic_label              best-effort leaf type/role label
  evidence_layer, source, limitation

MemoryAddressAtlas
  key, label, scope, source
  observations[]                 sorted by captured virtual address

MemoryAddressObservation
  window_ordinal, offset_bytes
  optional_size_bytes
  extent_evidence                exact, shallow estimate, or point only
  data_category, diagnostic_label
  evidence_layer, source, limitation
```

`dart.memory-diagnostics.v2` is intentionally incompatible with the grouped
logical maps in v1. A v1 consumer must not interpret v2 range bytes as logical
slot counts, and a v2 producer must not retain the old active/hole/reserved
ordering as a fallback labeled "memory map."

### Orthogonal state axes

Allocator state, logical utilization, and semantic data type answer different
questions and stay separate:

- `storage_state` answers whether the allocator owns a live payload, knows a
  range is free, holds unused arena capacity, or uses bytes for metadata or
  padding.
- `logical_use` answers whether bytes inside an allocated payload correspond
  to active elements, tombstones/inactive slots, or spare materialized
  capacity.
- `data_category` answers what the payload means to the simulation.

For example, an EnTT tombstone remains allocated component-page memory with
`logical_use=inactive`; it is not allocator-free memory. Similarly, unused
frame-arena capacity is reserved scratch capacity, not a free-list hole.

### Semantic categories

Colors are keyed to a small DART-owned taxonomy, while
`diagnostic_label` can carry a more specific best-effort type or role name.

| Category                  | Typical content                                                                                                  |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| Model/topology            | Body and joint topology, masses, inertias, material parameters, immutable baked data                             |
| State                     | Positions, orientations, velocities, generalized coordinates, warm starts that are semantically persistent state |
| Control/input             | Forces, torques, targets, actuator commands, lowered policy input                                                |
| Contact/constraint/solver | Contact manifolds, constraint rows, solver-owned persistent work data                                            |
| Collision/geometry        | Shapes, broad-phase structures, collision geometry and query data                                                |
| Cache/scratch             | Recomputable caches, per-step work arrays, frame-arena payload                                                   |
| Entity/index              | Entity identifiers, packed entity arrays, sparse indexes, dense-index maps                                       |
| Allocator infrastructure  | Pool descriptors and other allocator-owned payload that is not user simulation data                              |
| Unknown                   | Allocated bytes whose semantic owner cannot be proved                                                            |

This taxonomy is diagnostic metadata, not a replacement for DART 7's future
physical Model/State contract or the existing component serialization
categories. Unknown allocations remain visible and conspicuous; category
coverage is itself a useful diagnostic.

### Snapshot invariants

- All offset and size arithmetic is checked for overflow.
- Exact-region ranges are sorted, non-overlapping, contained in the region,
  and cover its full extent. A partial provider fills uncovered bytes with
  `Unknown` and marks the region partial.
- A semantic annotation can refine only an allocated payload range. Conflicting
  annotations are a provider error, not a request for the renderer to choose a
  color.
- Region and atlas keys are opaque diagnostic tokens. A provider that supports
  retained row state or layout comparison keeps them stable while the same
  observed region survives within one World generation, but they are not
  component, allocator, or object identifiers and must not be persisted across
  generations. An incremental current-sample-only provider may instead expose
  an explicitly snapshot-local address-order ordinal; consumers must not
  persist that ordinal, and stable opaque identity is required before adding
  retained selection or comparison.
- Page and cache-line overlays use only the captured base-address remainder for
  the corresponding host size. That low-bit alignment is enough to place
  boundaries after the raw base is scrubbed.
- Raw addresses may be used inside collection to sort and correlate ranges.
  They are removed before the renderer-neutral snapshot crosses the provider
  boundary.
- The snapshot owns its strings and scalar ranges. It contains no live pointer,
  span into allocator memory, EnTT type, component type, or renderer object.
- Layout payload stays on the current sample and is omitted from the bounded
  history ring unless a future, separately designed compressed-history format
  proves its cost and comparison semantics.

## DART 7 Provider

The DART 7 collector belongs beside the registry and allocator-aware storage in
the opaque World implementation. `World` remains a forwarding facade;
`detail::MemoryDiagnosticsTracker` (or a focused component it owns) performs
the capture and returns curated value types. EnTT, component types, allocator
headers, and raw addresses never enter a promoted public header.

Collection follows this order while the World is quiescent between steps:

1. Snapshot every free-list backing chunk and its allocator bookkeeping.
   Partition it into block headers plus allocated or free block payload in
   address order. Report aligned-allocation headers, color/alignment padding,
   and requested payload separately only when the allocator retains enough
   metadata to prove those internal boundaries; otherwise the allocated block
   remains one explicitly coarse allocator-owned extent.
2. Snapshot the frame allocator's primary region as used and reserved ranges.
   Report each live overflow allocation as a separate region. Without explicit
   per-allocation tags, the used prefix is `Cache/scratch` rather than a
   fabricated sequence of solver types.
3. Collect exact typed annotations from DART-owned storage whose pointer and
   byte extent are known: known ECS component pages, packed entity buffers, and
   selected dense World/stage buffers. Correlate each annotation to the
   containing allocator region, then convert it to a region-relative offset.
4. Split allocated payload ranges at annotation boundaries. Leave every
   uncorrelated allocation as `Unknown` rather than guessing from size or
   allocation order.
5. Validate the partition, scrub raw bases, and publish the immutable snapshot.

Default EnTT component storage is paged, so a known component collector reports
each materialized page separately using its actual page pointer and byte
extent. Packed entity arrays are separate ranges. Sparse pages or unknown
component storage remain unclassified when the type-erased interface cannot
prove their extents. A logical storage hole affects `logical_use`; it does not
turn page bytes into allocator-free space.

The semantic mapping is an internal DART table from known components or buffer
owners to the branch-neutral categories. It must not expose `typeid(T).name()`
as a stable identifier or reuse the serialization-oriented component category
as if it were a physical data-layout contract.

Pool storage receives only the precision its allocator can prove. If the
allocator exposes a backing block but not reliable slot ownership, show the
block as allocated allocator infrastructure or unknown payload. Do not derive
an exact slot map from aggregate pool counters.

The collection path is opt-in. It must not add a per-allocation semantic log to
the normal simulation hot path. If a future owner needs persistent tags, it
should register stable known buffers at construction/bake boundaries and prove
the disabled-path and steady-state overhead separately.

## DART 6 Provider

DART 6 uses the same schema and visual grammar but has two distinct providers.
Neither should add more bookkeeping to the already broad classic `World`
class. A focused diagnostics collector reads the existing World and
`MemoryManager` surfaces; allocator traversal stays in a non-mutating common
or detail-level snapshot helper. DART 6 compatibility work must not add data
members, virtual functions, or otherwise change public class layouts merely to
support this view.

### World MemoryManager regions

The free-list, frame, and any observable pool backing regions use exact region
maps. Their scope text must say that these are World reservation arenas. The
free-list prewarm allocation/deallocation and frame reset do not prove that
Skeleton, BodyNode, Joint, Shape, collision, or classic solver data resides in
those arenas.

### Classic graph address atlas

The collector traverses the active graph at a safe point, captures addresses,
and assigns type-specific diagnostic labels and semantic categories. It uses:

- point observations for objects whose complete allocation extent is unknown;
- shallow estimates only when the concrete counted type and exclusion of
  nested storage are explicit; and
- exact extents for contiguous buffers only when a stable pointer and byte
  capacity can be read safely during the snapshot.

The atlas can show address-order clustering, distinct virtual-page buckets,
and gaps as layout proxies. It cannot report allocator fragmentation, heap
ownership, physical adjacency, or cache misses. A missing object class or
nested allocation is `unavailable` or unobserved, not zero.

This is the truthful DART 6 application of the requirement: it visualizes the
contiguous regions that actually exist and exposes the scattered topology of
the legacy graph. Migrating the graph into a preallocated Model/State layout is
a separate architecture and compatibility project, not a visualization side
effect.

## Visual Grammar And Interaction

The model remains renderer-neutral and is consumed by each branch-local panel
adapter. DART 7 presents it through `PanelBuilder`; DART 6's current demos
adapter uses the legacy Dear ImGui/`DemoHost` path. In both cases, drawing,
sizing, clipping, and hit testing stay below the diagnostic model boundary.

### Geometry

- Each exact region has its own labeled group with size, source, completeness,
  and utilization summary. Regions are never joined across an address gap.
- Bytes advance left-to-right, then top-to-bottom. Every cell at one zoom level
  covers the same number of bytes within a region.
- The user can adjust bytes per row or zoom. Page and cache-line overlays
  appear only when the current scale can represent them without implying false
  precision.
- Atlas windows use an explicit broken-axis mark when a large unobserved gap is
  visually compressed. Exact offsets remain available in details.
- When several ranges fall into one screen cell, the cell is marked mixed and
  shows ordered sub-stripes or another composition cue. Zooming reveals exact
  boundaries. A blended color must not hide minority categories or suggest a
  new category.

### Encoding

- Semantic category uses a contrast-tested, color-vision-deficiency-friendly
  palette.
- Storage state and logical use also use borders, hatching, opacity, or glyphs;
  color is never the only distinction.
- Free, reserved, unknown, and atlas-unobserved encodings are visually and
  textually distinct.
- The legend includes full text labels and the current evidence-layer meaning.
  Unknown and unavailable are first-class legend entries.

These rules follow WCAG's requirement that color not be the sole means of
conveying information. They also make grayscale screenshots and PR evidence
usable.

### Details and filtering

Hover or selection shows region/atlas label, relative offset, byte size,
storage state, logical use, semantic category, specific diagnostic label,
evidence layer, source, and limitation. Exact region details may also show
alignment, page crossing, and cache-line crossing as derived facts.

Filters can isolate semantic categories, allocator state, evidence quality, or
region source. Filtering changes visibility, not offsets or the underlying
ordering. Text summaries remain available for keyboard and non-visual use.

Snapshot differences are valid only when schema, engine, World generation,
scope, region identity, and size compatibility checks pass. Relative offsets,
not process addresses, are compared. An incompatible region is reported as
added/removed or unavailable rather than force-aligned.

## Performance And Measurement Boundaries

Diagnostics remain runtime opt-in and off by default. The disabled panel must
return before OS probes, allocator walks, ECS/object traversal, snapshot
allocation, address sorting, formatting, or rendering-model construction.

An enabled capture may be linear in the observed regions, allocator blocks,
known component pages, and atlas objects. Capture cadence is independent from
the simulation step, display geometry is bounded, and current layout payload is
not retained in the history ring. The collector runs at a safe World boundary;
it does not race allocator mutation or retain pointers after collection.

Access tracing and cache measurement are optional, separate evidence sources:

- Linux `perf mem` can statistically sample load/store addresses and, where
  supported, report data source, latency, TLB, cache-line, physical-address, or
  page-size fields.
- Intel VTune Memory Access analysis can use hardware-event sampling and map
  samples to dynamic memory objects when allocation instrumentation is enabled.
  Dynamic object tracking has overhead and should not silently turn on with
  the layout panel.
- PAPI preset event names are portable vocabulary, not a guarantee that a
  processor exposes or permits every cache/TLB counter.

Unavailable events, unsupported processors, kernel restrictions, missing
permissions, and absent tools are reported as unavailable with a reason. They
must not become zero values or layout-derived estimates.

Any cache or latency result recorded with the diagnostic needs the executable
and build configuration, CPU, event names, collection tool/version, command or
configuration, workload/scene, sampling interval, warm-up policy, and run
count. A locality optimization claim also needs a controlled end-to-end timing
comparison; a prettier map or a lower address gap is not a performance result.

## Security And Lifetime

Raw virtual addresses can disclose ASLR and process-layout information. The
default UI, screenshots, logs, baseline snapshots, and exported diagnostics use
region- or atlas-window-relative offsets only. Raw bases remain transient
inside the collector and are erased before publication.

Specific labels are curated diagnostic roles, not mangled C++ type names,
pointer values, user-data contents, or allocator-header dumps. The collector
must validate allocator metadata before traversing it and fail the affected
region closed as unavailable if invariants do not hold. It must never
dereference an atlas observation merely to discover its contents.

A future developer-only raw-address export would require a separate explicit
security decision. It is not an option hidden behind a tooltip or screenshot
mode.

## Realization Order

This ordering describes dependency boundaries, not active project status:

1. **Shared model and rasterizer.** Replace grouped logical cells with the v2
   region/atlas model, byte-range validation, zoom aggregation, mixed-cell
   composition, and fixture tests. Keep branch-neutral file names; the branch
   already identifies DART 6 versus DART 7.
2. **Allocator snapshot substrate.** Add non-mutating capture of free-list
   backing chunks and frame primary/overflow regions without changing allocator
   object layout or the disabled allocation path.
3. **DART 7 provider.** Compose allocator ranges with known ECS pages, entity
   buffers, and DART-owned dense buffers behind the opaque World tracker.
4. **DART 6 provider.** Reuse exact allocator maps for reservation arenas and
   add the separately labeled classic-graph address atlas without public layout
   or ABI changes.
5. **Optional access evidence.** Add import/adaptation seams for named PMU or
   simulator output only after the allocation-layout view is independently
   correct. Keep access heat and cache events as separate overlays with their
   own evidence labels.

No slice may block a truthful partial snapshot on complete semantic tagging.
Exact unknown allocations are more useful than guessed categories.

## Verification Contract

Correctness is proved in the data model before it is judged from a screenshot.

| Concern                        | Required evidence                                                                                                                                                                                         |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Exact region partition         | Deterministic allocator fixtures with multiple chunks, alignment padding, allocation/deallocation, splitting, merging, expansion, and overflow; assert byte coverage, order, containment, and no overlap. |
| Logical versus allocator state | Fixtures where component tombstones and spare page slots remain allocated while free-list holes are independently free.                                                                                   |
| DART 7 paging                  | Known multi-page component storage and separate packed-entity buffers appear as their actual ranges; no synthetic single component slab is emitted.                                                       |
| DART 6 truthfulness            | Reservation arenas and graph atlas render in separate sections; large address gaps remain unobserved; shallow estimates and point observations are labeled; no atlas fragmentation metric exists.         |
| Sanitization                   | Published DTOs, text output, and captured UI contain no absolute pointer or raw base address.                                                                                                             |
| Aggregation                    | Boundary-heavy fixtures remain ordered at every zoom; mixed cells preserve composition and exact details; arithmetic overflow and zero-size inputs fail safely.                                           |
| Accessibility                  | Legend, patterns/borders, grayscale inspection, narrow/high-DPI layouts, keyboard-readable text details, and category filtering are exercised.                                                            |
| Disabled overhead              | The disabled path performs no OS probe, World/ECS/object walk, snapshot formatting, or diagnostic history allocation. Existing no-allocation-after-bake gates remain unchanged.                           |
| Visual result                  | Reproducible headless UI captures show multiple regions, typed categories, unknown ranges, free/reserved distinctions, discontinuities, and hover/selection details at a readable scale.                  |
| Cache claim                    | A named hardware-counter or simulator artifact plus controlled timing evidence. If unavailable, the UI says unavailable and makes no cache-miss claim.                                                    |

Implementation changes follow [`../ai/verification.md`](../ai/verification.md),
the affected branch's build and focused test gates, GUI capture inspection, API
boundary checks, and two independent clean review passes. A screenshot proves
presentation only; allocator/model tests prove range correctness.

## Alternatives And Decisions

| Alternative                                                            | Decision          | Reason                                                                                                                                       |
| ---------------------------------------------------------------------- | ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| Keep the grouped active/hole/reserved grid and improve its labels      | Reject            | It has no address or byte-order evidence and answers a different question.                                                                   |
| Synthesize one global World arena                                      | Reject            | Both branches have multiple regions; DART 6's classic graph is not owned by its MemoryManager.                                               |
| Draw every address gap as free memory                                  | Reject            | Only the owning allocator can prove a free range. Atlas gaps are unobserved process address space.                                           |
| Treat packed ECS holes as allocator fragmentation                      | Reject            | A tombstone can occupy allocated page bytes; logical inactivity and allocator free state are orthogonal.                                     |
| Infer fewer cache misses from adjacency or SoA-looking colors          | Reject            | Cache behavior depends on the access stream, working set, hardware, and runtime conditions; it needs layer-5 evidence.                       |
| Expose raw addresses for exactness                                     | Reject by default | Region-relative offsets preserve geometry without publishing process-layout information.                                                     |
| Install a global `malloc`/`new` hook for the default panel             | Reject            | Intrusive interception changes overhead and attribution risk, crosses third-party ownership, and is unnecessary for allocator-owned regions. |
| Force DART 6 graph storage into the new arena as part of visualization | Reject            | That is a separate architecture and compatibility change, not diagnostic instrumentation.                                                    |
| Require complete type tagging before showing a map                     | Reject            | Exact unknown ranges are truthful; guessed labels are not. Category coverage can improve incrementally.                                      |
| Use one renderer-specific ImGui data model                             | Reject            | Collection and semantics must remain testable headlessly and preserve the DART GUI backend boundary.                                         |

## Primary-Source Basis

| Source                                                                                                                                                                            | Design implication                                                                                                                                                                                                                                    |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [MuJoCo simulation framework](https://mujoco.readthedocs.io/en/3.3.1/programming/simulation.html)                                                                                 | `mjModel` and `mjData` partition large buffers into typed arrays with alignment and padding; the runtime arena has explicit opposing allocation roles. DART should expose real ranges and padding, not infer contiguity from adjacent pointer fields. |
| [EnTT entity-component-system documentation](https://github.com/skypjack/entt/wiki/Entity-Component-System)                                                                       | Default component storage is paged and deletion policies can leave tombstones. DART must report actual pages and keep logical holes separate from allocator free bytes.                                                                               |
| [Unity Memory Profiler memory map](https://docs.unity.cn/Packages/com.unity.memoryprofiler%400.2/manual/memory-map)                                                               | An effective memory map preserves virtual-address order, supports adjustable row scale and filters, shows discontinuities, provides range details, and supports compatible snapshot comparison.                                                       |
| [AMD Radeon Memory Visualizer](https://gpuopen.com/manuals/rmv_manual/)                                                                                                           | Heap, allocation, and resource views are distinct; filtering, snapshots, comparison, aliasing cues, and resource naming help retain evidence rather than collapsing everything into utilization totals.                                               |
| [NVIDIA PhysX simulation documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/docs/Simulation.html)                                                                | Allocation callbacks can carry semantic type/source labels, while scratch and contact blocks have explicit roles. Tags are useful when the owning system supplies them, but do not justify guessing labels after the fact.                            |
| [LLVM `BumpPtrAllocatorImpl`](https://llvm.org/docs/doxygen/classllvm_1_1BumpPtrAllocatorImpl.html)                                                                               | A bump allocator can span multiple slabs and route large allocations to separate slabs. The word "arena" does not prove one unlimited contiguous region.                                                                                              |
| [Intel VTune Memory Access analysis](https://www.intel.com/content/www/us/en/docs/vtune-profiler/user-guide/2024-0/memory-access-analysis.html)                                   | Cache misses, memory latency, remote access, and dynamic-object attribution come from hardware sampling and optional allocation instrumentation, not from allocation geometry.                                                                        |
| [Linux `perf mem`](https://man7.org/linux/man-pages/man1/perf-mem.1.html)                                                                                                         | Load/store address samples can include cache-line, data-source, latency, TLB, physical-address, and page-size fields where the platform supports them. This is a separate sampled evidence layer.                                                     |
| [Intel vectorization layout guidance](https://www.intel.com/content/www/us/en/docs/dpcpp-cpp-compiler/developer-guide-reference/2023-0/vectorization-programming-guidelines.html) | AoS versus SoA benefit depends on the fields and stride of the real access pattern. A layout map can motivate a hypothesis but cannot choose the faster representation alone.                                                                         |
| [PAPI preset events](https://icl.utk.edu/projects/papi/presets.html)                                                                                                              | Cache and TLB event names offer a common vocabulary, but event availability remains processor-dependent and must be reported honestly.                                                                                                                |
| [WCAG 2.2 use of color](https://www.w3.org/WAI/WCAG22/Understanding/use-of-color)                                                                                                 | Category and state distinctions need text, pattern, border, or shape cues in addition to color.                                                                                                                                                       |

## Related Owners

- [`hierarchical_allocator.md`](hierarchical_allocator.md): DART 7 World memory
  ownership, allocator roles, and no-allocation-after-bake contract.
- [`dart7_architecture_assessment.md`](dart7_architecture_assessment.md): current
  physical data-layout gaps and the future dense Model/State direction.
- [`demos_app.md`](demos_app.md): diagnostics panel ownership and renderer
  boundary.
- [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md):
  `PanelBuilder`, Dear ImGui, capture, and visual verification rules.
- [`../onboarding/api-boundaries.md`](../onboarding/api-boundaries.md): public
  facade, internal storage, and DART 6 compatibility policy.
- [`../onboarding/profiling.md`](../onboarding/profiling.md): text-first World
  profiling surfaces that complement, but do not replace, memory evidence.
