/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "simulation/module_detail.hpp"

namespace dart::python_nb {

// compute executors/profiles and ECS/memory diagnostics
void defSimPartCompute(nb::module_& m)
{
  nb::class_<sim::compute::ComputeExecutor>(m, "ComputeExecutor")
      .def_prop_ro(
          "worker_count",
          &sim::compute::ComputeExecutor::getWorkerCount,
          "Number of workers exposed by this executor.");

  nb::class_<sim::compute::SequentialExecutor, sim::compute::ComputeExecutor>(
      m, "SequentialExecutor")
      .def(nb::init<>(), "Create the reference sequential compute executor.");

  nb::class_<sim::compute::ParallelExecutor, sim::compute::ComputeExecutor>(
      m, "ParallelExecutor")
      .def(
          nb::init<std::size_t>(),
          nb::arg("worker_count") = 0,
          "Create a parallel compute executor. Zero lets Taskflow choose the "
          "worker count.")
      .def_prop_rw(
          "inline_threshold",
          &sim::compute::ParallelExecutor::getInlineThreshold,
          &sim::compute::ParallelExecutor::setInlineThreshold,
          "Graphs with at most this many nodes execute inline.");

  nb::class_<sim::compute::ComputeNodeExecutionProfile>(
      m, "ComputeNodeExecutionProfile")
      .def_ro(
          "name",
          &sim::compute::ComputeNodeExecutionProfile::name,
          "Compute node name.")
      .def_ro(
          "topological_index",
          &sim::compute::ComputeNodeExecutionProfile::topologicalIndex,
          "Index in the graph's topological order.")
      .def_ro(
          "dependency_count",
          &sim::compute::ComputeNodeExecutionProfile::dependencyCount,
          "Number of incoming dependencies.")
      .def_ro(
          "dependent_count",
          &sim::compute::ComputeNodeExecutionProfile::dependentCount,
          "Number of outgoing dependents.")
      .def_ro(
          "level",
          &sim::compute::ComputeNodeExecutionProfile::level,
          "Longest dependency depth from any source node.")
      .def_ro(
          "worker_index",
          &sim::compute::ComputeNodeExecutionProfile::workerIndex,
          "Compact index of the worker thread that ran this node.")
      .def_prop_ro(
          "start_time_us",
          [](const sim::compute::ComputeNodeExecutionProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.startTime)
                .count();
          },
          "Node start time relative to graph execution start, in microseconds.")
      .def_prop_ro(
          "end_time_us",
          [](const sim::compute::ComputeNodeExecutionProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.endTime)
                .count();
          },
          "Node finish time relative to graph execution start, in "
          "microseconds.")
      .def_prop_ro(
          "duration_us",
          [](const sim::compute::ComputeNodeExecutionProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.duration)
                .count();
          },
          "Time spent inside this node callable, in microseconds.")
      .def_prop_ro(
          "duration_ms",
          [](const sim::compute::ComputeNodeExecutionProfile& self) {
            return std::chrono::duration<double, std::milli>(self.duration)
                .count();
          },
          "Time spent inside this node callable, in milliseconds.");

  nb::class_<sim::compute::ComputeExecutionProfile>(
      m, "ComputeExecutionProfile")
      .def_ro(
          "worker_count",
          &sim::compute::ComputeExecutionProfile::workerCount,
          "Number of workers exposed by the executor.")
      .def_ro(
          "max_parallelism",
          &sim::compute::ComputeExecutionProfile::maxParallelism,
          "Largest number of node callables observed running concurrently.")
      .def_ro(
          "nodes",
          &sim::compute::ComputeExecutionProfile::nodes,
          "Per-node execution records, sorted by topological index.")
      .def_prop_ro(
          "wall_time_us",
          [](const sim::compute::ComputeExecutionProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.wallTime)
                .count();
          },
          "End-to-end graph execution time, in microseconds.")
      .def_prop_ro(
          "wall_time_ms",
          [](const sim::compute::ComputeExecutionProfile& self) {
            return std::chrono::duration<double, std::milli>(self.wallTime)
                .count();
          },
          "End-to-end graph execution time, in milliseconds.")
      .def_prop_ro(
          "total_node_time_us",
          [](const sim::compute::ComputeExecutionProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.totalNodeTime)
                .count();
          },
          "Sum of node callable durations, in microseconds.")
      .def_prop_ro(
          "critical_path_time_us",
          [](const sim::compute::ComputeExecutionProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.criticalPathTime)
                .count();
          },
          "Longest measured dependency path through the graph, in "
          "microseconds.")
      .def_prop_ro(
          "average_parallelism",
          &sim::compute::ComputeExecutionProfile::getAverageParallelism,
          "Total node time divided by graph wall time.")
      .def("is_empty", &sim::compute::ComputeExecutionProfile::isEmpty)
      .def(
          "get_node",
          [](const sim::compute::ComputeExecutionProfile& self,
             const std::string& name)
              -> std::optional<sim::compute::ComputeNodeExecutionProfile> {
            const auto* node = self.getNode(name);
            if (node == nullptr) {
              return std::nullopt;
            }
            return *node;
          },
          nb::arg("name"),
          "Returns a node profile copy with the given name, or None.")
      .def(
          "summary",
          &sim::compute::ComputeExecutionProfile::toSummaryText,
          "Compact, sorted, human- and agent-readable graph timing table.")
      .def("__repr__", &sim::compute::ComputeExecutionProfile::toSummaryText)
      .def("__str__", &sim::compute::ComputeExecutionProfile::toSummaryText);

  nb::class_<sim::compute::WorldStepStageProfile>(m, "WorldStepStageProfile")
      .def_ro(
          "name",
          &sim::compute::WorldStepStageProfile::name,
          "Stage name, e.g. \"rigid_body_contact\".")
      .def_prop_ro(
          "domain",
          [](const sim::compute::WorldStepStageProfile& self) {
            return std::string(sim::compute::toString(self.domain));
          },
          "Broad compute domain of the stage, e.g. \"rigid_body\".")
      .def_prop_ro(
          "acceleration",
          [](const sim::compute::WorldStepStageProfile& self) {
            return sim::compute::formatAccelerationMask(self.acceleration);
          },
          "Acceleration opportunities advertised by the stage metadata.")
      .def_ro(
          "accelerated_backend_enabled",
          &sim::compute::WorldStepStageProfile::acceleratedBackendEnabled,
          "Whether a backend-neutral accelerated implementation was active "
          "while this stage ran.")
      .def_ro(
          "graph_profiles",
          &sim::compute::WorldStepStageProfile::graphProfiles,
          "Compute graph profiles captured inside this stage.")
      .def_prop_ro(
          "duration_us",
          [](const sim::compute::WorldStepStageProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.duration)
                .count();
          },
          "Wall-clock time spent in this stage, in microseconds.")
      .def_prop_ro(
          "duration_ms",
          [](const sim::compute::WorldStepStageProfile& self) {
            return std::chrono::duration<double, std::milli>(self.duration)
                .count();
          },
          "Wall-clock time spent in this stage, in milliseconds.")
      .def_prop_ro(
          "total_graph_wall_time_us",
          [](const sim::compute::WorldStepStageProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.totalGraphWallTime())
                .count();
          },
          "Sum of nested compute graph wall times, in microseconds.")
      .def_prop_ro(
          "max_graph_worker_count",
          &sim::compute::WorldStepStageProfile::maxGraphWorkerCount,
          "Largest executor worker count among nested graph profiles.")
      .def_prop_ro(
          "max_graph_parallelism",
          &sim::compute::WorldStepStageProfile::maxGraphParallelism,
          "Largest observed parallelism among nested graph profiles.");

  nb::class_<sim::compute::WorldStepProfile>(m, "WorldStepProfile")
      .def_ro(
          "step_count",
          &sim::compute::WorldStepProfile::stepCount,
          "Number of steps captured (1 for the most recent step).")
      .def_ro(
          "stages",
          &sim::compute::WorldStepProfile::stages,
          "Per-stage timings, in pipeline execution order.")
      .def_prop_ro(
          "wall_time_us",
          [](const sim::compute::WorldStepProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.wallTime)
                .count();
          },
          "End-to-end wall time of the profiled step, in microseconds.")
      .def_prop_ro(
          "wall_time_ms",
          [](const sim::compute::WorldStepProfile& self) {
            return std::chrono::duration<double, std::milli>(self.wallTime)
                .count();
          },
          "End-to-end wall time of the profiled step, in milliseconds.")
      .def_prop_ro(
          "total_stage_time_us",
          [](const sim::compute::WorldStepProfile& self) {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       self.totalStageTime())
                .count();
          },
          "Sum of per-stage durations, in microseconds.")
      .def("is_empty", &sim::compute::WorldStepProfile::isEmpty)
      .def(
          "get_stage",
          [](const sim::compute::WorldStepProfile& self,
             const std::string& name)
              -> std::optional<sim::compute::WorldStepStageProfile> {
            const auto* stage = self.getStage(name);
            if (stage == nullptr) {
              return std::nullopt;
            }
            return *stage;
          },
          nb::arg("name"),
          "Returns a stage profile copy with the given name, or None.")
      .def(
          "summary",
          &sim::compute::WorldStepProfile::toSummaryText,
          "Compact, sorted, human- and agent-readable per-stage timing table.")
      .def("__repr__", &sim::compute::WorldStepProfile::toSummaryText)
      .def("__str__", &sim::compute::WorldStepProfile::toSummaryText);

  nb::class_<sim::compute::StepMetrics>(m, "StepMetrics")
      .def_ro(
          "kinetic_energy",
          &sim::compute::StepMetrics::kineticEnergy,
          "Total translational and rotational kinetic energy.")
      .def_ro(
          "potential_energy",
          &sim::compute::StepMetrics::potentialEnergy,
          "Total gravitational potential energy.")
      .def_ro(
          "total_energy",
          &sim::compute::StepMetrics::totalEnergy,
          "Kinetic plus potential energy.")
      .def_ro(
          "linear_momentum",
          &sim::compute::StepMetrics::linearMomentum,
          "Total linear momentum in the world frame.")
      .def_ro(
          "angular_momentum",
          &sim::compute::StepMetrics::angularMomentum,
          "Total angular momentum about the world origin.")
      .def_ro(
          "active_contact_count",
          &sim::compute::StepMetrics::activeContactCount,
          "Rigid contacts processed by the last mutating contact stage.")
      .def_ro(
          "max_penetration_depth",
          &sim::compute::StepMetrics::maxPenetrationDepth,
          "Maximum penetration depth reported by the last contact stage.")
      .def_ro(
          "last_step_iterations",
          &sim::compute::StepMetrics::lastStepIterations,
          "Solver iterations reported by the most recent World::step().")
      .def_ro(
          "last_step_residual",
          &sim::compute::StepMetrics::lastStepResidual,
          "Maximum final residual reported by the most recent World::step().")
      .def("__repr__", [](const sim::compute::StepMetrics& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("kinetic_energy", repr_double(self.kineticEnergy));
        fields.emplace_back(
            "potential_energy", repr_double(self.potentialEnergy));
        fields.emplace_back("total_energy", repr_double(self.totalEnergy));
        fields.emplace_back(
            "active_contact_count", std::to_string(self.activeContactCount));
        fields.emplace_back(
            "max_penetration_depth", repr_double(self.maxPenetrationDepth));
        fields.emplace_back(
            "last_step_iterations", std::to_string(self.lastStepIterations));
        fields.emplace_back(
            "last_step_residual", repr_double(self.lastStepResidual));
        return format_repr("StepMetrics", fields);
      });

  nb::class_<dart::common::MemoryManager::AllocatorDebugDiagnostics>(
      m, "AllocatorDebugDiagnostics")
      .def_ro(
          "live_bytes",
          &dart::common::MemoryManager::AllocatorDebugDiagnostics::liveBytes)
      .def_ro(
          "peak_live_bytes",
          &dart::common::MemoryManager::AllocatorDebugDiagnostics::
              peakLiveBytes)
      .def_ro(
          "live_allocation_count",
          &dart::common::MemoryManager::AllocatorDebugDiagnostics::
              liveAllocationCount);

  nb::class_<dart::common::MemoryManager::DebugDiagnostics>(
      m, "MemoryManagerDebugDiagnostics")
      .def_ro(
          "enabled", &dart::common::MemoryManager::DebugDiagnostics::enabled)
      .def_ro(
          "free_allocator",
          &dart::common::MemoryManager::DebugDiagnostics::freeAllocator)
      .def_ro(
          "pool_allocator",
          &dart::common::MemoryManager::DebugDiagnostics::poolAllocator);

  nb::class_<sim::WorldEcsStorageDiagnostics>(m, "WorldEcsStorageDiagnostics")
      .def_ro("storage_id", &sim::WorldEcsStorageDiagnostics::storageId)
      .def_ro("size", &sim::WorldEcsStorageDiagnostics::size)
      .def_ro("capacity", &sim::WorldEcsStorageDiagnostics::capacity);

  nb::class_<sim::WorldEcsDiagnostics>(m, "WorldEcsDiagnostics")
      .def_ro("entity_count", &sim::WorldEcsDiagnostics::entityCount)
      .def_ro("entity_capacity", &sim::WorldEcsDiagnostics::entityCapacity)
      .def_ro("storage_count", &sim::WorldEcsDiagnostics::storageCount)
      .def_ro("component_count", &sim::WorldEcsDiagnostics::componentCount)
      .def_ro(
          "component_capacity", &sim::WorldEcsDiagnostics::componentCapacity)
      .def_ro("storages", &sim::WorldEcsDiagnostics::storages);

  nb::class_<sim::WorldMemoryDiagnostics>(m, "WorldMemoryDiagnostics")
      .def_ro(
          "allocator_debug_diagnostics",
          &sim::WorldMemoryDiagnostics::allocatorDebugDiagnostics)
      .def_ro("ecs_diagnostics", &sim::WorldMemoryDiagnostics::ecsDiagnostics)
      .def_ro(
          "frame_scratch_capacity_bytes",
          &sim::WorldMemoryDiagnostics::frameScratchCapacityBytes)
      .def_ro(
          "frame_scratch_used_bytes",
          &sim::WorldMemoryDiagnostics::frameScratchUsedBytes)
      .def_ro(
          "frame_scratch_peak_used_bytes",
          &sim::WorldMemoryDiagnostics::frameScratchPeakUsedBytes)
      .def_ro(
          "frame_scratch_overflow_count",
          &sim::WorldMemoryDiagnostics::frameScratchOverflowCount)
      .def_ro(
          "frame_scratch_overflow_bytes",
          &sim::WorldMemoryDiagnostics::frameScratchOverflowBytes)
      .def_ro(
          "frame_scratch_reset_count",
          &sim::WorldMemoryDiagnostics::frameScratchResetCount);
}

} // namespace dart::python_nb
