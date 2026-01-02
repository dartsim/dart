/*
 * Copyright (c) 2011-2025, The DART development contributors
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#ifndef DART_SIMULATION_WORLD_HPP_
#define DART_SIMULATION_WORLD_HPP_

#include <dart/simulation/Fwd.hpp>
#include <dart/simulation/Recording.hpp>
#include <dart/simulation/solver/SolverTypes.hpp>

#include <dart/constraint/Fwd.hpp>

#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/Fwd.hpp>

#include <dart/dynamics/Fwd.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/NameManager.hpp>
#include <dart/common/SmartPointer.hpp>
#include <dart/common/Subject.hpp>

#include <dart/Export.hpp>
#include <dart/sensor/SensorManager.hpp>

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace dart {
namespace simulation {

namespace detail {
struct WorldEcsAccess;
} // namespace detail

class Solver;

/// Available collision detector backends for a World.
enum class CollisionDetectorType : int
{
  Dart,
  Fcl,
  Bullet,
  Ode,
};

/// Configuration bundle used when constructing a World.
struct WorldConfig final
{
  /// Friendly name for the world.
  std::string name = "world";

  /// Preferred collision detector for the world.
  CollisionDetectorType collisionDetector = CollisionDetectorType::Fcl;

  /// Assigns which rigid solver owns each world object class.
  struct SolverRouting final
  {
    /// Solver responsible for classic Skeletons.
    RigidSolverType skeletons = RigidSolverType::ClassicSkeleton;

    /// Solver responsible for ECS-backed simulation objects.
    RigidSolverType objects = RigidSolverType::EntityComponent;
  };

  /// Routing configuration for Skeletons and ECS-backed objects.
  ///
  /// The first solver of the requested type in registration order is selected
  /// as the owner.
  SolverRouting solverRouting;

  WorldConfig() = default;
  explicit WorldConfig(std::string worldName) : name(std::move(worldName)) {}
};

/// class World
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
class DART_API World : public virtual common::Subject
{
public:
  using NameChangedSignal = common::Signal<void(
      const std::string& _oldName, const std::string& _newName)>;

  /// Creates World as shared_ptr
  template <typename... Args>
  static WorldPtr create(Args&&... args);

  //--------------------------------------------------------------------------
  // Constructor and Destructor
  //--------------------------------------------------------------------------

  /// Creates a World
  static std::shared_ptr<World> create(const std::string& name = "world");

  /// Creates a World using a configuration bundle.
  static std::shared_ptr<World> create(const WorldConfig& config);

  /// Constructor
  World(const std::string& _name = "world");

  /// Constructor with configuration bundle
  explicit World(const WorldConfig& config);

  /// Destructor
  virtual ~World();

  /// Create a clone of this World. All Skeletons and SimpleFrames that are held
  /// by this World will be copied over.
  std::shared_ptr<World> clone() const;

  //--------------------------------------------------------------------------
  // Properties
  //--------------------------------------------------------------------------

  /// Set the name of this World
  const std::string& setName(const std::string& _newName);

  /// Get the name of this World
  const std::string& getName() const;

  /// Set gravity
  void setGravity(const Eigen::Vector3d& _gravity);

  /// Set gravity
  void setGravity(double x, double y, double z);

  /// Get gravity
  const Eigen::Vector3d& getGravity() const;

  /// Set time step. Invalid (<= 0 or non-finite) values are ignored.
  void setTimeStep(double _timeStep);

  /// Get time step
  double getTimeStep() const;

  //--------------------------------------------------------------------------
  // Structural Properties
  //--------------------------------------------------------------------------

  /// Get the indexed skeleton
  dynamics::SkeletonPtr getSkeleton(std::size_t _index) const;

  /// Find a Skeleton by name
  /// @param[in] _name The name of the Skeleton you are looking for.
  /// @return If the skeleton does not exist then return nullptr.
  dynamics::SkeletonPtr getSkeleton(const std::string& _name) const;

  /// Get the number of skeletons
  std::size_t getNumSkeletons() const;

  /// Add a skeleton to this world
  std::string addSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// Remove a skeleton from this world
  void removeSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// Remove all the skeletons in this world, and return a set of shared
  /// pointers to them, in case you want to recycle them
  std::set<dynamics::SkeletonPtr> removeAllSkeletons();

  /// Returns whether this World contains a Skeleton.
  bool hasSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const;

  /// Returns whether this World contains a Skeleton named @c skeletonName.
  bool hasSkeleton(const std::string& skeletonName) const;

  /// Get the dof index for the indexed skeleton
  int getIndex(int _index) const;

  /// Get the indexed Entity
  dynamics::SimpleFramePtr getSimpleFrame(std::size_t _index) const;

  /// Find an Entity by name
  dynamics::SimpleFramePtr getSimpleFrame(const std::string& _name) const;

  /// Get the number of Entities
  std::size_t getNumSimpleFrames() const;

  /// Add an Entity to this world
  std::string addSimpleFrame(const dynamics::SimpleFramePtr& _frame);

  /// Remove a SimpleFrame from this world
  void removeSimpleFrame(const dynamics::SimpleFramePtr& _frame);

  /// Remove all SimpleFrames in this world, and return a set of shared
  /// pointers to them, in case you want to recycle them
  std::set<dynamics::SimpleFramePtr> removeAllSimpleFrames();

  //--------------------------------------------------------------------------
  // Sensor Properties
  //--------------------------------------------------------------------------

  /// Get the indexed sensor
  sensor::SensorPtr getSensor(std::size_t index) const;

  /// Find a sensor by name
  sensor::SensorPtr getSensor(const std::string& name) const;

  /// Get the number of sensors
  std::size_t getNumSensors() const;

  /// Add a sensor to this world
  std::string addSensor(const sensor::SensorPtr& sensor);

  /// Remove a sensor from this world
  void removeSensor(const sensor::SensorPtr& sensor);

  /// Remove all sensors in this world, and return a set of shared pointers to
  /// them, in case you want to recycle them
  std::set<sensor::SensorPtr> removeAllSensors();

  /// Returns whether this World contains a sensor.
  bool hasSensor(const sensor::SensorPtr& sensor) const;

  /// Returns whether this World contains a sensor named @c sensorName.
  bool hasSensor(const std::string& sensorName) const;

  /// Get the sensor manager.
  sensor::SensorManager& getSensorManager();

  /// Get the sensor manager (const).
  const sensor::SensorManager& getSensorManager() const;

  //--------------------------------------------------------------------------
  // Collision checking
  //--------------------------------------------------------------------------

  /// Perform collision checking with 'option' over all the feasible collision
  /// pairs in this World, and the result will be stored 'result'. If no
  /// argument is passed in then it will return just whether there is collision
  /// or not without the contact information such as contact point, normal, and
  /// penetration depth.
  bool checkCollision(
      const collision::CollisionOption& option
      = collision::CollisionOption(false, 1u, nullptr),
      collision::CollisionResult* result = nullptr);

  /// Return the collision checking result of the last simulation step. If this
  /// world hasn't stepped forward yet, then the result would be empty. Note
  /// that this function does not return the collision checking result of
  /// World::checkCollision().
  const collision::CollisionResult& getLastCollisionResult() const;

  /// Sets the collision detector used by the world's constraint solver.
  void setCollisionDetector(
      const collision::CollisionDetectorPtr& collisionDetector);

  /// Sets the collision detector via a typed backend selection.
  void setCollisionDetector(CollisionDetectorType collisionDetector);

  /// Returns the collision detector used by the world.
  collision::CollisionDetectorPtr getCollisionDetector();

  /// Returns the collision detector used by the world (const).
  collision::ConstCollisionDetectorPtr getCollisionDetector() const;

  //--------------------------------------------------------------------------
  // Simulation
  //--------------------------------------------------------------------------

  /// Reset the time, frame counter and recorded histories.
  ///
  /// This also clears constraint impulses on all Skeletons in the World so
  /// stale constraint forces do not leak across independent simulation runs.
  void reset();

  /// Calculate the dynamics and integrate the world for one step
  /// @param[in] _resetCommand True if you want to reset to zero the joint
  /// command after simulation step.
  void step(bool _resetCommand = true);

  /// Set current time
  void setTime(double _time);

  /// Get current time
  double getTime() const;

  /// Get the number of simulated frames
  ///
  /// TODO(MXG): I think the name of this function is much too similar to
  /// getSimpleFrame()
  int getSimFrames() const;

  //--------------------------------------------------------------------------
  // Constraint
  //--------------------------------------------------------------------------

  /// Sets the constraint solver
  ///
  /// Note that the internal properties of @c solver will be overwritten by this
  /// World.
  void setConstraintSolver(constraint::UniqueConstraintSolverPtr solver);

  /// Get the constraint solver
  constraint::ConstraintSolver* getConstraintSolver();

  /// Get the constraint solver
  const constraint::ConstraintSolver* getConstraintSolver() const;

  /// Bake simulated current state and store it into mRecording
  void bake();

  /// Get recording
  Recording* getRecording();

  /// \{ @name Iterations

  /// Iterates all the Skeletons and invokes the callback function.
  ///
  /// Example:
  /// @code{.cpp}
  /// bodyNode->eachSkeleton([](const Skeleton* skel) {
  ///   // ...
  /// });
  ///
  /// bodyNode->eachSkeleton([](const Skeleton* skel) -> bool {
  ///   if (!skel->getName() == "name")
  ///   {
  ///     // to stop iterating when found a ShapeNode with no VisualAspect
  ///     return false;
  ///   }
  ///   return true;
  /// });
  /// @endcode
  ///
  /// @tparam Func: The callback function type. The function signature should be
  /// equivalent to @c void(const Skeleton*) or @c bool(const Skeleton*). If
  /// you want to conditionally iterate, use @c bool(const Skeleton*) and
  /// return false when to stop iterating.
  ///
  /// @param[in] func: The callback function to be called for each Skeleton.
  template <typename Func>
  void eachSkeleton(Func func) const;

  /// Iterates all the Skeletons and invokes the callback function.
  ///
  /// @tparam Func: The callback function type. The function signature should be
  /// equivalent to @c void(Skeleton*) or @c bool(Skeleton*). If
  /// you want to conditionally iterate, use @c bool(Skeleton*) and
  /// return false when to stop iterating.
  ///
  /// @param[in] func: The callback function to be called for each Skeleton.
  template <typename Func>
  void eachSkeleton(Func func);

  /// Iterates all the SimpleFrames and invokes the callback function.
  ///
  /// @tparam Func: The callback function type. The function signature should be
  /// equivalent to @c void(const SimpleFrame*) or @c bool(const SimpleFrame*).
  /// If you want to conditionally iterate, use @c bool(const SimpleFrame*) and
  /// return false when to stop iterating.
  ///
  /// @param[in] func: The callback function to be called for each SimpleFrame.
  template <typename Func>
  void eachSimpleFrame(Func func) const;

  /// Iterates all the SimpleFrames and invokes the callback function.
  ///
  /// @tparam Func: The callback function type. The function signature should be
  /// equivalent to @c void(SimpleFrame*) or @c bool(SimpleFrame*). If
  /// you want to conditionally iterate, use @c bool(SimpleFrame*) and
  /// return false when to stop iterating.
  ///
  /// @param[in] func: The callback function to be called for each SimpleFrame.
  template <typename Func>
  void eachSimpleFrame(Func func);

  /// \}

protected:
  friend struct detail::WorldEcsAccess;

  /// Register when a Skeleton's name is changed
  void handleSkeletonNameChange(
      const dynamics::ConstMetaSkeletonPtr& _skeleton);

  /// Register when a SimpleFrame's name is changed
  void handleSimpleFrameNameChange(const dynamics::Entity* _entity);

  Solver* getConstraintCapableSolver();
  const Solver* getConstraintCapableSolver() const;
  Solver* getCollisionCapableSolver();
  const Solver* getCollisionCapableSolver() const;

  /// Returns the solver configured to handle Skeletons, or nullptr.
  Solver* getSkeletonSolver();

  /// Returns the solver configured to handle Skeletons, or nullptr.
  const Solver* getSkeletonSolver() const;

  /// Returns the solver configured to handle ECS-backed objects, or nullptr.
  Solver* getObjectSolver();

  /// Returns the solver configured to handle ECS-backed objects, or nullptr.
  const Solver* getObjectSolver() const;

  //--------------------------------------------------------------------------
  // Solver & ECS internals
  //--------------------------------------------------------------------------

  /// Adds a solver to this world, taking ownership.
  Solver* addSolver(std::unique_ptr<Solver> solver);

  /// Adds a solver to this world with an initial enabled state.
  ///
  /// Disabled solvers still receive structural notifications when they are
  /// configured to own those objects (e.g., Skeletons or ECS objects), but
  /// they are skipped when stepping and when resolving World APIs that
  /// require a solver backend.
  Solver* addSolver(std::unique_ptr<Solver> solver, bool enabled);

  /// Returns the number of solvers registered with this world.
  std::size_t getNumSolvers() const;

  /// Returns the indexed solver.
  Solver* getSolver(std::size_t index);

  /// Returns the indexed solver (const).
  const Solver* getSolver(std::size_t index) const;

  /// Returns the index of the given solver, or getNumSolvers() if not found.
  std::size_t getSolverIndex(const Solver* solver) const;

  /// Returns the first rigid solver matching the given type, or nullptr.
  Solver* getSolver(RigidSolverType type);

  /// Returns the first rigid solver matching the given type, or nullptr.
  const Solver* getSolver(RigidSolverType type) const;

  /// Returns the first solver with the given name, or nullptr.
  Solver* getSolver(const std::string& name);

  /// Returns the first solver with the given name, or nullptr.
  const Solver* getSolver(const std::string& name) const;

  /// Returns the enabled solver configured for Skeletons, or nullptr.
  Solver* getActiveRigidSolver();

  /// Returns the enabled solver configured for Skeletons, or nullptr.
  const Solver* getActiveRigidSolver() const;

  /// Enables or disables a solver by index.
  bool setSolverEnabled(std::size_t index, bool enabled);

  /// Returns whether the indexed solver is enabled.
  bool isSolverEnabled(std::size_t index) const;

  /// Enables or disables a solver by pointer.
  bool setSolverEnabled(Solver* solver, bool enabled);

  /// Returns whether the given solver is enabled.
  bool isSolverEnabled(const Solver* solver) const;

  /// Moves a solver within the execution order.
  bool moveSolver(std::size_t fromIndex, std::size_t toIndex);

  /// Name of this World
  std::string mName;

  /// Skeletons in this world
  std::vector<dynamics::SkeletonPtr> mSkeletons;

  std::map<dynamics::ConstMetaSkeletonPtr, dynamics::SkeletonPtr>
      mMapForSkeletons;

  /// Connections for noticing changes in Skeleton names
  /// TODO(MXG): Consider putting this functionality into NameManager
  std::vector<common::Connection> mNameConnectionsForSkeletons;

  /// NameManager for keeping track of Skeletons
  dart::common::NameManager<dynamics::SkeletonPtr> mNameMgrForSkeletons;

  /// Entities in this world
  std::vector<dynamics::SimpleFramePtr> mSimpleFrames;

  /// Connections for noticing changes in Frame names
  /// TODO(MXG): Consider putting this functionality into NameManager
  std::vector<common::Connection> mNameConnectionsForSimpleFrames;

  /// Map from raw SimpleFrame pointers to their shared_ptrs
  std::map<const dynamics::SimpleFrame*, dynamics::SimpleFramePtr>
      mSimpleFrameToShared;

  /// NameManager for keeping track of Entities
  dart::common::NameManager<dynamics::SimpleFramePtr> mNameMgrForSimpleFrames;

  /// Manager for sensors in this world
  sensor::SensorManager mSensorManager;

  /// The first indeices of each skeleton's dof in mDofs
  ///
  /// For example, if this world has three skeletons and their dof are
  /// 6, 1 and 2 then the mIndices goes like this: [0 6 7].
  std::vector<int> mIndices;

  /// Gravity
  Eigen::Vector3d mGravity;

  /// Simulation time step
  double mTimeStep;

  /// Current simulation time
  double mTime;

  /// Current simulation frame number
  int mFrame;

  /// Recording buffer for baked states
  Recording* mRecording;

private:
  struct EcsData;
  std::unique_ptr<EcsData> mEcsData;

protected:
  struct SolverEntry final
  {
    std::unique_ptr<Solver> solver;
    bool enabled{true};
  };

  /// Collection of solvers registered with the world.
  std::vector<SolverEntry> mSolvers;

  /// Solver type configured to handle classic Skeletons.
  RigidSolverType mSkeletonSolverType{RigidSolverType::ClassicSkeleton};

  /// Solver type configured to handle ECS-backed objects.
  RigidSolverType mObjectSolverType{RigidSolverType::EntityComponent};

  //--------------------------------------------------------------------------
  // Signals
  //--------------------------------------------------------------------------
  NameChangedSignal mNameChangedSignal;

public:
  //--------------------------------------------------------------------------
  // Slot registers
  //--------------------------------------------------------------------------
  common::SlotRegister<NameChangedSignal> onNameChanged;
};
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END

} // namespace simulation
} // namespace dart

#include <dart/simulation/detail/World-impl.hpp>

#endif // DART_SIMULATION_WORLD_HPP_
