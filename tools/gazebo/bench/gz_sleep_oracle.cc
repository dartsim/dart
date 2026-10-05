// Differential sleep oracle for the gz-physics dartsim plugin.
//
// DART puts resting bodies to sleep (automatic deactivation). Sleeping must not
// change what Gazebo sees, so a sleeping body has to wake whenever gz-physics
// changes something that moves it. For each scenario (one state mutation the
// dartsim plugin exposes, listed with --list), this driver runs the same
// script twice, with DART's deactivation enabled (the default) and disabled:
//   1. load a small SDF world through the dartsim plugin and apply gz-sim's
//      default per-pair contact limit,
//   2. step until it has settled (--settle steps),
//   3. apply the mutation (commands and wrenches are re-applied every step,
//      like gz-sim's systems do),
//   4. step --steps more,
// and compares, step by step, the link poses gz-physics published
// (ChangedWorldPoses, what gz-sim sees), and the contacts it reported on the
// first step after the mutation and at the end. A difference beyond --tolerance
// is a MISMATCH: sleeping changed the simulation. A pose or contact point that
// is not finite (a run blew up) always differs. The table also reports how
// many bodies were resting when the mutation was applied and how far the
// mutation moved the reference run. A mutation that cannot move a body at rest
// comes with a kick, so a stale parameter still shows; a third run applies the
// kick alone, and "moved" is then how far the mutation moved the kicked
// reference run away from it. A scenario with nothing asleep, or whose
// mutation moved nothing, is UNEXERCISED: it could not have revealed a missed
// wake. Scenarios whose body DART keeps awake by design (held by a joint
// constraint, or driven) are marked as such instead; they check that
// deactivation leaves awake bodies alone.
//
// Scenarios cover joint spring stiffness and reference, damping and friction
// (gz-physics 8 and later), position, velocity and effort limits, velocity
// commands, force, position and velocity, and the joint-to-child transform;
// link and model gravity flags; world gravity; free-group pose and linear
// velocity; link wrenches; model static and collision flags; collision and
// category masks; shape pose and attaching a shape; the per-pair contact
// limit; the collision detector; contact-properties callbacks; attaching fixed
// and revolute joints and detaching joints; removing and spawning models. They
// do not cover SetSolver, free-group angular velocity, prismatic joints, joint
// axes, friction-pyramid slip compliance or constructing empty entities.
//
// gz-physics installs its own BodyNodeCollisionFilter subclass in every world.
// --default-filter replaces it with DART's BodyNodeCollisionFilter through
// RetrieveWorld, which shows how sleeping behaves for DART users, and skips the
// scenarios that need gz-physics' filter (masks, spawning and removal update
// it).
//
// usage: gz_sleep_oracle <dartsim-plugin.so> [--scenario NAME] [--settle N]
//            [--steps N] [--tolerance X] [--default-filter]
//            [--allow-unexercised] [--list]
//
// Exit status: 0 when every scenario matches and was exercised, 1 on any
// mismatch or, without --allow-unexercised (for a build that does not put
// these bodies to sleep), on any unexercised scenario.

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionFilter.hpp>

#include <dart/dynamics/Skeleton.hpp>

#include <gz/math/Pose3.hh>
#include <gz/physics/BoxShape.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Gravity.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/Model.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/World.hh>
#include <gz/physics/config.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <cmath>
#include <cstdio>
#include <cstdlib>

// RetrieveWorld is the dartsim plugin's custom feature; gz-physics does not
// install its header, so it comes from the gz-physics source tree.
#include "World.hh"

namespace {

namespace physics = gz::physics;

using CommonFeatures = physics::FeatureList<
    physics::sdf::ConstructSdfWorld,
    physics::sdf::ConstructSdfModel,
    physics::RemoveModelFromWorld,
    physics::ForwardStep,
    physics::GetContactsFromLastStepFeature,
    physics::CollisionPairMaxContacts,
    physics::CollisionDetector,
    physics::Gravity,
    physics::GetModelFromWorld,
    physics::GetLinkFromModel,
    physics::GetJointFromModel,
    physics::GetShapeFromLink,
    physics::SetBasicJointState,
    physics::SetJointVelocityCommandFeature,
    physics::SetJointPositionLimitsFeature,
    physics::SetJointVelocityLimitsFeature,
    physics::SetJointEffortLimitsFeature,
    physics::AttachFixedJointFeature,
    physics::AttachRevoluteJointFeature,
    physics::DetachJointFeature,
    physics::SetJointTransformFromParentFeature,
    physics::SetJointTransformToChildFeature,
    physics::FindFreeGroupFeature,
    physics::SetFreeGroupWorldPose,
    physics::SetFreeGroupWorldVelocity,
    physics::AddLinkExternalForceTorque,
    physics::GravityEnabled,
    physics::ModelStaticState,
    physics::ModelCollisionEnabled,
    physics::CollisionFilterMaskFeature,
    physics::CategoryFilterMaskFeature,
    physics::SetShapeKinematicProperties,
    physics::AttachBoxShapeFeature,
    physics::SetContactPropertiesCallbackFeature,
    physics::dartsim::RetrieveWorld>;

#if GZ_PHYSICS_MAJOR_VERSION >= 8
// Joint dynamics setters arrived in gz-physics 8 (Ionic).
using Features = physics::FeatureList<
    CommonFeatures,
    physics::SetJointDampingCoefficientFeature,
    physics::SetJointFrictionFeature,
    physics::SetJointSpringStiffnessFeature,
    physics::SetJointSpringReferenceFeature>;
#else
using Features = CommonFeatures;
#endif

using WorldPtr = physics::World3dPtr<Features>;
using ContactPoint = physics::World3d<Features>::ContactPoint;
using PoseMap = std::map<std::size_t, gz::math::Pose3d>;

// gz-sim passes the SDF <max_contacts> default to SetCollisionPairMaxContacts.
constexpr std::size_t kGzSimCollisionPairMaxContacts = 20;
constexpr double kStepSize = 0.001;

struct Run
{
  WorldPtr world;
  dart::simulation::WorldPtr dartWorld;
  // EntityPtr is not assignable, so it is emplaced.
  std::optional<physics::FixedJoint3dPtr<Features>> attached;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;
  PoseMap poses;

  auto model(const std::string& name) const
  {
    return world->GetModel(name);
  }

  auto link(const std::string& modelName, const std::string& linkName) const
  {
    return world->GetModel(modelName)->GetLink(linkName);
  }

  auto hinge() const
  {
    return world->GetModel("hinge")->GetJoint("hinge");
  }

  // Welds the "box" link to the "other" link at their offset, as gz-sim's
  // DetachableJoint system does.
  void weldBoxToOther(const Eigen::Vector3d& offset)
  {
    attached.emplace(
        link("box", "link")->AttachFixedJoint(link("other", "link"), "weld"));
    Eigen::Isometry3d parentToChild = Eigen::Isometry3d::Identity();
    parentToChild.translation() = offset;
    (*attached)->SetTransformFromParent(parentToChild);
  }

  // Spawns a model the way gz-sim's UserCommands system does.
  void spawn(const std::string& modelSdf)
  {
    sdf::Root root;
    const auto errors = root.LoadSdfString(
        "<?xml version=\"1.0\"?><sdf version=\"1.7\">" + modelSdf + "</sdf>");
    if (!errors.empty() || !root.Model()) {
      std::fprintf(stderr, "cannot spawn %s\n", modelSdf.c_str());
      std::exit(1);
    }
    world->ConstructModel(*root.Model());
  }

  void step()
  {
    physics::ForwardStep::Output output;
    world->Step(output, state, input);
    for (const auto& worldPose :
         output.Get<physics::ChangedWorldPoses>().entries)
      poses[worldPose.body] = worldPose.pose;
  }
};

struct Scenario
{
  std::string name;
  std::string feature;
  std::string models;
  std::function<void(Run&)> setup;
  std::function<void(Run&)> mutate;
  std::function<void(Run&)> everyStep;
  // The mutation updates gz-physics' collision filter, which --default-filter
  // replaces.
  bool needsGazeboFilter = false;
  // DART never puts the body to sleep before the mutation: a joint constraint
  // holds it (DART only lets contact islands sleep) or it is driven.
  bool awakeByDesign = false;
  // Applied right after a mutation that cannot move a body at rest. The
  // mutation counts as exercised only if it moves the kicked run.
  std::function<void(Run&)> kick = nullptr;
};

// What gz-sim would see from one run, step by step.
struct Outcome
{
  // Every link's published position and orientation after each step.
  std::vector<std::vector<double>> poses;
  std::vector<std::size_t> contactCounts;
  std::vector<std::array<double, 3>> finalContacts;
  std::size_t resting = 0;
  std::size_t mobile = 0;

  void record(const Run& run)
  {
    std::vector<double>& values = poses.emplace_back();
    for (const auto& [id, pose] : run.poses) {
      values.insert(
          values.end(),
          {pose.Pos().X(),
           pose.Pos().Y(),
           pose.Pos().Z(),
           pose.Rot().W(),
           pose.Rot().X(),
           pose.Rot().Y(),
           pose.Rot().Z()});
    }
    contactCounts.push_back(run.world->GetContactsFromLastStep().size());
  }
};

//==============================================================================
std::string inertial(double mass, double x, double y, double z)
{
  const auto moment = [mass](double a, double b) {
    return std::to_string(mass * (a * a + b * b) / 12.0);
  };
  return "<inertial><mass>" + std::to_string(mass) + "</mass><inertia><ixx>"
         + moment(y, z) + "</ixx><iyy>" + moment(x, z) + "</iyy><izz>"
         + moment(x, y)
         + "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>";
}

std::string boxCollision(double x, double y, double z)
{
  return "<collision name=\"collision\"><geometry><box><size>"
         + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z)
         + "</size></box></geometry></collision>";
}

// A free 0.5 m cube, resting on the ground at z = 0.25.
std::string boxModel(
    const std::string& name,
    double x,
    double z = 0.25,
    bool isStatic = false,
    double mass = 1.0)
{
  return "<model name=\"" + name + "\"><static>" + (isStatic ? "true" : "false")
         + "</static><pose>" + std::to_string(x) + " 0 " + std::to_string(z)
         + " 0 0 0</pose><link name=\"link\">" + inertial(mass, 0.5, 0.5, 0.5)
         + boxCollision(0.5, 0.5, 0.5) + "</link></model>";
}

// A heavy base plate with a flap hinged about +y on its +x edge. The flap is
// rotated by `angle` from lying flat on the ground (negative lifts it), and
// that pose is joint position zero. `axis` adds SDF <axis> children.
std::string flapModel(double angle, const std::string& axis = "")
{
  const double x = 0.25 + 0.25 * std::cos(angle);
  const double z = 0.05 - 0.25 * std::sin(angle);
  return "<model name=\"hinge\"><link name=\"base\"><pose>0 0 0.05 0 0 0</pose>"
         + inertial(20.0, 0.5, 0.5, 0.1) + boxCollision(0.5, 0.5, 0.1)
         + "</link><link name=\"flap\"><pose>" + std::to_string(x) + " 0 "
         + std::to_string(z) + " 0 " + std::to_string(angle) + " 0</pose>"
         + inertial(1.0, 0.5, 0.5, 0.1) + boxCollision(0.5, 0.5, 0.1)
         + "</link><joint name=\"hinge\" type=\"revolute\"><parent>base"
           "</parent><child>flap</child><pose relative_to=\"flap\">-0.25 0 0 0 0"
           " 0</pose><axis><xyz>0 1 0</xyz>"
         + axis + "</axis></joint></model>";
}

std::string dynamics(
    double damping, double friction, double reference, double stiffness)
{
  return "<dynamics><damping>" + std::to_string(damping)
         + "</damping><friction>" + std::to_string(friction)
         + "</friction><spring_reference>" + std::to_string(reference)
         + "</spring_reference><spring_stiffness>" + std::to_string(stiffness)
         + "</spring_stiffness></dynamics>";
}

std::string worldSdf(const std::string& models)
{
  return "<?xml version=\"1.0\"?><sdf version=\"1.7\"><world name=\"oracle\">"
         "<physics name=\"1ms\" type=\"ignored\"><max_step_size>"
         + std::to_string(kStepSize)
         + "</max_step_size><real_time_factor>0</real_time_factor></physics>"
           "<gravity>0 0 -9.8</gravity><model name=\"ground\"><static>true"
           "</static><link name=\"link\"><collision name=\"collision\">"
           "<geometry><plane><normal>0 0 1</normal><size>100 100</size>"
           "</plane></geometry></collision></link></model>"
         + models + "</world></sdf>";
}

//==============================================================================
// One step of an upward push on the flap (applied before the next step), for
// mutations that are inert from rest.
void kickFlap(Run& run)
{
  run.link("hinge", "flap")
      ->AddExternalForce(Eigen::Vector3d(0.0, 0.0, 2000.0));
}

// One step of a +x push on the "other" box.
void pushOther(Run& run)
{
  run.link("other", "link")
      ->AddExternalForce(Eigen::Vector3d(2000.0, 0.0, 0.0));
}

// A surface-velocity callback like gz-sim's TrackController (conveyors).
void addConveyor(Run& run)
{
  run.world->AddContactPropertiesCallback(
      "conveyor", [](const auto&, std::size_t, auto& params) {
        params.firstFrictionalDirection = Eigen::Vector3d::UnitX();
        params.contactSurfaceMotionVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
      });
}

std::vector<Scenario> makeScenarios()
{
  const std::string box = boxModel("box", 0.0);
  std::vector<Scenario> scenarios;
#if GZ_PHYSICS_MAJOR_VERSION >= 8
  scenarios.push_back(
      {"joint_spring_reference",
       "SetJointSpringReferenceFeature",
       flapModel(0.0, dynamics(0.5, 0.0, 0.0, 8.0)),
       {},
       [](Run& run) { run.hinge()->SetSpringReference(0, -1.0); },
       {}});
  scenarios.push_back(
      {"joint_spring_stiffness",
       "SetJointSpringStiffnessFeature",
       flapModel(0.0, dynamics(0.5, 0.0, -1.0, 0.0)),
       {},
       [](Run& run) { run.hinge()->SetSpringStiffness(0, 8.0); },
       {}});
  scenarios.push_back(
      {"joint_damping",
       "SetJointDampingCoefficientFeature + kick",
       flapModel(0.0),
       {},
       [](Run& run) { run.hinge()->SetDampingCoefficient(0, 5.0); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       kickFlap});
  // Joint friction holds the flap up, so DART keeps it awake.
  scenarios.push_back(
      {"joint_friction",
       "SetJointFrictionFeature",
       flapModel(-0.6, dynamics(0.0, 20.0, 0.0, 0.0)),
       {},
       [](Run& run) { run.hinge()->SetFriction(0, 0.0); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/true});
  scenarios.push_back(
      {"joint_friction_added",
       "SetJointFrictionFeature + kick",
       flapModel(0.0),
       {},
       [](Run& run) { run.hinge()->SetFriction(0, 20.0); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       kickFlap});
#endif
  // The upper limit holds the flap up, so DART keeps it awake.
  scenarios.push_back(
      {"joint_position_limits",
       "SetJointPositionLimitsFeature",
       flapModel(-0.6, "<limit><lower>-1</lower><upper>0</upper></limit>"),
       {},
       [](Run& run) { run.hinge()->SetMaxPosition(0, 1.0); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/true});
  scenarios.push_back(
      {"joint_limits_tightened",
       "SetJointPositionLimitsFeature (tightened)",
       flapModel(0.0, "<limit><lower>-1</lower><upper>1</upper></limit>"),
       {},
       // The flap lies at position 0, so the new upper limit lifts it.
       [](Run& run) { run.hinge()->SetMaxPosition(0, -0.5); },
       {}});
  // A zero velocity command makes the joint a servo that holds the flap up,
  // so DART keeps it awake.
  scenarios.push_back(
      {"joint_effort_limits",
       "SetJointEffortLimitsFeature",
       flapModel(-0.6),
       [](Run& run) { run.hinge()->SetVelocityCommand(0, 0.0); },
       [](Run& run) {
         run.hinge()->SetMinEffort(0, 0.0);
         run.hinge()->SetMaxEffort(0, 0.0);
       },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/true});
  scenarios.push_back(
      {"joint_velocity_limits",
       "SetJointVelocityLimitsFeature + kick",
       flapModel(0.0),
       {},
       [](Run& run) {
         run.hinge()->SetMinVelocity(0, -0.2);
         run.hinge()->SetMaxVelocity(0, 0.2);
       },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       kickFlap});
  scenarios.push_back(
      {"joint_velocity_command",
       "SetJointVelocityCommandFeature",
       flapModel(0.0),
       {},
       {},
       [](Run& run) {
         run.hinge()->SetVelocityCommand(0, -1.0);
       }});
  scenarios.push_back(
      {"joint_force",
       "SetBasicJointState::SetForce",
       flapModel(0.0),
       {},
       {},
       [](Run& run) {
         run.hinge()->SetForce(0, -10.0);
       }});
  scenarios.push_back(
      {"joint_position",
       "SetBasicJointState::SetPosition",
       flapModel(0.0),
       {},
       [](Run& run) { run.hinge()->SetPosition(0, -0.5); },
       {}});
  scenarios.push_back(
      {"joint_velocity",
       "SetBasicJointState::SetVelocity",
       flapModel(0.0),
       {},
       [](Run& run) { run.hinge()->SetVelocity(0, -3.0); },
       {}});
  scenarios.push_back(
      {"link_gravity",
       "GravityEnabled",
       // The spring cannot lift the flap against gravity, only without it.
       flapModel(0.0, dynamics(0.5, 0.0, -1.0, 4.5)),
       {},
       [](Run& run) { run.link("hinge", "flap")->SetGravityEnabled(false); },
       {}});
  scenarios.push_back(
      {"model_gravity",
       "GravityEnabled (model)",
       flapModel(0.0, dynamics(0.5, 0.0, -1.0, 4.5)),
       {},
       [](Run& run) { run.model("hinge")->SetGravityEnabled(false); },
       {}});
  scenarios.push_back(
      {"joint_transform_to_child",
       "SetJointTransformToChildFeature",
       flapModel(0.0),
       {},
       [](Run& run) {
         // Lifts the flap 0.2 m above its hinge; it swings back down.
         run.hinge()->SetTransformToChild(
             Eigen::Isometry3d(Eigen::Translation3d(0.25, 0.0, 0.2)));
       },
       {}});
  scenarios.push_back(
      {"world_gravity",
       "Gravity",
       box,
       {},
       [](Run& run) {
         run.world->SetGravity(Eigen::Vector3d(0.0, 15.0, -9.8));
       },
       {}});
  scenarios.push_back(
      {"world_pose",
       "SetFreeGroupWorldPose",
       box,
       {},
       [](Run& run) {
         Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
         pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.55);
         run.model("box")->FindFreeGroup()->SetWorldPose(pose);
       },
       {}});
  scenarios.push_back(
      {"world_velocity",
       "SetFreeGroupWorldVelocity",
       box,
       {},
       [](Run& run) {
         run.model("box")->FindFreeGroup()->SetWorldLinearVelocity(
             Eigen::Vector3d(1.0, 0.0, 0.0));
       },
       {}});
  scenarios.push_back(
      {"link_wrench", "AddLinkExternalForceTorque", box, {}, {}, [](Run& run) {
         run.link("box", "link")
             ->AddExternalForce(Eigen::Vector3d(20.0, 0.0, 0.0));
       }});
  scenarios.push_back(
      {"model_static",
       "ModelStaticState",
       box + boxModel("floater", 2.0, 1.0, true),
       {},
       [](Run& run) { run.model("floater")->SetStatic(false); },
       {}});
  scenarios.push_back(
      {"model_collision",
       "ModelCollisionEnabled",
       box,
       {},
       [](Run& run) { run.model("box")->SetCollisionEnabled(false); },
       {}});
  scenarios.push_back(
      {"collision_filter_mask",
       "CollisionFilterMaskFeature",
       box,
       {},
       [](Run& run) {
         run.link("ground", "link")->GetShape(0)->SetCollisionFilterMask(0x1);
         run.link("box", "link")->GetShape(0)->SetCollisionFilterMask(0x2);
       },
       {},
       true});
  scenarios.push_back(
      {"category_mask",
       "CategoryFilterMaskFeature",
       box,
       {},
       [](Run& run) {
         // With no category bit shared, the box falls through the ground.
         run.link("ground", "link")->GetShape(0)->SetCategoryFilterMask(0x0);
         run.link("box", "link")->GetShape(0)->SetCategoryFilterMask(0x0);
       },
       {},
       true});
  scenarios.push_back(
      {"shape_pose",
       "SetShapeKinematicProperties",
       box,
       {},
       [](Run& run) {
         // The collision box moves 0.1 m up its link, which then drops 0.1 m.
         run.link("box", "link")->GetShape(0)->SetRelativeTransform(
             Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1)));
       },
       {}});
  scenarios.push_back(
      {"attach_shape",
       "AttachBoxShapeFeature + kick",
       box,
       {},
       [](Run& run) {
         // An outrigger on the +x side, touching the ground. (A shape attached
         // into the ground is inert here: DART resolves penetration at
         // 1 mm/s.)
         run.link("box", "link")
             ->AttachBoxShape(
                 "outrigger",
                 Eigen::Vector3d(0.5, 0.5, 0.1),
                 Eigen::Isometry3d(Eigen::Translation3d(0.5, 0.0, -0.2)));
       },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       // A torque that tips the box onto the outrigger.
       [](Run& run) {
         run.link("box", "link")
             ->AddExternalTorque(Eigen::Vector3d(0.0, 200.0, 0.0));
       }});
  scenarios.push_back(
      {"collision_pair_max_contacts",
       "CollisionPairMaxContacts",
       box,
       {},
       [](Run& run) { run.world->SetCollisionPairMaxContacts(1); },
       {}});
  scenarios.push_back(
      {"collision_detector",
       "CollisionDetector + kick",
       // A ball rests on Bullet and is pushed on ODE. ODE and Bullet roll it
       // differently (for a box they report the same contacts), so a stale
       // detector state shows. Rolling on ODE rather than Bullet keeps the
       // run insensitive to tiny differences between the runs.
       "<model name=\"ball\"><pose>0 0 0.25 0 0 0</pose><link name=\"link\">"
       "<inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy>"
       "<izz>0.025</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
       "</inertial><collision name=\"collision\"><geometry><sphere><radius>"
       "0.25</radius></sphere></geometry></collision></link></model>",
       [](Run& run) { run.world->SetCollisionDetector("bullet"); },
       [](Run& run) { run.world->SetCollisionDetector("ode"); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       [](Run& run) {
         run.link("ball", "link")
             ->AddExternalForce(Eigen::Vector3d(2000.0, 0.0, 0.0));
       }});
  scenarios.push_back(
      {"contact_callback_add",
       "SetContactPropertiesCallbackFeature (add)",
       box,
       {},
       addConveyor,
       {}});
  // The conveyor keeps the box moving, so DART keeps it awake.
  scenarios.push_back(
      {"contact_callback_remove",
       "SetContactPropertiesCallbackFeature (remove)",
       box,
       addConveyor,
       [](Run& run) { run.world->RemoveContactPropertiesCallback("conveyor"); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/true});
  scenarios.push_back(
      {"attach_fixed_joint",
       "AttachFixedJointFeature + kick",
       box + boxModel("other", 1.0),
       {},
       [](Run& run) { run.weldBoxToOther(Eigen::Vector3d(-1.0, 0.0, 0.0)); },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       pushOther});
  scenarios.push_back(
      {"attach_revolute_joint",
       "AttachRevoluteJointFeature + kick",
       box + boxModel("other", 1.0),
       {},
       [](Run& run) {
         auto hinge = run.link("box", "link")->AttachRevoluteJoint(
             run.link("other", "link"), "hinge", Eigen::Vector3d::UnitY());
         hinge->SetTransformFromParent(
             Eigen::Isometry3d(Eigen::Translation3d(-1.0, 0.0, 0.0)));
       },
       {},
       /*needsGazeboFilter=*/false,
       /*awakeByDesign=*/false,
       pushOther});
  scenarios.push_back(
      {"detach_joint",
       "DetachJointFeature",
       // The box is welded 0.25 m above the other one, then dropped. Its model
       // has a second link, resting 3 m away: welding moves the box's link
       // into the other model's skeleton, and the empty skeleton a one-link
       // model would leave behind (as with gz-sim's DetachableJoint) counts as
       // an awake body, which keeps every island in DART awake.
       boxModel("other", 0.0)
           + "<model name=\"box\"><pose>0 0 1 0 0 0</pose><link name=\"link\">"
           + inertial(1.0, 0.5, 0.5, 0.5) + boxCollision(0.5, 0.5, 0.5)
           + "</link><link name=\"anchor\"><pose>3 0 -0.75 0 0 0</pose>"
           + inertial(1.0, 0.5, 0.5, 0.5) + boxCollision(0.5, 0.5, 0.5)
           + "</link></model>",
       [](Run& run) { run.weldBoxToOther(Eigen::Vector3d(0.0, 0.0, 0.75)); },
       [](Run& run) { (*run.attached)->Detach(); },
       {}});
  scenarios.push_back(
      {"remove_support",
       "RemoveEntities",
       // The box rests on a static box, which is removed. (On a free box, the
       // pair does not fall asleep in every build that sleeps.)
       boxModel("support", 0.0, 0.25, true) + boxModel("box", 0.0, 0.75),
       {},
       [](Run& run) { run.model("support")->Remove(); },
       {},
       true});
  scenarios.push_back(
      {"spawn_onto_sleeper",
       "ConstructSdfModel",
       // A heavy box lands on the edge of the resting one and tips over it.
       box,
       {},
       [](Run& run) {
         run.spawn(boxModel("dropped", 0.3, 0.9, false, 20.0));
       },
       {},
       true});
  return scenarios;
}

//==============================================================================
// Runs one scenario on a fresh engine, so entity ids match between runs.
Outcome simulate(
    gz::plugin::Loader& loader,
    const std::string& pluginName,
    const Scenario& scenario,
    bool deactivation,
    bool defaultFilter,
    long settle,
    long steps)
{
  const auto engine = physics::RequestEngine3d<Features>::From(
      loader.Instantiate(pluginName));
  if (!engine) {
    std::fprintf(stderr, "the dartsim plugin lacks a requested feature\n");
    std::exit(1);
  }

  sdf::Root root;
  const auto errors = root.LoadSdfString(worldSdf(scenario.models));
  if (!errors.empty() || !root.WorldByIndex(0)) {
    for (const auto& error : errors)
      std::fprintf(stderr, "sdf: %s\n", error.Message().c_str());
    std::exit(1);
  }

  Run run;
  run.world = engine->ConstructWorld(*root.WorldByIndex(0));
  run.dartWorld = run.world->GetDartsimWorld();
  if (defaultFilter) {
    run.dartWorld->getConstraintSolver()->getCollisionOption().collisionFilter
        = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
  }
  if (!deactivation) {
    auto options = run.dartWorld->getDeactivationOptions();
    options.mEnabled = false;
    run.dartWorld->setDeactivationOptions(options);
  }
  run.world->SetCollisionPairMaxContacts(kGzSimCollisionPairMaxContacts);
  run.input.Get<std::chrono::steady_clock::duration>()
      = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(kStepSize));
  if (scenario.setup)
    scenario.setup(run);

  Outcome outcome;
  for (long i = 0; i < settle; ++i) {
    run.step();
    outcome.record(run);
  }

  for (std::size_t i = 0; i < run.dartWorld->getNumSkeletons(); ++i) {
    const auto skeleton = run.dartWorld->getSkeleton(i);
    if (!skeleton->isMobile())
      continue;
    ++outcome.mobile;
    if (skeleton->isResting())
      ++outcome.resting;
  }

  if (scenario.mutate)
    scenario.mutate(run);
  if (scenario.kick)
    scenario.kick(run);
  for (long i = 0; i < steps; ++i) {
    if (scenario.everyStep)
      scenario.everyStep(run);
    run.step();
    outcome.record(run);
  }

  for (const auto& contact : run.world->GetContactsFromLastStep()) {
    const auto& point = contact.template Get<ContactPoint>().point;
    outcome.finalContacts.push_back({point.x(), point.y(), point.z()});
  }
  return outcome;
}

// |a - b|, or infinity when either is not finite: std::max and std::min skip a
// NaN, so a run that blew up would otherwise match.
double absDifference(double a, double b)
{
  const double difference = std::abs(a - b);
  return std::isfinite(difference) ? difference
                                   : std::numeric_limits<double>::infinity();
}

double maxDifference(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() != b.size())
    return std::numeric_limits<double>::infinity();
  double difference = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
    difference = std::max(difference, absDifference(a[i], b[i]));
  return difference;
}

// Largest pose difference between two runs over steps [begin, end).
double trajectoryDifference(
    const Outcome& a, const Outcome& b, std::size_t begin, std::size_t end)
{
  double difference = 0.0;
  for (std::size_t i = begin; i < end; ++i)
    difference = std::max(difference, maxDifference(a.poses[i], b.poses[i]));
  return difference;
}

// Largest distance of a run's poses over steps [begin, end) from step
// `begin - 1`, the state the mutation was applied to. Links a mutation adds
// (spawned models) have no earlier pose and are left out.
double displacement(const Outcome& run, std::size_t begin, std::size_t end)
{
  const std::vector<double>& before = run.poses[begin - 1];
  double difference = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    const std::vector<double> after(
        run.poses[i].begin(),
        run.poses[i].begin()
            + static_cast<std::ptrdiff_t>(
                std::min(before.size(), run.poses[i].size())));
    difference = std::max(difference, maxDifference(after, before));
  }
  return difference;
}

// Largest distance from a contact point in `a` to the nearest one in `b`.
double contactPointDistance(
    const std::vector<std::array<double, 3>>& a,
    const std::vector<std::array<double, 3>>& b)
{
  double distance = 0.0;
  for (const auto& p : a) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& q : b) {
      nearest = std::min(
          nearest,
          std::max(
              {absDifference(p[0], q[0]),
               absDifference(p[1], q[1]),
               absDifference(p[2], q[2])}));
    }
    distance = std::max(distance, nearest);
  }
  return distance;
}

// Compares the contact counts on the first step after the mutation and at
// the end, and the final contact points. Counts in between are not compared:
// a contact may flicker on a pose difference far below the tolerance.
double contactDifference(
    const Outcome& a, const Outcome& b, std::size_t mutated)
{
  if (a.contactCounts[mutated] != b.contactCounts[mutated]
      || a.finalContacts.size() != b.finalContacts.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(
      contactPointDistance(a.finalContacts, b.finalContacts),
      contactPointDistance(b.finalContacts, a.finalContacts));
}

int usage(const char* program)
{
  std::fprintf(
      stderr,
      "usage: %s <dartsim-plugin.so> [--scenario NAME] [--settle N] "
      "[--steps N] [--tolerance X] [--default-filter] [--allow-unexercised] "
      "[--list]\n",
      program);
  return 2;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 2)
    return usage(argv[0]);

  const std::string pluginLib = argv[1];
  std::string only;
  long settle = 3000;
  long steps = 500;
  double tolerance = 1e-3;
  bool list = false;
  bool defaultFilter = false;
  bool allowUnexercised = false;
  for (int i = 2; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--list") {
      list = true;
      continue;
    }
    if (key == "--default-filter") {
      defaultFilter = true;
      continue;
    }
    if (key == "--allow-unexercised") {
      allowUnexercised = true;
      continue;
    }
    if (i + 1 >= argc)
      return usage(argv[0]);
    const char* value = argv[++i];
    if (key == "--scenario")
      only = value;
    else if (key == "--settle")
      settle = std::atol(value);
    else if (key == "--steps")
      steps = std::atol(value);
    else if (key == "--tolerance")
      tolerance = std::atof(value);
    else
      return usage(argv[0]);
  }

  const auto scenarios = makeScenarios();
  if (list) {
    for (const auto& scenario : scenarios)
      std::printf(
          "%-28s %s\n", scenario.name.c_str(), scenario.feature.c_str());
    return 0;
  }

  if (settle < 1 || steps < 1)
    return usage(argv[0]);

  gz::plugin::Loader loader;
  std::string pluginName;
  for (const auto& name : loader.LoadLib(pluginLib)) {
    if (name.find("dartsim") != std::string::npos)
      pluginName = name;
  }
  if (pluginName.empty()) {
    std::fprintf(stderr, "no dartsim plugin in %s\n", pluginLib.c_str());
    return 1;
  }

  std::printf(
      "settle=%ld steps=%ld tolerance=%g filter=%s plugin=%s\n",
      settle,
      steps,
      tolerance,
      defaultFilter ? "dart-default" : "gz-physics",
      pluginLib.c_str());
  std::printf(
      "%-28s %-44s %-8s %-9s %-9s %-9s %-15s %s\n",
      "scenario",
      "feature",
      "resting",
      "moved",
      "pre_diff",
      "post_diff",
      "contacts",
      "verdict");

  std::size_t ran = 0;
  std::size_t skipped = 0;
  std::size_t mismatches = 0;
  std::size_t unexercised = 0;
  for (const auto& scenario : scenarios) {
    if (!only.empty() && scenario.name != only)
      continue;
    if (defaultFilter && scenario.needsGazeboFilter) {
      ++skipped;
      std::printf(
          "%-28s %-44s skipped (needs gz-physics' filter)\n",
          scenario.name.c_str(),
          scenario.feature.c_str());
      continue;
    }
    ++ran;

    const Outcome sleeping = simulate(
        loader, pluginName, scenario, true, defaultFilter, settle, steps);
    const Outcome reference = simulate(
        loader, pluginName, scenario, false, defaultFilter, settle, steps);
    const auto mutated = static_cast<std::size_t>(settle);
    const auto end = reference.poses.size();
    double moved = displacement(reference, mutated, end);
    if (scenario.kick) {
      // How far the mutation moved the kicked run.
      Scenario kickOnly = scenario;
      kickOnly.mutate = nullptr;
      const Outcome kicked = simulate(
          loader, pluginName, kickOnly, false, defaultFilter, settle, steps);
      moved = trajectoryDifference(reference, kicked, mutated, end);
    }
    const double pre = trajectoryDifference(sleeping, reference, 0, mutated);
    const double post = trajectoryDifference(sleeping, reference, mutated, end);
    const double contacts = contactDifference(sleeping, reference, mutated);

    const bool posesDiffer = pre > tolerance || post > tolerance;
    const bool contactsDiffer = contacts > tolerance;
    const bool asleep = sleeping.resting > 0;
    const bool inert = moved <= tolerance;
    std::vector<std::string> notes;
    if (posesDiffer)
      notes.emplace_back("poses");
    if (contactsDiffer)
      notes.emplace_back("contacts");
    if (!asleep)
      notes.emplace_back(
          scenario.awakeByDesign ? "awake by design" : "nothing asleep");
    if (inert)
      notes.emplace_back("inert");
    std::string verdict = "match";
    if (posesDiffer || contactsDiffer) {
      verdict = "MISMATCH";
      ++mismatches;
    } else if (inert || (!asleep && !scenario.awakeByDesign)) {
      verdict = "UNEXERCISED";
      ++unexercised;
    }
    for (std::size_t i = 0; i < notes.size(); ++i)
      verdict += (i == 0 ? " (" : ", ") + notes[i];
    verdict += notes.empty() ? "" : ")";

    const std::string resting = std::to_string(sleeping.resting) + "/"
                                + std::to_string(sleeping.mobile);
    // Contact counts with sleeping and without, on the first post-mutation step
    // and at the end.
    const std::string contactCounts
        = std::to_string(sleeping.contactCounts[mutated]) + "/"
          + std::to_string(reference.contactCounts[mutated]) + ","
          + std::to_string(sleeping.finalContacts.size()) + "/"
          + std::to_string(reference.finalContacts.size());
    std::printf(
        "%-28s %-44s %-8s %-9.2e %-9.2e %-9.2e %-15s %s\n",
        scenario.name.c_str(),
        scenario.feature.c_str(),
        resting.c_str(),
        moved,
        pre,
        post,
        contactCounts.c_str(),
        verdict.c_str());
    std::fflush(stdout);
  }

  if (ran + skipped == 0) {
    std::fprintf(stderr, "unknown scenario: %s\n", only.c_str());
    return 2;
  }
  std::printf(
      "SUMMARY scenarios=%zu skipped=%zu mismatches=%zu unexercised=%zu%s\n",
      ran,
      skipped,
      mismatches,
      unexercised,
      unexercised > 0 && allowUnexercised ? " (allowed)" : "");
  return mismatches > 0 || (unexercised > 0 && !allowUnexercised) ? 1 : 0;
}
