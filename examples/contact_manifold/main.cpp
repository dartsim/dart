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

#include <dart/gui/All.hpp>
#include <dart/gui/IncludeImGui.hpp>

#include <dart/utils/All.hpp>

#include <dart/All.hpp>

#include <CLI/CLI.hpp>

#include <iostream>
#include <unordered_map>

#include <cmath>

using namespace dart;

//==============================================================================
struct ContactKey
{
  Eigen::Vector3d position;
  double tolerance = 0.05;

  bool operator==(const ContactKey& other) const
  {
    return (position - other.position).norm() < tolerance;
  }
};

struct ContactKeyHash
{
  std::size_t operator()(const ContactKey& key) const
  {
    constexpr double gridSize = 0.05;
    int x = static_cast<int>(std::floor(key.position.x() / gridSize));
    int y = static_cast<int>(std::floor(key.position.y() / gridSize));
    int z = static_cast<int>(std::floor(key.position.z() / gridSize));
    return std::hash<int>()(x) ^ (std::hash<int>()(y) << 1)
           ^ (std::hash<int>()(z) << 2);
  }
};

//==============================================================================
struct TrackedContact
{
  Eigen::Vector3d position;
  Eigen::Vector3d normal;
  int age = 0;
  bool seenThisFrame = false;
};

//==============================================================================
dynamics::SkeletonPtr createBox(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero())
{
  dynamics::SkeletonPtr boxSkel = dynamics::Skeleton::create("box");

  dynamics::BodyNodePtr boxBody
      = boxSkel->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr)
            .second;

  double boxWidth = 1.0;
  double boxDepth = 1.0;
  double boxHeight = 0.5;
  auto boxShape = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(boxWidth, boxDepth, boxHeight));
  dynamics::ShapeNode* shapeNode = boxBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position + offset;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

//==============================================================================
std::vector<dynamics::SkeletonPtr> createBoxStack(
    std::size_t numBoxes,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
    double heightFromGround = 0.5)
{
  std::vector<dynamics::SkeletonPtr> boxSkels(numBoxes);

  for (auto i = 0u; i < numBoxes; ++i)
    boxSkels[i] = createBox(
        Eigen::Vector3d(0.0, 0.0, heightFromGround + 0.25 + i * 0.5), offset);

  return boxSkels;
}

//==============================================================================
dynamics::SkeletonPtr createFloor(
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero())
{
  dynamics::SkeletonPtr floor = dynamics::Skeleton::create("floor");

  dynamics::BodyNodePtr body
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr).second;

  double floorWidth = 10.0;
  double floorHeight = 0.01;
  auto box = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(floorWidth, floorWidth, floorHeight));
  dynamics::ShapeNode* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = offset + Eigen::Vector3d(0.0, 0.0, -floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
void populateWorld(
    const simulation::WorldPtr& world,
    const Eigen::Vector3d& offset,
    std::size_t numBoxes)
{
  world->removeAllSkeletons();
  world->addSkeleton(createFloor(offset));

  auto boxSkels = createBoxStack(numBoxes, offset);
  for (const auto& boxSkel : boxSkels)
    world->addSkeleton(boxSkel);

  world->reset();
}

//==============================================================================
Eigen::Vector4d getColorForAge(int age)
{
  if (age <= 0) {
    return Eigen::Vector4d(1.0, 0.0, 0.0, 0.8);
  } else if (age < 5) {
    double t = age / 5.0;
    return Eigen::Vector4d(1.0, t, 0.0, 0.8);
  } else if (age < 10) {
    double t = (age - 5) / 5.0;
    return Eigen::Vector4d(1.0 - t, 1.0, 0.0, 0.8);
  } else {
    return Eigen::Vector4d(0.0, 1.0, 0.0, 0.8);
  }
}

//==============================================================================
class ContactVisualizer
{
public:
  ContactVisualizer(
      const simulation::WorldPtr& world, const Eigen::Vector3d& offset)
    : mWorld(world), mOffset(offset)
  {
  }

  void update()
  {
    if (!mWorld)
      return;

    auto* solver = mWorld->getConstraintSolver();
    if (!solver)
      return;

    for (auto& [key, contact] : mTrackedContacts)
      contact.seenThisFrame = false;

    const auto& result = solver->getLastCollisionResult();
    for (const auto& contact : result.getContacts()) {
      ContactKey key{contact.point, 0.05};

      auto it = mTrackedContacts.find(key);
      if (it != mTrackedContacts.end()) {
        it->second.age++;
        it->second.seenThisFrame = true;
        it->second.position = contact.point;
        it->second.normal = contact.normal;
      } else {
        TrackedContact tc;
        tc.position = contact.point;
        tc.normal = contact.normal;
        tc.age = 0;
        tc.seenThisFrame = true;
        mTrackedContacts[key] = tc;
      }
    }

    for (auto it = mTrackedContacts.begin(); it != mTrackedContacts.end();) {
      if (!it->second.seenThisFrame) {
        it = mTrackedContacts.erase(it);
      } else {
        ++it;
      }
    }

    updateVisuals();
  }

  void clearVisuals()
  {
    for (auto& frame : mContactFrames) {
      mWorld->removeSimpleFrame(frame);
    }
    mContactFrames.clear();
    for (auto& frame : mNormalFrames) {
      mWorld->removeSimpleFrame(frame);
    }
    mNormalFrames.clear();
  }

  void setShowContacts(bool show)
  {
    mShowContacts = show;
  }
  void setShowNormals(bool show)
  {
    mShowNormals = show;
  }
  bool getShowContacts() const
  {
    return mShowContacts;
  }
  bool getShowNormals() const
  {
    return mShowNormals;
  }

  std::size_t getNumTrackedContacts() const
  {
    return mTrackedContacts.size();
  }

  std::size_t getNumStableContacts() const
  {
    std::size_t count = 0;
    for (const auto& [key, contact] : mTrackedContacts) {
      if (contact.age >= 10)
        count++;
    }
    return count;
  }

  int getMaxAge() const
  {
    int maxAge = 0;
    for (const auto& [key, contact] : mTrackedContacts) {
      maxAge = std::max(maxAge, contact.age);
    }
    return maxAge;
  }

  void reset()
  {
    mTrackedContacts.clear();
  }

private:
  void updateVisuals()
  {
    clearVisuals();

    if (!mShowContacts && !mShowNormals)
      return;

    std::size_t idx = 0;
    for (const auto& [key, contact] : mTrackedContacts) {
      if (mShowContacts) {
        auto frame = std::make_shared<dynamics::SimpleFrame>(
            dynamics::Frame::World(), "contact_" + std::to_string(idx));

        auto sphere
            = std::make_shared<dynamics::SphereShape>(mContactSphereRadius);
        frame->setShape(sphere);

        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.translation() = contact.position;
        frame->setRelativeTransform(tf);

        Eigen::Vector4d color = getColorForAge(contact.age);
        frame->createVisualAspect();
        frame->getVisualAspect()->setColor(Eigen::Vector3d(color.head<3>()));
        frame->getVisualAspect()->setAlpha(color.w());

        mWorld->addSimpleFrame(frame);
        mContactFrames.push_back(frame);
      }

      if (mShowNormals) {
        auto frame = std::make_shared<dynamics::SimpleFrame>(
            dynamics::Frame::World(), "normal_" + std::to_string(idx));

        Eigen::Vector3d start = contact.position;
        Eigen::Vector3d end = contact.position + contact.normal * mNormalLength;

        auto arrow = std::make_shared<dynamics::ArrowShape>(
            start,
            end,
            dynamics::ArrowShape::Properties(0.005, 1.5),
            Eigen::Vector4d(0.2, 0.2, 1.0, 0.8));
        frame->setShape(arrow);

        mWorld->addSimpleFrame(frame);
        mNormalFrames.push_back(frame);
      }

      idx++;
    }
  }

  simulation::WorldPtr mWorld;
  Eigen::Vector3d mOffset;
  std::unordered_map<ContactKey, TrackedContact, ContactKeyHash>
      mTrackedContacts;
  std::vector<dynamics::SimpleFramePtr> mContactFrames;
  std::vector<dynamics::SimpleFramePtr> mNormalFrames;
  bool mShowContacts = true;
  bool mShowNormals = false;
  double mContactSphereRadius = 0.03;
  double mNormalLength = 0.15;
};

//==============================================================================
class ContactManifoldWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  ContactManifoldWorldNode(
      const dart::simulation::WorldPtr& world,
      std::shared_ptr<ContactVisualizer> visualizer)
    : dart::gui::RealTimeWorldNode(world), mVisualizer(std::move(visualizer))
  {
  }

  void customPostStep() override
  {
    if (mVisualizer) {
      mVisualizer->update();
    }
  }

private:
  std::shared_ptr<ContactVisualizer> mVisualizer;
};

//==============================================================================
class ContactManifoldWidget : public dart::gui::ImGuiWidget
{
public:
  ContactManifoldWidget(
      dart::gui::ImGuiViewer* viewer,
      dart::simulation::WorldPtr worldLeft,
      dart::simulation::WorldPtr worldRight,
      std::shared_ptr<ContactVisualizer> vizLeft,
      std::shared_ptr<ContactVisualizer> vizRight,
      const Eigen::Vector3d& leftOffset,
      const Eigen::Vector3d& rightOffset,
      std::size_t numBoxes)
    : mViewer(viewer),
      mWorldLeft(std::move(worldLeft)),
      mWorldRight(std::move(worldRight)),
      mVizLeft(std::move(vizLeft)),
      mVizRight(std::move(vizRight)),
      mLeftOffset(leftOffset),
      mRightOffset(rightOffset),
      mNumBoxes(numBoxes),
      mGuiNumBoxes(static_cast<int>(numBoxes)),
      mManifoldEnabledLeft(false),
      mManifoldEnabledRight(true)
  {
    applyContactManifoldSetting(mWorldLeft, mManifoldEnabledLeft);
    applyContactManifoldSetting(mWorldRight, mManifoldEnabledRight);
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(320, 480));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "Contact Manifold Demo",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
                | ImGuiWindowFlags_HorizontalScrollbar)) {
      ImGui::End();
      return;
    }

    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    ImGui::TextWrapped(
        "This demo shows contact manifold caching. "
        "Left stack has manifold OFF (contacts flicker). "
        "Right stack has manifold ON (contacts persist).");
    ImGui::Spacing();

    ImGui::Separator();
    ImGui::Text(
        "%.3f ms/frame (%.1f FPS)",
        1000.0f / ImGui::GetIO().Framerate,
        ImGui::GetIO().Framerate);
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      int e = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation()) {
        if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
          mViewer->simulate(true);
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
          mViewer->simulate(false);
      }

      const auto time = mWorldLeft
                            ? mWorldLeft->getTime()
                            : (mWorldRight ? mWorldRight->getTime() : 0.0);
      ImGui::Text("Time: %.3f", time);
    }

    if (ImGui::CollapsingHeader(
            "Contact Manifold", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool manifoldLeft = mManifoldEnabledLeft;
      bool manifoldRight = mManifoldEnabledRight;

      ImGui::Checkbox("Left stack: Manifold", &manifoldLeft);
      ImGui::Checkbox("Right stack: Manifold", &manifoldRight);

      if (manifoldLeft != mManifoldEnabledLeft) {
        mManifoldEnabledLeft = manifoldLeft;
        applyContactManifoldSetting(mWorldLeft, mManifoldEnabledLeft);
      }

      if (manifoldRight != mManifoldEnabledRight) {
        mManifoldEnabledRight = manifoldRight;
        applyContactManifoldSetting(mWorldRight, mManifoldEnabledRight);
      }

      ImGui::SliderInt("Stack Height", &mGuiNumBoxes, 1, 20);

      if (ImGui::Button("Reset Stacks")) {
        mNumBoxes = static_cast<std::size_t>(mGuiNumBoxes);
        populateWorld(mWorldLeft, mLeftOffset, mNumBoxes);
        populateWorld(mWorldRight, mRightOffset, mNumBoxes);
        applyContactManifoldSetting(mWorldLeft, mManifoldEnabledLeft);
        applyContactManifoldSetting(mWorldRight, mManifoldEnabledRight);
        if (mVizLeft)
          mVizLeft->reset();
        if (mVizRight)
          mVizRight->reset();
      }

      ImGui::Spacing();
      ImGui::Text(
          "Left Stack (Manifold %s):", mManifoldEnabledLeft ? "ON" : "OFF");
      ImGui::Text(
          "  Manifolds: %zu, Persistent: %zu",
          getManifoldCount(mWorldLeft),
          getPersistentCount(mWorldLeft));

      ImGui::Text(
          "Right Stack (Manifold %s):", mManifoldEnabledRight ? "ON" : "OFF");
      ImGui::Text(
          "  Manifolds: %zu, Persistent: %zu",
          getManifoldCount(mWorldRight),
          getPersistentCount(mWorldRight));
    }

    if (ImGui::CollapsingHeader(
            "Contact Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::TextWrapped(
          "Contact colors by age:\n"
          "  Red = new (age 0)\n"
          "  Yellow = warming up (age ~5)\n"
          "  Green = stable (age >= 10)");
      ImGui::Spacing();

      bool showContactsLeft = mVizLeft ? mVizLeft->getShowContacts() : false;
      bool showContactsRight = mVizRight ? mVizRight->getShowContacts() : false;
      bool showNormalsLeft = mVizLeft ? mVizLeft->getShowNormals() : false;
      bool showNormalsRight = mVizRight ? mVizRight->getShowNormals() : false;

      if (ImGui::Checkbox("Show Left Contacts", &showContactsLeft) && mVizLeft)
        mVizLeft->setShowContacts(showContactsLeft);
      if (ImGui::Checkbox("Show Right Contacts", &showContactsRight)
          && mVizRight)
        mVizRight->setShowContacts(showContactsRight);
      if (ImGui::Checkbox("Show Left Normals", &showNormalsLeft) && mVizLeft)
        mVizLeft->setShowNormals(showNormalsLeft);
      if (ImGui::Checkbox("Show Right Normals", &showNormalsRight) && mVizRight)
        mVizRight->setShowNormals(showNormalsRight);

      ImGui::Spacing();
      if (mVizLeft) {
        ImGui::Text(
            "Left: %zu contacts, %zu stable, max age %d",
            mVizLeft->getNumTrackedContacts(),
            mVizLeft->getNumStableContacts(),
            mVizLeft->getMaxAge());
      }
      if (mVizRight) {
        ImGui::Text(
            "Right: %zu contacts, %zu stable, max age %d",
            mVizRight->getNumTrackedContacts(),
            mVizRight->getNumStableContacts(),
            mVizRight->getMaxAge());
      }
    }

    if (ImGui::CollapsingHeader("View")) {
      osg::Vec3d eye;
      osg::Vec3d center;
      osg::Vec3d up;
      mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

      ImGui::Text("Eye   : (%.2f, %.2f, %.2f)", eye.x(), eye.y(), eye.z());
      ImGui::Text(
          "Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
      ImGui::Text("Up    : (%.2f, %.2f, %.2f)", up.x(), up.y(), up.z());
    }

    if (ImGui::CollapsingHeader("Help")) {
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 300);
      ImGui::Text("User Guide:\n");
      ImGui::Text("%s", mViewer->getInstructions().c_str());
      ImGui::PopTextWrapPos();
    }

    ImGui::End();
  }

protected:
  void applyContactManifoldSetting(
      const simulation::WorldPtr& world, bool enabled) const
  {
    if (!world)
      return;

    auto* solver = world->getConstraintSolver();
    if (!solver)
      return;

    solver->setContactManifoldCacheEnabled(enabled);
  }

  std::size_t getManifoldCount(const simulation::WorldPtr& world) const
  {
    auto* solver = world ? world->getConstraintSolver() : nullptr;
    return solver ? solver->getNumContactManifolds() : 0u;
  }

  std::size_t getPersistentCount(const simulation::WorldPtr& world) const
  {
    auto* solver = world ? world->getConstraintSolver() : nullptr;
    return solver ? solver->getNumPersistentContacts() : 0u;
  }

  ::osg::ref_ptr<dart::gui::ImGuiViewer> mViewer;
  dart::simulation::WorldPtr mWorldLeft;
  dart::simulation::WorldPtr mWorldRight;
  std::shared_ptr<ContactVisualizer> mVizLeft;
  std::shared_ptr<ContactVisualizer> mVizRight;
  Eigen::Vector3d mLeftOffset;
  Eigen::Vector3d mRightOffset;
  std::size_t mNumBoxes;
  int mGuiNumBoxes;
  bool mManifoldEnabledLeft;
  bool mManifoldEnabledRight;
};

//==============================================================================
int main(int argc, char* argv[])
{
  CLI::App app("Contact Manifold Demo - visualizes persistent contacts");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  const Eigen::Vector3d leftOffset(-3.0, 0.0, 0.0);
  const Eigen::Vector3d rightOffset(3.0, 0.0, 0.0);
  constexpr std::size_t numBoxes = 5u;

  simulation::WorldPtr worldLeft = simulation::World::create();
  simulation::WorldPtr worldRight = simulation::World::create();

  populateWorld(worldLeft, leftOffset, numBoxes);
  populateWorld(worldRight, rightOffset, numBoxes);

  if (auto* solver = worldLeft->getConstraintSolver())
    solver->setContactManifoldCacheEnabled(false);

  if (auto* solver = worldRight->getConstraintSolver())
    solver->setContactManifoldCacheEnabled(true);

  auto vizLeft = std::make_shared<ContactVisualizer>(worldLeft, leftOffset);
  auto vizRight = std::make_shared<ContactVisualizer>(worldRight, rightOffset);

  osg::ref_ptr<ContactManifoldWorldNode> nodeLeft
      = new ContactManifoldWorldNode(worldLeft, vizLeft);
  osg::ref_ptr<ContactManifoldWorldNode> nodeRight
      = new ContactManifoldWorldNode(worldRight, vizRight);

  osg::ref_ptr<dart::gui::ImGuiViewer> viewer = new dart::gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->addWorldNode(nodeLeft);
  viewer->addWorldNode(nodeRight);

  viewer->getImGuiHandler()->addWidget(std::make_shared<ContactManifoldWidget>(
      viewer,
      worldLeft,
      worldRight,
      vizLeft,
      vizRight,
      leftOffset,
      rightOffset,
      numBoxes));

  viewer->setUpViewInWindow(0, 0, 1024, 768);

  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(14.00f, 14.00f, 10.00f),
      ::osg::Vec3(0.00f, 0.00f, 2.00f),
      ::osg::Vec3(0.00f, 0.00f, 1.00f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  std::cout << "=== Contact Manifold Demo ===" << std::endl;
  std::cout << "Left stack: Manifold OFF (contacts may flicker)" << std::endl;
  std::cout << "Right stack: Manifold ON (contacts persist)" << std::endl;
  std::cout << std::endl;
  std::cout << "Contact colors:" << std::endl;
  std::cout << "  Red    = new contact (age 0)" << std::endl;
  std::cout << "  Yellow = warming up (age ~5)" << std::endl;
  std::cout << "  Green  = stable contact (age >= 10)" << std::endl;
  std::cout << std::endl;
  std::cout << "Use the ImGui panel to toggle settings." << std::endl;

  viewer->run();

  return 0;
}
