/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "TinkertoyWidget.hpp"

#include "dart/external/imgui/imgui.h"

#include "TinkertoyWorldNode.hpp"

//==============================================================================
TinkertoyWidget::TinkertoyWidget(
    dart::gui::osg::ImGuiViewer* viewer,
    TinkertoyWorldNode* node)
  : mViewer(viewer),
    mNode(node),
    mGuiGravity(true),
    mGravity(true),
    mGuiHeadlights(true)
{
  // Do nothing
}

//==============================================================================
void TinkertoyWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(10,20));
  if (!ImGui::Begin("Tinkertoy Control", nullptr, ImVec2(360,640), 0.5f,
                    ImGuiWindowFlags_NoResize |
                    ImGuiWindowFlags_MenuBar |
                    ImGuiWindowFlags_HorizontalScrollbar))
  {
    // Early out if the window is collapsed, as an optimization.
    ImGui::End();
    return;
  }

  // Menu
  if (ImGui::BeginMenuBar())
  {
    if (ImGui::BeginMenu("Menu"))
    {
      if (ImGui::MenuItem("Exit"))
        mViewer->setDone(true);
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help"))
    {
      if (ImGui::MenuItem("About DART"))
        mViewer->showAbout();
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }

  ImGui::Text("<TODO: Short description about this example>");
  ImGui::Spacing();

  if (ImGui::CollapsingHeader("Help"))
  {
    ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
    ImGui::Text("User Guid:\n");
    ImGui::Text("%s", mViewer->getInstructions().c_str());
    ImGui::Text("Left-click on a block to select it.\n");
    ImGui::Text("Press [Backspace] to deselect.\n");
    ImGui::Text("Press [Tab] to reset the camera view\n");
    ImGui::Text("Press [`] to reset the orientation of the target\n");
    ImGui::Text("\n --- While Simulation is Paused ---\n");
    ImGui::Text("The selected block will be red; all other blocks will be yellow.\n");
    ImGui::Text("Press [1] -> [3] to attach a new block to the selected block.\n");
    ImGui::Text("[1]: Attach using a WeldJoint\n");
    ImGui::Text("[2]: Attach using a RevoluteJoint\n");
    ImGui::Text("[3]: Attach using a BallJoint\n");
    ImGui::Text("The longitudinal direction of the new block will be along the x-axis (Red) of the target.\n");
    ImGui::Text("The joint axis will follow the z-axis (Blue) of the target when making a RevolueJoint.\n");
    ImGui::Text("Press [Delete] to permanently remove a block and all of its children.\n");
    ImGui::Text("Adding a block when nothing is currently selected will attach it to the world, beginning a new tree.\n");
    ImGui::Text("\n --- While Simulation is Active ---\n");
    ImGui::Text("The selected block will be Fuchsia; all other blocks will be blue.\n");
    ImGui::Text("Move around the target to pull on the selected block.\n");
    ImGui::Text("Press [Up] or [Down] to adjust the pulling strength.\n");
    ImGui::Text("Press [G] to toggle Gravity\n");
    ImGui::Text("Blocks belonging to different trees can collide with each other during simulation.\n");
    ImGui::Text("Collisions between blocks belonging to the same tree will be ignored.\n");
    ImGui::PopTextWrapPos();
  }

  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
  {
    int e = mViewer->isSimulating() ? 0 : 1;
    if(mViewer->isAllowingSimulation())
    {
      if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
        mViewer->simulate(true);
      ImGui::SameLine();
      if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
        mViewer->simulate(false);
    }
  }

  if (ImGui::CollapsingHeader("World Options", ImGuiTreeNodeFlags_DefaultOpen))
  {
    // Gravity
    ImGui::Checkbox("Gravity On/Off", &mGuiGravity);
    setGravity(mGuiGravity);

    ImGui::Spacing();

    // Headlights
    mGuiHeadlights = mViewer->checkHeadlights();
    ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
    mViewer->switchHeadlights(mGuiHeadlights);
  }

  if (ImGui::CollapsingHeader("Tinkertoy Options", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::SliderFloat(
          "Force Coeff", &mNode->mForceCoeff,
          MinForceCoeff, MaxForceCoeff, "%.1f");

    ImGui::Spacing();

    const auto reorient = ImGui::Button("Reorient Target");
    if (reorient)
      mNode->reorientTarget();

    const auto clearTarget = ImGui::Button("Reset Target");
    if (clearTarget)
      mNode->clearPick();

    const auto addWeld = ImGui::Button("Add a Weld-Joint Block");
    if (addWeld)
      mNode->addWeldJointBlock();

    const auto addRevolute = ImGui::Button("Add a Revolute-Joint Block");
    if (addRevolute)
      mNode->addRevoluteJointBlock();

    const auto addBall = ImGui::Button("Add a Ball-Joint Block");
    if (addBall)
      mNode->addBallJointBlock();

    const auto deleteBlock = ImGui::Button("Delete Block");
    if (deleteBlock)
      mNode->deletePick();
  }

  ImGui::End();
}

//==============================================================================
void TinkertoyWidget::setGravity(bool gravity)
{
  if (mGravity == gravity)
    return;

  mGravity = gravity;

  if (mGravity)
    mNode->getWorld()->setGravity(-9.81*Eigen::Vector3d::UnitZ());
  else
    mNode->getWorld()->setGravity(Eigen::Vector3d::Zero());
}
