/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "AtlasSimbiconWidget.hpp"

#include "dart/gui/imgui.h"
#include "AtlasSimbiconWorldNode.hpp"

//==============================================================================
AtlasSimbiconWidget::AtlasSimbiconWidget(
    dart::gui::osg::ImGuiViewer* viewer,
    AtlasSimbiconWorldNode* node)
  : mViewer(viewer),
    mNode(node),
    mGuiGravityAcc(9.81f),
    mGravityAcc(mGuiGravityAcc),
    mGuiHeadlights(true)
{
  // Do nothing
}

//==============================================================================
void AtlasSimbiconWidget::render()
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
    ImGui::Text("Press [r] to reset Atlas to the initial position.\n");
    ImGui::Text("Press [a] to push forward Atlas toroso.\n");
    ImGui::Text("Press [s] to push backward Atlas toroso.\n");
    ImGui::Text("Press [d] to push left Atlas toroso.\n");
    ImGui::Text("Press [f] to push right Atlas toroso.\n");
    ImGui::Text("Left-click on a block to select it.\n");
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
    ImGui::SliderFloat("Gravity Acc.", &mGuiGravityAcc, 5.0, 20.0, "-%.2f");
    setGravity(mGuiGravityAcc);

    ImGui::Spacing();

    // Headlights
    mGuiHeadlights = mViewer->checkHeadlights();
    ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
    mViewer->switchHeadlights(mGuiHeadlights);
  }

  if (ImGui::CollapsingHeader("Atlas Simbicon Options", ImGuiTreeNodeFlags_DefaultOpen))
  {
    const auto reset = ImGui::Button("Reset Atlas");
    if (reset)
      mNode->reset();

    ImGui::Spacing();

    // Stride
    int e = mIsShortStride ? 0 : 1;
    if (ImGui::RadioButton("Short", &e, 0) && !mViewer->isSimulating())
      mNode->changeToRunning();
    ImGui::SameLine();
    if (ImGui::RadioButton("Large", &e, 1) && mViewer->isSimulating())
      mNode->changeToWalking();
  }

  ImGui::End();
}

//==============================================================================
void AtlasSimbiconWidget::setGravity(float gravity)
{
  if (mGravityAcc == gravity)
    return;

  mGravityAcc = gravity;
  mNode->getWorld()->setGravity(-mGravityAcc*Eigen::Vector3d::UnitY());
}
