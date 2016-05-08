/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_GUI_OSG_VIEWER_HPP_
#define DART_GUI_OSG_VIEWER_HPP_

#include <map>
#include <unordered_set>
#include <memory>

#include <osgViewer/Viewer>

#include <Eigen/Core>

#include "dart/common/Subject.hpp"

namespace dart {

namespace simulation {
class World;
} // namespace simulation

namespace dynamics {
class Entity;
class SimpleFrame;
class Shape;
class BodyNode;
} // namespace dynamics

namespace gui {
namespace osg {

class WorldNode;
class DefaultEventHandler;
class DragAndDrop;
class SimpleFrameDnD;
class SimpleFrameShapeDnD;
class InteractiveFrame;
class InteractiveFrameDnD;
class BodyNodeDnD;
class Viewer;
class SaveScreen;

class ViewerAttachment : public virtual ::osg::Group
{
public:

  friend class Viewer;

  /// Default constructor
  ViewerAttachment();

  /// Virtual destructor
  virtual ~ViewerAttachment();

  /// This function will get called each time the Viewer is refreshed. Override
  /// it to have your attachment update with each refresh cycle.
  virtual void refresh() = 0;

  Viewer* getViewer();

  const Viewer* getViewer() const;

protected:

  /// This function will be called by attach(Viewer*) so you can do customized
  /// setup when the Viewer changes. By default this function does nothing, so
  /// overriding it is completely safe.
  virtual void customAttach(Viewer* newViewer);

  /// This function will get called when the visual is attached to a new Viewer.
  /// It adds this node as a child to the root node of the Viewer so the
  /// refresh() function will be called at each update cycle. It will also call
  /// customAttach(Viewer*) so you can do customized setup.
  virtual void attach(Viewer* newViewer);



private:

  Viewer* mViewer;

};

class Viewer : public osgViewer::Viewer, public dart::common::Subject
{
public:

  /// Constructor for dart::gui::osg::Viewer. This will automatically create the
  /// default event handler.
  Viewer(const ::osg::Vec4& clearColor = ::osg::Vec4(0.9,0.9,0.9,1.0));

  /// Destructor
  virtual ~Viewer();

  /// Capture the current screen in a png file. The filename argument should
  /// include the full path for the file.
  ///
  /// It is recommended that you end the filename with ".png". That is the type
  /// of file that will be produced, but the extension will not be added on
  /// automatically.
  // TODO(MXG): Add a bool argument that instructs the viewer to append a date
  // and time to the name of the file.
  void captureScreen(const std::string& filename);

  /// As the screen refreshes, save screen capture images to the specified
  /// directory.
  ///
  /// The prefix argument will be the first part of the name of each
  /// image. The second part of the image will contain an integer with "digits"
  /// number of digits; any unused leading digits will be filled with 0. The
  /// integer corresponds to its number in the sequence of frames.
  ///
  /// Set the restart argument to true if you want the sequence counter to
  /// restart from 0.
  void record(const std::string& directory, const std::string& prefix = "image",
              bool restart = false, std::size_t digits=6);

  /// If the Viewer is recording, then pause the recording. The next time
  /// record() is called, the numbering of the images will resume from where it
  /// left off.
  void pauseRecording();

  /// Returns true if the Viewer is currently recording.
  bool isRecording() const;

  /// Creates the default event handler for this dart::gui::osg::Viewer
  virtual void switchDefaultEventHandler(bool _on);

  /// Return a pointer to the default event handler
  DefaultEventHandler* getDefaultEventHandler() const;

  /// Pass in true to turn headlights on, false to turn headlights off
  virtual void switchHeadlights(bool _on);

  /// True iff headlights are currently set to true
  bool checkHeadlights() const;

  /// Add a WorldNode to this Viewer. If active is true, the WorldNode will
  /// respond to user input (but it will not begin simulating unless its
  /// simulation is currently turned on).
  void addWorldNode(WorldNode* _newWorldNode, bool _active=true);

  /// Remove a WorldNode from this Viewer
  void removeWorldNode(WorldNode* _oldWorldNode);

  /// Remove the WorldNode associated with _oldWorld from this Viewer
  void removeWorldNode(std::shared_ptr<dart::simulation::World> _oldWorld);

  /// Get the WorldNode associated with the given _world. Returns nullptr if
  /// this Viewer does not contain a WorldNode associated with _world.
  WorldNode* getWorldNode(
      std::shared_ptr<dart::simulation::World> _world) const;

  /// Add an attachment to this Viewer. Note that an attachment can only be
  /// attached to one Viewer at a time.
  void addAttachment(ViewerAttachment* _attachment);

  /// Remove the attachment from this Viewer.
  void removeAttachment(ViewerAttachment* _attachment);

  /// Get the set of attachments in this Viewer
  const std::unordered_set<ViewerAttachment*>& getAttachments() const;

  /// Get the Group node that contains the LightSources for this Viewer
  ::osg::Group* getLightGroup();

  /// Get the Group node that contains the LightSources for this Viewer
  const ::osg::Group* getLightGroup() const;

  /// Set up the default lighting scheme
  void setupDefaultLights();

  /// Set the direction that this Viewer should consider to be upwards (default
  /// is <0,0,1>)
  void setUpwardsDirection(const ::osg::Vec3& _up);

  /// Set the direction that this Viewer should consider to be upwards (default
  /// is <0,0,1>)
  void setUpwardsDirection(const Eigen::Vector3d& _up);

  /// Set the given WorldNode to active
  void setWorldNodeActive(WorldNode* _node, bool _active=true);

  /// Set the given World to active
  void setWorldNodeActive(std::shared_ptr<dart::simulation::World> _world,
                          bool _active=true);

  /// Set all currently active WorldNodes to simulate _on
  void simulate(bool _on);

  /// Return true iff this Viewer is currently set to simulate
  bool isSimulating() const;

  /// Prevent simulation from starting, even if simulate(true) is called
  void allowSimulation(bool _allow);

  /// Return true iff this Viewer is currently allowing simulation to happen
  bool isAllowingSimulation() const;

  /// Returns a nullptr if _entity is not a type that can support the built-in
  /// drag and drop features, otherwise it returns a pointer to the DragAndDrop
  /// interface object that has been created (allowing you to configure it). If
  /// a DragAndDrop interface already existed for this object, the existing one
  /// will be returned.
  DragAndDrop* enableDragAndDrop(dart::dynamics::Entity* _entity);

  /// A version of enableDragAndDrop specifically for SimpleFrame objects.
  SimpleFrameDnD* enableDragAndDrop(dart::dynamics::SimpleFrame* _frame);

  /// A version of enableDragAndDrop specifically for a single shape within a
  /// SimpleFrame object. Dragging and Dropping the shape will also drag and
  /// drop the entire Frame
  SimpleFrameShapeDnD* enableDragAndDrop(dart::dynamics::SimpleFrame* _frame,
                                         dart::dynamics::Shape* _shape);

  /// A version of enableDragAndDrop specifically for InteractiveFrames
  InteractiveFrameDnD* enableDragAndDrop(InteractiveFrame* _frame);

  /// A version of enableDragAndDrop specifically for BodyNodes
  BodyNodeDnD* enableDragAndDrop(dart::dynamics::BodyNode* _bn,
                                 bool _useExternalIK = true,
                                 bool _useWholeBody = false);

  /// Delete a DragAndDrop object. Returns true if the DnD was active and has
  /// now been deleted
  bool disableDragAndDrop(DragAndDrop* _dnd);

  /// Delete a SimpleFrameDnD object. Returns true if the DnD was active and has
  /// now been deleted
  bool disableDragAndDrop(SimpleFrameDnD* _dnd);

  /// Delete a SimpleFrameShapeDnD object. Returns true if a DnD existed and
  /// could be deleted
  bool disableDragAndDrop(SimpleFrameShapeDnD* _dnd);

  /// Delete an InteractiveFrameDnD object. Returns true if a DnD existed and
  /// could be deleted
  bool disableDragAndDrop(InteractiveFrameDnD* _dnd);

  /// Delete a BodyNodeDnD object. Returns true if a DnD existed and could be
  /// deleted
  bool disableDragAndDrop(BodyNodeDnD* _dnd);

  /// Get a string containing the user interface constructions for this Viewer
  const std::string& getInstructions() const;

  /// Add something to the instructions for this Viewer. You are strongly
  /// recommended to end your string with an end-of-line character. An
  /// end-of-line character will NOT be added automatically.
  void addInstructionText(const std::string& _instruction);

  /// Called automatically at the beginning of each render cycle
  virtual void updateViewer();

  /// Called automatically by updateViewer()
  void updateDragAndDrops();

  /// Get the root ::osg::Group of this Viewer
  const ::osg::ref_ptr<::osg::Group>& getRootGroup() const;

protected:

  friend class SaveScreen;

  /// Current number of the image sequence for screen recording
  std::size_t mImageSequenceNum;

  /// Number of digits to use when saving an image sequence
  std::size_t mImageDigits;

  /// Whether or not the Viewer is currently recording
  bool mRecording;

  /// Whether or not the Viewer is staged for a screen capture
  bool mScreenCapture;

  /// Directory for saving images
  std::string mImageDirectory;

  /// Prefix to apply to images
  std::string mImagePrefix;

  /// Name for the next screen capture
  std::string mScreenCapName;

  /// Default WorldNodeEventHandler for this dart::gui::osg::Viewer
  ::osg::ref_ptr<DefaultEventHandler> mDefaultEventHandler;

  /// The root node of this Viewer
  ::osg::ref_ptr<::osg::Group> mRootGroup;

  /// The Group Node containing light sources
  ::osg::ref_ptr<::osg::Group> mLightGroup;

  /// Non-headlights Light #1
  ::osg::ref_ptr<::osg::Light> mLight1;

  /// Non-headlights LightSource #1
  ::osg::ref_ptr<::osg::LightSource> mLightSource1;

  /// Non-headlights Light #2
  ::osg::ref_ptr<::osg::Light> mLight2;

  /// Non-headlights LightSource #2
  ::osg::ref_ptr<::osg::LightSource> mLightSource2;

  /// Vector pointing upwards
  ::osg::Vec3 mUpwards;

  /// Vector pointing to the side
  ::osg::Vec3 mOver;

  /// True iff this Viewer is currently simulating
  bool mSimulating;

  /// True iff this Viewer is allowing simulation
  bool mAllowSimulation;

  /// True iff headlights were last set to be on
  bool mHeadlights;

  /// Map of WorldNodes in this dart::gui::osg::Viewer. A WorldNode will map to
  /// true iff it is currently active
  std::map<WorldNode*,bool> mWorldNodes;

  std::unordered_set<ViewerAttachment*> mAttachments;

  /// string of instructions for this Viewer
  std::string mInstructions;

  // TODO(MXG): Consolidate all these maps into a single map of maps which uses
  // typeid as a key

  /// Map from SimpleFrame ptrs to SimpleFrameDnD ptrs
  std::map<dart::dynamics::SimpleFrame*,SimpleFrameDnD*> mSimpleFrameDnDMap;

  /// Multimap from Shape ptrs to SimpleFrameShapeDnD ptrs. We use a multimap
  /// in order to support the possibility of a single Shape being used by
  /// multiple objects
  std::multimap<dart::dynamics::Shape*,SimpleFrameShapeDnD*> mSimpleFrameShapeDnDMap;

  /// Map from InteractiveFrame ptrs to InteractiveFrameDnD ptrs
  std::map<InteractiveFrame*,InteractiveFrameDnD*> mInteractiveFrameDnDMap;

  /// Map from BodyNode ptrs to BodyNodeDnD ptrs
  std::map<dart::dynamics::BodyNode*,BodyNodeDnD*> mBodyNodeDnDMap;
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_VIEWER_HPP_
