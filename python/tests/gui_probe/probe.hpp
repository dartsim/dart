#pragma once

#include <dart/gui/osg/Viewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>

#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>

#include <vector>

namespace dart::python::gui_probe {

inline void refresh(dart::gui::osg::WorldNode* node)
{
  node->refresh();
}

inline void refreshViewer(dart::gui::osg::Viewer* viewer)
{
  const auto& root = viewer->getRootGroup();
  std::vector<::osg::ref_ptr<dart::gui::osg::WorldNode>> nodes;
  for (unsigned int i = 0; i < root->getNumChildren(); ++i) {
    if (auto* node
        = dynamic_cast<dart::gui::osg::WorldNode*>(root->getChild(i)))
      nodes.emplace_back(node);
  }
  for (const auto& node : nodes)
    node->refresh();
}

inline bool handle(
    osgGA::GUIEventHandler* handler, osgViewer::View* viewer, int key)
{
  ::osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter;
  event->setEventType(osgGA::GUIEventAdapter::KEYDOWN);
  event->setKey(key);
  return handler->handle(*event, *viewer);
}

inline unsigned int dispatchViewerHandlers(osgViewer::View* viewer, int key)
{
  unsigned int handled = 0;
  // Copy because a Python callback may remove its own handler.
  auto handlers = viewer->getEventHandlers();
  for (const auto& handler : handlers) {
    if (auto* guiHandler = dynamic_cast<osgGA::GUIEventHandler*>(handler.get()))
      handled += handle(guiHandler, viewer, key);
  }
  return handled;
}

} // namespace dart::python::gui_probe
