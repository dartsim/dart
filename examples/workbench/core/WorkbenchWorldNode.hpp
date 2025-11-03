#ifndef DART_DOC_EXAMPLES_WORKBENCH_CORE_WORKBENCHWORLDNODE_HPP_
#define DART_DOC_EXAMPLES_WORKBENCH_CORE_WORKBENCHWORLDNODE_HPP_

#include <dart/gui/osg/RealTimeWorldNode.hpp>

#include <memory>

#include "WorkbenchTypes.hpp"

namespace workbench {

class WorkbenchWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  using dart::gui::osg::RealTimeWorldNode::RealTimeWorldNode;

  void setServicesPtr(WorkbenchServices* services);
  void setActiveSession(const std::shared_ptr<ExampleSession>& session);
  void clearSession();

  void customPreRefresh() override;
  void customPostRefresh() override;
  void customPreStep() override;

private:
  std::shared_ptr<ExampleSession> mSession;
  WorkbenchServices* mServices{nullptr};
};

} // namespace workbench

#endif // DART_DOC_EXAMPLES_WORKBENCH_CORE_WORKBENCHWORLDNODE_HPP_
