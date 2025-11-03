#include "WorkbenchWorldNode.hpp"

namespace workbench {

void WorkbenchWorldNode::setServicesPtr(WorkbenchServices* services)
{
  mServices = services;
}

void WorkbenchWorldNode::setActiveSession(
    const std::shared_ptr<ExampleSession>& session)
{
  mSession = session;
  if (mSession)
    setWorld(mSession->world);
  else
    setWorld(nullptr);
}

void WorkbenchWorldNode::clearSession()
{
  setActiveSession(nullptr);
}

void WorkbenchWorldNode::customPreRefresh()
{
  if (mSession && mServices && mSession->onPreRefresh)
    mSession->onPreRefresh(*mServices);
}

void WorkbenchWorldNode::customPostRefresh()
{
  if (mSession && mServices && mSession->onPostRefresh)
    mSession->onPostRefresh(*mServices);
}

void WorkbenchWorldNode::customPreStep()
{
  if (mSession && mServices && mSession->onStep && getWorld())
    mSession->onStep(*mServices, getWorld()->getTimeStep());
}

} // namespace workbench
