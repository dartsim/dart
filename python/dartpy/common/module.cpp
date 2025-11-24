#include "common/module.hpp"

#include "common/composite.hpp"
#include "common/logging.hpp"
#include "common/observer.hpp"
#include "common/resource.hpp"
#include "common/resource_retriever.hpp"
#include "common/stopwatch.hpp"
#include "common/string.hpp"
#include "common/subject.hpp"
#include "common/uri.hpp"

namespace dart::python_nb {

void defCommonModule(nanobind::module_& m)
{
  defLogging(m);
  defObserver(m);
  defSubject(m);
  defComposite(m);
  defResource(m);
  defResourceRetriever(m);
  defStopwatch(m);
  defString(m);
  defUri(m);
}

} // namespace dart::python_nb
