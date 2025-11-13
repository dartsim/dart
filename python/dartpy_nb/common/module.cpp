#include "common/module.hpp"

#include "common/logging.hpp"
#include "common/stopwatch.hpp"
#include "common/string.hpp"
#include "common/uri.hpp"

namespace dart::python_nb {

void defCommonModule(nanobind::module_& m)
{
  defLogging(m);
  defStopwatch(m);
  defString(m);
  defUri(m);
}

} // namespace dart::python_nb
