#include "io/module.hpp"

#include "io/mjcf_parser.hpp"
#include "io/resource_retriever.hpp"
#include "io/sdf_parser.hpp"
#include "io/urdf_parser.hpp"

namespace dart::python_nb {

void defIoModule(nanobind::module_& m)
{
  defIoResourceRetriever(m);
  defUrdfParser(m);
  defSdfParser(m);
  defMjcfParser(m);
}

} // namespace dart::python_nb
