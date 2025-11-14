#include "utils/module.hpp"

#include "utils/dart_loader.hpp"
#include "utils/mjcf_parser.hpp"
#include "utils/resource_retriever.hpp"
#include "utils/sdf_parser.hpp"
#include "utils/skel_parser.hpp"

namespace dart::python_nb {

void defUtilsModule(nanobind::module_& m)
{
  auto sm = m.def_submodule("utils");
  defUtilsResourceRetriever(sm);
  defDartLoader(sm);
  defSkelParser(sm);
  defSdfParser(sm);
  defMjcfParser(sm);
}

} // namespace dart::python_nb
