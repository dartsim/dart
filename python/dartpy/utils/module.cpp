#include "utils/module.hpp"

#include "utils/dart_loader.hpp"
#include "utils/mjcf_parser.hpp"
#include "utils/resource_retriever.hpp"
#include "utils/sdf_parser.hpp"
#include "utils/skel_parser.hpp"

namespace dart::python_nb {

void defUtilsModule(nanobind::module_& m)
{
  defUtilsResourceRetriever(m);
  defDartLoader(m);
  defSkelParser(m);
  defSdfParser(m);
  defMjcfParser(m);
}

} // namespace dart::python_nb
