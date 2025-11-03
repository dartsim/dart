#ifndef DART_DOC_EXAMPLES_WORKBENCH_REGISTRY_EXAMPLEREGISTRY_HPP_
#define DART_DOC_EXAMPLES_WORKBENCH_REGISTRY_EXAMPLEREGISTRY_HPP_

#include "../core/WorkbenchTypes.hpp"

#include <filesystem>

namespace workbench {

ExampleList loadExampleRecords(const std::filesystem::path& examplesRoot);
std::filesystem::path detectExamplesRoot();

} // namespace workbench

#endif // DART_DOC_EXAMPLES_WORKBENCH_REGISTRY_EXAMPLEREGISTRY_HPP_
