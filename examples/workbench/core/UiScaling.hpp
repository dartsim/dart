#ifndef DART_DOC_EXAMPLES_WORKBENCH_CORE_UISCALING_HPP_
#define DART_DOC_EXAMPLES_WORKBENCH_CORE_UISCALING_HPP_

namespace workbench {

/// Detects the preferred UI scale factor based on environment variables or,
/// as a fallback, the current display DPI. The returned value is clamped to the
/// [0.5, 3.0] range.
float detectUiScale();

} // namespace workbench

#endif // DART_DOC_EXAMPLES_WORKBENCH_CORE_UISCALING_HPP_
