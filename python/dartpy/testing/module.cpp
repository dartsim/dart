// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <dart/dynamics/SphereShape.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>

#include <exception>
#include <iomanip>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace dart::python {

// dartpy._testing: hooks for dartpy's own test suite, not part of the API.
void dart_testing(nb::module_& m)
{
  auto sm = m.def_submodule(
      "_testing", "Hooks for dartpy's own tests; not part of the API.");

  // Logs from C++ worker threads through DART's three paths, three records
  // per message: the DART_WARN family instantiated in this extension, a
  // diagnostic emitted inside libdart (SphereShape rejects a non-positive
  // radius) and the console macros. The console line uses per-thread width,
  // fill and base manipulators, so shared stream state would corrupt it. Joins
  // with the GIL held or released.
  sm.def(
      "log_from_threads",
      [](int threads, int messages, bool holdGil) {
        auto run = [&] {
          std::vector<std::thread> workers;
          try {
            for (int t = 0; t < threads; ++t) {
              workers.emplace_back([t, messages] {
                dynamics::SphereShape shape(1.0);
                for (int i = 0; i < messages; ++i) {
                  common::warn("worker {} spdlog {}", t, i);
                  shape.setRadius(-(1.0 + i));
                  dtwarn << "worker " << std::setw(2 + t) << std::setfill('0')
                         << t << std::setfill(' ') << " console " << std::hex
                         << i << std::dec << "\n";
                }
              });
            }
          } catch (...) {
            // A thread that failed to start leaves the earlier ones running;
            // joining them keeps ~std::thread from calling std::terminate.
            for (auto& worker : workers)
              worker.join();
            throw;
          }
          for (auto& worker : workers)
            worker.join();
        };
        if (holdGil) {
          run();
        } else {
          nb::gil_scoped_release release;
          run();
        }
      },
      nb::arg("threads"),
      nb::arg("messages"),
      nb::arg("hold_gil"));

  // Logs from a thread_local destructor that runs after the bridge's own
  // per-thread console stream is gone. Thread-local objects are destroyed in
  // reverse construction order, so one constructed before the thread's first
  // log outlives the stream.
  sm.def("log_from_late_thread_local_destructor", [] {
    struct LateLogger
    {
      ~LateLogger()
      {
        dtwarn << "late thread_local console\n";
        common::warn("late thread_local spdlog");
      }
    };
    std::thread([] {
      thread_local LateLogger late;
      (void)late;
      dtwarn << "early thread_local console\n";
    }).join();
  });

  // Logs from a static destructor, which runs after Python has finalized and
  // after the main thread's console stream is gone.
  sm.def("log_from_static_destructor", [] {
    struct ExitLogger
    {
      ~ExitLogger()
      {
        dtwarn << "static destructor console\n";
        common::warn("static destructor spdlog");
      }
    };
    static ExitLogger exitLogger;
    (void)exitLogger;
  });

  // Throws the named C++ exception, to test the translator. "int" throws a
  // non-std exception.
  sm.def(
      "throw_",
      [](const std::string& kind) {
        if (kind == "int")
          throw 42;
        if (kind == "bad_alloc")
          throw std::bad_alloc();
        if (kind == "exception")
          throw std::exception();
        if (kind == "logic_error")
          throw std::logic_error("logic");
        if (kind == "out_of_range")
          throw std::out_of_range("out of range");
        if (kind == "overflow_error")
          throw std::overflow_error("overflow");
        if (kind == "runtime_error")
          throw std::runtime_error("runtime");
        if (kind == "underflow_error")
          throw std::underflow_error("underflow");
        throw std::invalid_argument("unknown exception kind: " + kind);
      },
      nb::arg("kind"));
}

} // namespace dart::python
