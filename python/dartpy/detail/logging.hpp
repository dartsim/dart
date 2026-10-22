#pragma once

#include <cstddef>

namespace dartnb {
// Routes DART's C++ diagnostics (the dtwarn/dterr console streams through
// dart::common::setConsoleStreams(), the DART_WARN family from libdart and
// this extension alike through dart::common::setLogCallback()) into
// logging.getLogger("dartpy").
//
// Logging threads never touch Python: each thread formats its own console
// lines, records go into a bounded queue, and Py_AddPendingCall drains it on
// the main thread at its next safe point. The queue drops its oldest record
// when full and reports the count. Python's atexit flips the bridge to
// pass-through, so diagnostics logged after that go straight to the console.
// On POSIX, fork handlers hand the child an unlocked, empty queue. Idempotent;
// call once at import with the GIL.
void installLogBridge();

// Delivers queued records on the calling thread and returns how many it
// drained, counting the drop notice. Requires the GIL.
std::size_t flushLogs();
} // namespace dartnb
