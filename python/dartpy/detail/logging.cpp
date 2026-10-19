// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/logging.hpp"

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>

#ifndef _WIN32
  #include <pthread.h>
#endif

#include <deque>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <ostream>
#include <streambuf>
#include <string>
#include <string_view>
#include <utility>

namespace dartnb {
namespace {

// Python logging levels.
constexpr int kTrace = 5;
constexpr int kDebug = 10;
constexpr int kInfo = 20;
constexpr int kWarning = 30;
constexpr int kError = 40;
constexpr int kCritical = 50;

struct Record
{
  int level;
  std::string message;
};

// Writes a line to stderr without touching Python.
void print(int level, const std::string& message)
{
  const char* name = level >= kCritical  ? "critical"
                     : level >= kError   ? "error"
                     : level >= kWarning ? "warning"
                     : level >= kInfo    ? "info"
                                         : "debug";
  // One insertion, so concurrent lines do not interleave.
  std::cerr << ("[dartpy " + std::string(name) + "] " + message + '\n');
}

std::string dropNotice(std::size_t dropped, std::size_t capacity)
{
  return "dartpy dropped " + std::to_string(dropped)
         + " log records: the queue holds " + std::to_string(capacity)
         + " and nothing drained it in time";
}

// Nothing may propagate through the C frames of Py_AddPendingCall or a
// std::streambuf.
template <typename F>
void guarded(F&& call)
{
  try {
    call();
  } catch (nb::python_error& e) {
    e.discard_as_unraisable("dartpy log bridge");
  } catch (const std::exception& e) {
    print(kError, std::string("dartpy log bridge: ") + e.what());
  } catch (...) {
    print(kError, "dartpy log bridge: unknown C++ exception");
  }
}

class Queue
{
public:
  static constexpr std::size_t kCapacity = 1024;

  // Any thread, with or without the GIL.
  void push(int level, std::string message)
  {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mPassthrough) {
      lock.unlock();
      print(level, message);
      return;
    }
    if (mRecords.size() >= kCapacity) {
      mRecords.pop_front();
      ++mDropped;
    }
    mRecords.push_back({level, std::move(message)});
    if (mScheduled)
      return;
    // Still under the lock, so the decision is serialized with shutdown().
    // Py_AddPendingCall never waits for the GIL. When the interpreter's
    // pending-call table is full, flushLogs() or the next push retries.
    mScheduled = Py_AddPendingCall(&Queue::pending, this) == 0;
  }

  // Delivers the queued records on the calling thread and returns how many
  // records it drained, counting the drop notice. Requires the GIL.
  std::size_t drain()
  {
    std::deque<Record> records;
    std::size_t dropped = 0;
    bool passthrough = false;
    {
      std::lock_guard<std::mutex> lock(mMutex);
      records.swap(mRecords);
      dropped = std::exchange(mDropped, 0);
      passthrough = mPassthrough;
      // Pushes during delivery schedule a new drain.
      mScheduled = false;
    }
    if (dropped > 0)
      records.push_back({kWarning, dropNotice(dropped, kCapacity)});
    if (records.empty())
      return 0;
    if (passthrough) {
      for (const auto& record : records)
        print(record.level, record.message);
      return records.size();
    }
    nb::object log;
    guarded([&] {
      log = nb::module_::import_("logging").attr("getLogger")("dartpy").attr(
          "log");
    });
    for (const auto& record : records) {
      if (!log.is_valid())
        print(record.level, record.message);
      else
        guarded([&] { log(record.level, record.message); });
    }
    return records.size();
  }

  // Python can no longer be entered: deliver what is queued to the console.
  void shutdown()
  {
    std::deque<Record> records;
    std::size_t dropped = 0;
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mPassthrough = true;
      records.swap(mRecords);
      dropped = std::exchange(mDropped, 0);
    }
    for (const auto& record : records)
      print(record.level, record.message);
    if (dropped > 0)
      print(kWarning, dropNotice(dropped, kCapacity));
  }

  // pthread_atfork handlers: the queue mutex is the only lock on the bridged
  // path, and holding it across fork() keeps a child forked from a
  // single-threaded process able to log and flush. Forking a process with other
  // running threads is unsupported: a thread the child does not have may hold
  // a lock below the bridge (libstdc++'s locale mutex, which any stream
  // construction takes under a non-classic global locale, or a spdlog sink),
  // and CPython warns about such forks; use spawn or forkserver instead. The
  // parent keeps delivering what it queued; the child starts empty, since its
  // pending call may not survive the fork.
  static void beforeFork()
  {
    queue().mMutex.lock();
  }

  static void afterForkParent()
  {
    queue().mMutex.unlock();
  }

  static void afterForkChild()
  {
    Queue& self = queue();
    self.mRecords.clear();
    self.mDropped = 0;
    self.mScheduled = false;
    self.mMutex.unlock();
  }

  // Never destroyed: DART statistics may log after Python has shut down.
  static Queue& queue()
  {
    static Queue* const instance = new Queue();
    return *instance;
  }

private:
  static int pending(void* self)
  {
    guarded([self] { static_cast<Queue*>(self)->drain(); });
    return 0;
  }

  std::mutex mMutex;
  std::deque<Record> mRecords;
  std::size_t mDropped = 0;
  bool mScheduled = false;
  bool mPassthrough = false;
};

// Receives one thread's dtmsg/dtdbg/dtwarn/dterr console output and the
// spdlog-free DART_WARN fallback. The colored level header that Console.cpp
// and Logging-impl.hpp write selects the Python level.
class LineBuffer final : public std::streambuf
{
public:
  ~LineBuffer() override
  {
    guarded([this] { complete(); });
  }

protected:
  int_type overflow(int_type ch) override
  {
    if (ch != traits_type::eof())
      put(static_cast<char>(ch));
    return ch;
  }

  std::streamsize xsputn(const char* data, std::streamsize count) override
  {
    for (std::streamsize i = 0; i < count; ++i)
      put(data[i]);
    return count;
  }

  // Delivers the unterminated text, if any.
  int sync() override
  {
    complete();
    return 0;
  }

private:
  void put(char ch)
  {
    if (ch == '\n')
      complete();
    else
      mText += ch;
  }

  void complete()
  {
    if (mText.empty())
      return;
    std::string message = stripAnsi(mText);
    mText.clear();
    if (!message.empty() && message.back() == '\r')
      message.pop_back();
    classify(message);
    if (!message.empty())
      Queue::queue().push(mLevel, std::move(message));
  }

  // "Warning [File.cpp:12] text" from dtwarn, "[warn] text" from the
  // spdlog-free DART_WARN fallback. Other lines continue the previous
  // message at its level.
  void classify(std::string& message)
  {
    static constexpr std::pair<std::string_view, int> kHeaders[] = {
        {"Warning [", kWarning},
        {"Error [", kError},
        {"Msg ", kInfo},
        {"Dbg ", kDebug},
        {"[trace] ", kTrace},
        {"[debug] ", kDebug},
        {"[info] ", kInfo},
        {"[warn] ", kWarning},
        {"[error] ", kError},
        {"[fatal] ", kCritical},
    };
    for (const auto& [header, headerLevel] : kHeaders) {
      if (message.compare(0, header.size(), header) != 0)
        continue;
      mLevel = headerLevel;
      // Keep "[File.cpp:12] text"; drop the level word.
      const std::size_t keep
          = header.back() == '[' ? header.size() - 1 : header.size();
      message.erase(0, keep);
      return;
    }
  }

  static std::string stripAnsi(std::string_view text)
  {
    std::string result;
    result.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
      if (text[i] == '\033' && i + 1 < text.size() && text[i + 1] == '[') {
        const std::size_t end = text.find('m', i);
        if (end == std::string_view::npos)
          break;
        i = end;
        continue;
      }
      result += text[i];
    }
    return result;
  }

  std::string mText;
  int mLevel = kWarning;
};

// Each thread writes to its own ostream, so manipulators such as std::setw
// and std::hex, the stream state and the line buffer are never shared.
std::ostream& consoleStream()
{
  struct ThreadConsole
  {
    explicit ThreadConsole(bool* destroyed) : mDestroyed(destroyed) {}
    ~ThreadConsole()
    {
      if (mDestroyed)
        *mDestroyed = true;
    }
    LineBuffer buffer;
    std::ostream stream{&buffer};
    bool* mDestroyed;
  };
  // Thread-local and static destructors that run after this thread's console
  // is destroyed may still log. Both flags are trivially destructible, so they
  // stay readable then, and such a late writer gets a console that leaks
  // rather than one that is gone.
  thread_local bool destroyed = false;
  thread_local ThreadConsole* late = nullptr;
  if (destroyed) {
    if (!late)
      late = new ThreadConsole(nullptr);
    return late->stream;
  }
  thread_local ThreadConsole console(&destroyed);
  return console.stream;
}

int pythonLevel(dart::common::LogLevel level)
{
  using dart::common::LogLevel;
  switch (level) {
    case LogLevel::Trace:
      return kTrace;
    case LogLevel::Debug:
      return kDebug;
    case LogLevel::Info:
      return kInfo;
    case LogLevel::Warn:
      return kWarning;
    case LogLevel::Error:
      return kError;
    default:
      return kCritical;
  }
}

// dart::common::LogCallback: the DART_WARN family from libdart and from this
// extension, on the logging thread. The only lock on this path is the queue's,
// which the fork handlers cover. Every compile-time-enabled message arrives
// here, whatever spdlog's runtime level, and Python's logger filters it. No
// level check happens before the queue: logging offers no hook that fires when
// a level changes, so a cached threshold could go stale and silently drop the
// messages a user just enabled, and DART formats the message before this call
// anyway, so the check would save only a string copy and a push.
void onLog(
    dart::common::LogLevel level,
    const char* message,
    std::size_t length,
    void* /*userData*/)
{
  guarded([&] {
    Queue::queue().push(pythonLevel(level), std::string(message, length));
  });
}

void shutdown()
{
  Queue::queue().shutdown();
}

} // namespace

void installLogBridge()
{
  static bool installed = false;
  if (installed)
    return;
  installed = true;

  dart::common::setConsoleStreams(&consoleStream, &consoleStream);
  // Not a spdlog logger: with a header-only spdlog, libdart and this extension
  // have separate registries, and a logger installed here would miss libdart's
  // own diagnostics. Python logging filters by level.
  dart::common::setLogCallback(&onLog);

#ifndef _WIN32
  pthread_atfork(
      &Queue::beforeFork, &Queue::afterForkParent, &Queue::afterForkChild);
#endif

  // Python atexit runs while logging still works, so queued records drain
  // there; Py_AtExit covers embedding hosts that skip the atexit module.
  nb::module_::import_("atexit").attr("register")(nb::cpp_function([] {
    Queue::queue().drain();
    shutdown();
  }));
  Py_AtExit(&shutdown);
}

std::size_t flushLogs()
{
  consoleStream().flush();
  return Queue::queue().drain();
}

} // namespace dartnb
