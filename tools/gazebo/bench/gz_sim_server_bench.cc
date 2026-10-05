// End-to-end gz-sim benchmark: run an SDF world headless in a gz::sim::Server
// for a number of iterations and report wall-clock time per iteration. Use a
// world with <real_time_factor>0</real_time_factor> so the server is not
// throttled to real time.
//
// usage: gz_sim_server_bench [--engine <physics-plugin.so>] <world.sdf>
//            <iterations> [chunk]
//
// Prints the time per iteration for every chunk of iterations, then a summary
// (the real-time factor assumes the 1 ms step of the gz-sim example worlds).
#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>

#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv)
{
  std::string engine;
  int arg = 1;
  if (argc > 2 && std::string(argv[1]) == "--engine") {
    engine = argv[2];
    if (!std::filesystem::is_regular_file(engine)) {
      std::fprintf(stderr, "engine must name an existing plugin file\n");
      return 2;
    }
    arg = 3;
  }
  if (argc - arg < 2) {
    std::fprintf(
        stderr,
        "usage: %s [--engine <physics-plugin.so>] <world.sdf> <iterations> "
        "[chunk]\n",
        argv[0]);
    return 2;
  }
  const std::string world = argv[arg];
  char* end;
  errno = 0;
  const long iterations = std::strtol(argv[arg + 1], &end, 10);
  if (errno != 0 || end == argv[arg + 1] || *end != '\0' || iterations <= 0) {
    std::fprintf(stderr, "iterations must be positive\n");
    return 2;
  }
  const long chunk
      = argc - arg > 2 ? std::max(1L, std::atol(argv[arg + 2])) : iterations;

  // Errors only: the server's progress messages would swamp the timings.
  gz::common::Console::SetVerbosity(1);
  gz::sim::ServerConfig config;
  if (!config.SetSdfFile(world)) {
    std::fprintf(stderr, "world file must not be empty\n");
    return 2;
  }
  if (!engine.empty())
    config.SetPhysicsEngine(engine);

  using Clock = std::chrono::steady_clock;
  const auto loadStart = Clock::now();
  gz::sim::Server server(config);
  std::printf(
      "world=%s engine=%s load_s=%.3f\n",
      world.c_str(),
      engine.empty() ? "default" : engine.c_str(),
      std::chrono::duration<double>(Clock::now() - loadStart).count());
  std::fflush(stdout);

  long done = 0;
  double total = 0.0;
  while (done < iterations) {
    const long n = std::min(chunk, iterations - done);
    const auto before = server.IterationCount();
    const auto start = Clock::now();
    const bool ok = server.Run(true, static_cast<std::uint64_t>(n), false);
    const double seconds
        = std::chrono::duration<double>(Clock::now() - start).count();
    // Run() also returns true when a signal stopped the server early, and a
    // world that failed to load has no iteration count.
    const auto after = server.IterationCount();
    const std::uint64_t ran = before && after ? *after - *before : 0u;
    if (!ok || ran != static_cast<std::uint64_t>(n)) {
      std::fprintf(
          stderr,
          "error: the server ran %llu of %ld iterations after iteration %ld\n",
          static_cast<unsigned long long>(ran),
          n,
          done);
      return 1;
    }
    total += seconds;
    done += n;
    std::printf(
        "iter=%ld chunk_ms_per_iter=%.4f chunk_rtf=%.3f\n",
        done,
        1e3 * seconds / static_cast<double>(n),
        1e-3 * static_cast<double>(n) / seconds);
    std::fflush(stdout);
  }
  std::printf(
      "SUMMARY iterations=%ld wall_s=%.3f ms_per_iter=%.4f rtf=%.3f\n",
      iterations,
      total,
      1e3 * total / static_cast<double>(iterations),
      1e-3 * static_cast<double>(iterations) / total);
  return 0;
}
