#include "robots/atlas.hpp"
#include "robots/g1.hpp"
#include "robots/hubo.hpp"

#include <CLI/CLI.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace {

const std::vector<std::string> kRobots = {"atlas", "g1", "hubo"};

}

int main(int argc, char* argv[])
{
  CLI::App app("Inverse kinematics humanoid example.");

  std::string robot = "atlas";
  app.add_option("-r,--robot", robot, "Robot to load: atlas, g1, hubo.")
      ->check(CLI::IsMember(kRobots));

  G1Options g1;
  auto* g1Group = app.add_option_group("G1 options");
  auto* packageUriOpt = g1Group->add_option(
      "-p,--package-uri",
      g1.packageUri,
      "ROS package root for package:// URIs (file:// or http(s)://).\n"
      "Only used with --robot g1.");
  auto* robotUriOpt = g1Group->add_option(
      "--robot-uri",
      g1.robotUri,
      "URDF/SDF entry point to load (package://, file://, or http(s)://).\n"
      "Only used with --robot g1.");
  auto* packageNameOpt = g1Group->add_option(
      "--package-name",
      g1.packageName,
      "Override the ROS package name registered with the package URI.\n"
      "Only used with --robot g1.");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  finalizeG1Options(
      g1,
      packageNameOpt->count() > 0,
      packageUriOpt->count() > 0,
      robotUriOpt->count() > 0);

  if (robot == "atlas")
    return runAtlas();
  if (robot == "g1")
    return runG1(g1);
  if (robot == "hubo")
    return runHubo();

  std::cerr << "Unknown robot: " << robot << "\n";
  return 1;
}
