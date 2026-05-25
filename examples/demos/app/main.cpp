/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/gui/application.hpp>

// dart-demos: a single window hosting all of DART's GUI examples as scenes that
// the user picks from a categorized sidebar and switches between at runtime.
int main(int argc, char* argv[])
{
  return dart::gui::runDemos(
      argc, argv, dart::examples::demos::makeDemoScenes());
}
