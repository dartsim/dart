/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dart.h"

constexpr double default_angle = 20.0*M_PI/180.0;
constexpr double default_distance = 0.01;
constexpr double default_domino_height = 0.07;
constexpr double default_domino_width = 0.03;

using namespace dart::dynamics;

class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(dart::simulation::WorldPtr world)
  {

  }

  void keyboard(unsigned char key, int x, int y) override
  {
    if(!_hasEverRun)
    {
      switch(key)
      {
        case 'q':
          attemptToCreateDomino( default_angle);
        case 'w':
          attemptToCreateDomino(0.0);
        case 'e':
          attemptToCreateDomino(-default_angle);
        case 'd':
          deleteLastDomino();
        case ' ':
          _hasEverRun = true;
      }
    }

    SimWindow::keyboard(key, x, y);
  }


  void attemptToCreateDomino(double angle)
  {

  }

  void deleteLastDomino()
  {

  }


protected:

  /// History of the dominoes that have been created
  std::vector<SkeletonPtr> _dominoes;

  /// History of the angles that the user has specified
  std::vector<double> _angles;

  /// Sum of all angles so far
  double _totalAngle;

  /// Set to true the first time spacebar is pressed
  bool _hasEverRun;

};


int main(int argc, char* argv[])
{

}
