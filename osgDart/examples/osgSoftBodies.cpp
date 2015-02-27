/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "osgDart/osgDart.h"

#include <osgViewer/Viewer>

int main()
{
  using namespace dart::dynamics;

  dart::simulation::World* world =
      dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/softBodies.skel");

  osg::ref_ptr<osgDart::WorldNode> node = new osgDart::WorldNode(world);

  node->simulate(true);
  node->setNumStepsPerCycle(20);

  osgViewer::Viewer viewer;
  viewer.getCamera()->setClearColor(osg::Vec4(0.9,0.9,0.9,1.0));
  viewer.setSceneData(node);


//  viewer.getLight()->setDiffuse(osg::Vec4(0.1,0.1,0.1,1));

    viewer.getLight()->setDiffuse(osg::Vec4(0.2,0.2,0.2,1));

//    viewer.getLight()->setDiffuse(osg::Vec4(0.4,0.4,0.4,1));

//    viewer.getLight()->setDiffuse(osg::Vec4(0.3,0.3,0.3,1));

//  osg::ref_ptr<osg::Light> testLight = new osg::Light;
//  testLight->setThreadSafeRefUnref(true);
//  testLight->setLightNum(1);
//  testLight->setAmbient(osg::Vec4(0,0,0,1));
//  testLight->setDiffuse(osg::Vec4(0.2,0.2,0.2,1.0));
//  testLight->setSpecular(osg::Vec4(0.25,0.25,0.25,1.0));
//  testLight->setPosition(osg::Vec4(-1,0,-1,0));


  osg::ref_ptr<osg::Group> lightGroup = new osg::Group;
  osg::ref_ptr<osg::StateSet> lightSS = node->getOrCreateStateSet();
  osg::ref_ptr<osg::LightSource> lightSource1 = new osg::LightSource;
  osg::ref_ptr<osg::LightSource> lightSource2 = new osg::LightSource;

  osg::ref_ptr<osg::Light> light1 = new osg::Light;
  light1->setLightNum(1);
  light1->setPosition(osg::Vec4(1,0,-1,0));
  light1->setAmbient(osg::Vec4(0.1,0.1,0.1,1.0));
  light1->setDiffuse(osg::Vec4(0.7,0.7,0.7,1.0));
//  light1->setSpecular(osg::Vec4(0.9,0.9,0.9,1.0));
  light1->setSpecular(osg::Vec4(0.0,0.0,0.0,1.0));
  lightSource1->setLight(light1);

  lightSource1->setLocalStateSetModes(osg::StateAttribute::ON);
  lightSource1->setStateSetModes(*lightSS, osg::StateAttribute::ON);
  lightGroup->addChild(lightSource1);

  osg::ref_ptr<osg::Light> light2 = new osg::Light;
  light2->setLightNum(2);
  light2->setPosition(osg::Vec4(-1,0,-1,0));
  light2->setAmbient(osg::Vec4(0,0,0,1));
  light2->setDiffuse(osg::Vec4(0.3,0.3,0.3,1.0));
  light2->setSpecular(osg::Vec4(0.225,0.225,0.255,1.0));
  lightSource2->setLight(light2);

  lightSource2->setLocalStateSetModes(osg::StateAttribute::ON);
  lightSource2->setStateSetModes(*lightSS, osg::StateAttribute::ON);
  lightGroup->addChild(lightSource2);



  node->addChild(lightGroup);


  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
