### Version 5.1.3 (201X-XX-XX)

1. Backported [#749](https://github.com/dartsim/dart/pull/749) to DART 5.1
    * [Pull request #759](https://github.com/dartsim/dart/pull/759)

### Version 5.1.2 (2016-04-25)

1. Fixed inverse kinematics (backporting)
    * [Pull request #684](https://github.com/dartsim/dart/pull/684)

1. Fixed aligned memory allocation with Eigen objects in loading meshes
    * [Pull request #606](https://github.com/dartsim/dart/pull/606)

1. Fixed incorrect applying joint constraint impulses (backporting)
    * [Pull request #579](https://github.com/dartsim/dart/pull/579)

1. Fixed some build and packaging issues
    * [Pull request #559](https://github.com/dartsim/dart/pull/559)
    * [Pull request #595](https://github.com/dartsim/dart/pull/595)
    * [Pull request #696](https://github.com/dartsim/dart/pull/696)

### Version 5.1.1 (2015-11-06)

1. Add bullet dependency to package.xml
    * [Pull request #523](https://github.com/dartsim/dart/pull/523)

1. Improved handling of missing symbols of Assimp package
    * [Pull request #542](https://github.com/dartsim/dart/pull/542)

1. Improved travis-ci build log for Mac
    * [Pull request #529](https://github.com/dartsim/dart/pull/529)

1. Fixed warnings in Function.cpp
    * [Pull request #550](https://github.com/dartsim/dart/pull/550)

1. Fixed build failures on AppVeyor
    * [Pull request #543](https://github.com/dartsim/dart/pull/543)

1. Fixed const qualification of ResourceRetriever
    * [Pull request #534](https://github.com/dartsim/dart/pull/534)
    * [Issue #532](https://github.com/dartsim/dart/issues/532)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #527](https://github.com/dartsim/dart/pull/527)

1. Fixed copy safety for various classes
    * [Pull request #526](https://github.com/dartsim/dart/pull/526)
    * [Pull request #539](https://github.com/dartsim/dart/pull/539)
    * [Issue #524](https://github.com/dartsim/dart/issues/524)

### Version 5.1.0 (2015-10-15)

1. Fixed incorrect rotational motion of BallJoint and FreeJoint
    * [Pull request #518](https://github.com/dartsim/dart/pull/518)

1. Removed old documents: dart-tutorial, programmingGuide
    * [Pull request #515](https://github.com/dartsim/dart/pull/515)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #513](https://github.com/dartsim/dart/pull/513)

1. Fixed segfault in Linkage::Criteria
    * [Pull request #491](https://github.com/dartsim/dart/pull/491)
    * [Issue #489](https://github.com/dartsim/dart/issues/489)

1. Improved sdf/urdf parser
    * [Pull request #497](https://github.com/dartsim/dart/pull/497)
    * [Pull request #485](https://github.com/dartsim/dart/pull/485)

1. Fixed CMake warnings
    * [Pull request #483](https://github.com/dartsim/dart/pull/483)

1. Fixed build issues on Windows
    * [Pull request #516](https://github.com/dartsim/dart/pull/516)
    * [Pull request #509](https://github.com/dartsim/dart/pull/509)
    * [Pull request #486](https://github.com/dartsim/dart/pull/486)
    * [Pull request #482](https://github.com/dartsim/dart/pull/482)
    * [Issue #487](https://github.com/dartsim/dart/issues/487)

1. Fixed IpoptSolver bugs
    * [Pull request #481](https://github.com/dartsim/dart/pull/481)

1. Added Frame::getTransform(withRespecTo, inCoordinatesOf)
    * [Pull request #475](https://github.com/dartsim/dart/pull/475)
    * [Issue #471](https://github.com/dartsim/dart/issues/471)

1. Improved API documentation -- set the SHOW_USED_FILES tag to NO
    * [Pull request #474](https://github.com/dartsim/dart/pull/474)

1. Added convenience setters for generalized coordinates of FreeJoint
    * [Pull request #470](https://github.com/dartsim/dart/pull/470)
    * [Pull request #507](https://github.com/dartsim/dart/pull/507)

1. Fixed compilation warnings
    * [Pull request #480](https://github.com/dartsim/dart/pull/480)
    * [Pull request #469](https://github.com/dartsim/dart/pull/469)
    * [Issue #418](https://github.com/dartsim/dart/issues/418)

1. Added a mutex to Skeleton
    * [Pull request #466](https://github.com/dartsim/dart/pull/466)

1. Added generic URIs support
    * [Pull request #464](https://github.com/dartsim/dart/pull/464)
    * [Pull request #517](https://github.com/dartsim/dart/pull/517)

1. Added End Effector, Inverse Kinematics, and osgDart
    * [Pull request #461](https://github.com/dartsim/dart/pull/461)
    * [Pull request #495](https://github.com/dartsim/dart/pull/495)
    * [Pull request #502](https://github.com/dartsim/dart/pull/502)
    * [Pull request #506](https://github.com/dartsim/dart/pull/506)
    * [Pull request #514](https://github.com/dartsim/dart/pull/514)
    * [Issue #381](https://github.com/dartsim/dart/issues/381)
    * [Issue #454](https://github.com/dartsim/dart/issues/454)
    * [Issue #478](https://github.com/dartsim/dart/issues/478)

1. Removed outdated packaging scripts
    * [Pull request #456](https://github.com/dartsim/dart/pull/456)

1. Added initial position and initial velocity properties
    * [Pull request #449](https://github.com/dartsim/dart/pull/449)

1. Added a package.xml file for REP-136 support
    * [Pull request #446](https://github.com/dartsim/dart/pull/446)

1. Improved Linkage and Chain Criteria
    * [Pull request #443](https://github.com/dartsim/dart/pull/443)
    * [Issue #437](https://github.com/dartsim/dart/issues/437)

1. Added Joint::isCyclic to mark SO(2) topology
    * [Pull request #441](https://github.com/dartsim/dart/pull/441)

1. Fixed SEGFAULTs in DartLoader
    * [Pull request #439](https://github.com/dartsim/dart/pull/439)

1. Added the SYSTEM flag to include_directories
    * [Pull request #435](https://github.com/dartsim/dart/pull/435)

1. Improved Joint warning
    * [Pull request #430](https://github.com/dartsim/dart/pull/430)

1. Added tutorials (http://dart.readthedocs.org/)
    * [Pull request #504](https://github.com/dartsim/dart/pull/504)
    * [Pull request #484](https://github.com/dartsim/dart/pull/484)
    * [Pull request #423](https://github.com/dartsim/dart/pull/423)
    * [Pull request #511](https://github.com/dartsim/dart/pull/511)

### Version 5.0.2 (2015-09-28)

1. Fixed bug in Jacobian update notifications
    * [Pull request #500](https://github.com/dartsim/dart/pull/500)
    * [Issue #499](https://github.com/dartsim/dart/issues/499)

### Version 5.0.1 (2015-07-28)

1. Improved app indexing for bipedStand and atlasSimbicon
    * [Pull request #417](https://github.com/dartsim/dart/pull/417)

1. Added clipping command when it exceeds the limits
    * [Pull request #419](https://github.com/dartsim/dart/pull/419)

1. Improved CollisionNode's index validity check
    * [Pull request #421](https://github.com/dartsim/dart/pull/421)

1. Standardized warning messages for Joints
    * [Pull request #425](https://github.com/dartsim/dart/pull/425)
    * [Pull request #429](https://github.com/dartsim/dart/pull/429)

1. Fixed bug in SDF parser -- correct child for a joint
    * [Pull request #431](https://github.com/dartsim/dart/pull/431)

1. Fixed SDF parsing for single link model without joint
    * [Pull request #444](https://github.com/dartsim/dart/pull/444)

1. Added missing virtual destructors to Properties in Entity and [Soft]BodyNode
    * [Pull request #458](https://github.com/dartsim/dart/pull/458)

1. Limited maximum required version of Assimp less than 3.0~dfsg-4
    * [Pull request #459](https://github.com/dartsim/dart/pull/459)

1. Fixed SEGFAULTs in DartLoader
    * [Pull request #472](https://github.com/dartsim/dart/pull/472)

### Version 5.0.0 (2015-06-15)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #414](https://github.com/dartsim/dart/pull/414)

1. Added some missing API for DegreeOfFreedom
    * [Pull request #408](https://github.com/dartsim/dart/pull/408)

1. Replaced logMaps with Eigen::AngleAxisd
    * [Pull request #407](https://github.com/dartsim/dart/pull/407)

1. Improved FCL collision detector
    * [Pull request #405](https://github.com/dartsim/dart/pull/405)

1. Removed deprecated API and suppressed warnings
    * [Pull request #404](https://github.com/dartsim/dart/pull/404)

1. Added use of OpenGL's multisample anti-aliasing
    * [Pull request #402](https://github.com/dartsim/dart/pull/402)

1. Added computation of differences of generalized coordinates
    * [Pull request #389](https://github.com/dartsim/dart/pull/389)
    * [Issue #290](https://github.com/dartsim/dart/issues/290)

1. Added deprecated and force-linline definitions for clang
    * [Pull request #384](https://github.com/dartsim/dart/pull/384)
    * [Issue #379](https://github.com/dartsim/dart/issues/379)

1. Eradicated memory leaks and maked classes copy-safe and clonable
    * [Pull request #369](https://github.com/dartsim/dart/pull/369)
    * [Pull request #390](https://github.com/dartsim/dart/pull/390)
    * [Pull request #391](https://github.com/dartsim/dart/pull/391)
    * [Pull request #392](https://github.com/dartsim/dart/pull/392)
    * [Pull request #397](https://github.com/dartsim/dart/pull/397)
    * [Pull request #415](https://github.com/dartsim/dart/pull/415)
    * [Issue #280](https://github.com/dartsim/dart/issues/280)
    * [Issue #339](https://github.com/dartsim/dart/issues/339)
    * [Issue #370](https://github.com/dartsim/dart/issues/370)
    * [Issue #383](https://github.com/dartsim/dart/issues/383)

1. Improved PlaneShape constructors
    * [Pull request #366](https://github.com/dartsim/dart/pull/366)
    * [Pull request #377](https://github.com/dartsim/dart/pull/377)
    * [Issue #373](https://github.com/dartsim/dart/issues/373)

1. Added appveyor options for parallel build and detailed log
    * [Pull request #365](https://github.com/dartsim/dart/pull/365)

1. Improved robustness and package handling for URDF parsing
    * [Pull request #364](https://github.com/dartsim/dart/pull/364)

1. Fixed bug in BodyNode::_updateBodyJacobianSpatialDeriv()
    * [Pull request #363](https://github.com/dartsim/dart/pull/363)

1. Added alpha channel and Color functions
    * [Pull request #359](https://github.com/dartsim/dart/pull/359)
    * [Issue #358](https://github.com/dartsim/dart/issues/358)

1. Added Jacobian getters to Skeleton
    * [Pull request #357](https://github.com/dartsim/dart/pull/357)

1. Added ArrowShape for visualizing arrows
    * [Pull request #356](https://github.com/dartsim/dart/pull/356)

1. Fixed matrix dimension bug in operationalSpaceControl app
    * [Pull request #354](https://github.com/dartsim/dart/pull/354)

1. Added build type definitions
    * [Pull request #353](https://github.com/dartsim/dart/pull/353)

1. Added Signal class
    * [Pull request #350](https://github.com/dartsim/dart/pull/350)

1. Added LineSegmentShape for visualizing line segments
    * [Pull request #349](https://github.com/dartsim/dart/pull/349)
    * [Issue #346](https://github.com/dartsim/dart/issues/346)

1. Fixed segfault in SoftSdfParser
    * [Pull request #345](https://github.com/dartsim/dart/pull/345)

1. Added subscriptions for destructions and notifications
    * [Pull request #343](https://github.com/dartsim/dart/pull/343)

1. Added NloptSolver::[get/set]NumMaxEvaluations()
    * [Pull request #342](https://github.com/dartsim/dart/pull/342)

1. Added support of Eigen::VectorXd in parser
    * [Pull request #341](https://github.com/dartsim/dart/pull/341)

1. Added Skeleton::getNumJoints()
    * [Pull request #335](https://github.com/dartsim/dart/pull/335)

1. Fixed bug in DARTCollide for sphere-sphere collision
    * [Pull request #332](https://github.com/dartsim/dart/pull/332)

1. Fixed naming issues for Skeletons in World
    * [Pull request #331](https://github.com/dartsim/dart/pull/331)
    * [Issue #330](https://github.com/dartsim/dart/issues/330)

1. Added PlanarJoint support for URDF loader
    * [Pull request #326](https://github.com/dartsim/dart/pull/326)

1. Fixed rotation of the inertia reference frame for URDF loader
    * [Pull request #326](https://github.com/dartsim/dart/pull/326)
    * [Issue #47](https://github.com/dartsim/dart/issues/47)

1. Fixed bug in loading WorldFile
    * [Pull request #325](https://github.com/dartsim/dart/pull/325)

1. Added plotting of 2D trajectories
    * [Pull request #324](https://github.com/dartsim/dart/pull/324)

1. Removed unsupported axis orders of EulerJoint
    * [Pull request #323](https://github.com/dartsim/dart/pull/323)
    * [Issue #321](https://github.com/dartsim/dart/issues/321)

1. Added convenience functions to help with setting joint positions
    * [Pull request #322](https://github.com/dartsim/dart/pull/322)
    * [Pull request #338](https://github.com/dartsim/dart/pull/338)

1. Added Frame class and auto-updating for forward kinematics
    * [Pull request #319](https://github.com/dartsim/dart/pull/319)
    * [Pull request #344](https://github.com/dartsim/dart/pull/344)
    * [Pull request #367](https://github.com/dartsim/dart/pull/367)
    * [Pull request #380](https://github.com/dartsim/dart/pull/380)
    * [Issue #289](https://github.com/dartsim/dart/issues/289)
    * [Issue #294](https://github.com/dartsim/dart/issues/294)
    * [Issue #305](https://github.com/dartsim/dart/issues/305)

1. Added Travis-CI build test for OSX
    * [Pull request #313](https://github.com/dartsim/dart/pull/313)
    * [Issue #258](https://github.com/dartsim/dart/issues/258)

1. Added specification of minimum dependency version
    * [Pull request #306](https://github.com/dartsim/dart/pull/306)

### Version 4.3.6 (2016-04-16)

1. Fixed duplicate entries in Skeleton::mBodyNodes causing segfault in destructor
    * [Issue #671](https://github.com/dartsim/dart/issues/671)
    * [Pull request #672](https://github.com/dartsim/dart/pull/672)

### Version 4.3.5 (2016-01-09)

1. Fixed incorrect applying of joint constraint impulses (backported from 6.0.0)
    * [Pull request #578](https://github.com/dartsim/dart/pull/578)

### Version 4.3.4 (2015-01-24)

1. Fixed build issue with gtest on Mac
    * [Pull request #315](https://github.com/dartsim/dart/pull/315)

### Version 4.3.3 (2015-01-23)

1. Fixed joint Coulomb friction
    * [Pull request #311](https://github.com/dartsim/dart/pull/311)

### Version 4.3.2 (2015-01-22)

1. Fixed installation -- missing headers (utils/urdf, utils/sdf)

### Version 4.3.1 (2015-01-21)

1. Fixed API incompatibility introduced by dart-4.3.0
    * [Issue #303](https://github.com/dartsim/dart/issues/303)
    * [Pull request #309](https://github.com/dartsim/dart/pull/309)

### Version 4.3.0 (2015-01-19)

1. Added name manager for efficient name look-up and unique naming
    * [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added all-inclusive header and namespace headers
    * [Pull request #278](https://github.com/dartsim/dart/pull/278)
1. Added DegreeOfFreedom class for getting/setting data of individual generalized coordinates 
    * [Pull request #288](https://github.com/dartsim/dart/pull/288)
1. Added hybrid dynamics
    * [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added joint actuator types
    * [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added Coulomb joint friction
    * [Pull request #301](https://github.com/dartsim/dart/pull/301)
1. Migrated to C++11
    * [Pull request #268](https://github.com/dartsim/dart/pull/268)
    * [Pull request #299](https://github.com/dartsim/dart/pull/299)
1. Improved readability of CMake output messages
    * [Pull request #272](https://github.com/dartsim/dart/pull/272)
1. Fixed const-correctneess of member functions
    * [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added handling use of 'package:/' in URDF
    * [Pull request #273](https://github.com/dartsim/dart/pull/273)
    * [Issue #271](https://github.com/dartsim/dart/issues/271)

### Version 4.2.1 (2015-01-07)

1. Fixed version numbering of shared libraries in debian packages
    * [Pull request #286](https://github.com/dartsim/dart/pull/286)
1. Fixed Jacobian and its derivatives of FreeJoint/BallJoint
    * [Pull request #284](https://github.com/dartsim/dart/pull/284)

### Version 4.2.0 (2014-11-22)

1. Added reset functions for Simulation and Recording class
    * [Pull request #231](https://github.com/dartsim/dart/pull/231)
1. Added operational space control example
    * [Pull request #257](https://github.com/dartsim/dart/pull/257)
1. Fixed misuse of Bullet collision shapes
    * [Pull request #228](https://github.com/dartsim/dart/pull/228)
1. Fixed adjacent body pair check for Bullet collision detector
    * [Pull request #246](https://github.com/dartsim/dart/pull/246)
1. Fixed incorrect computation of constraint impulse for BallJointConstraint and WeldJointContraint
    * [Pull request #247](https://github.com/dartsim/dart/pull/247)
1. Improved generation of soft box shape for soft body
    * [Commit ec31f44](https://github.com/dartsim/dart/commit/ec31f44)

### Version 4.1.1 (2014-07-17)

1. Added ABI check script
    * [Pull request #226](https://github.com/dartsim/dart/pull/226)
    * [Pull request #227](https://github.com/dartsim/dart/pull/227)
1. Fixed build issues on Linux
    * [Pull request #214](https://github.com/dartsim/dart/pull/214)
    * [Pull request #219](https://github.com/dartsim/dart/pull/219)
1. Fixed build issues on Windows
    * [Pull request #215](https://github.com/dartsim/dart/pull/215)
    * [Pull request #217](https://github.com/dartsim/dart/pull/217)
1. Fixed unintended warning messages
    * [Pull request #220](https://github.com/dartsim/dart/pull/220)

### Version 4.1.0 (2014-07-02)

1. Fixed bug in switching collision detectors
    * [Issue #127](https://github.com/dartsim/dart/issues/127)
    * [Pull request #195](https://github.com/dartsim/dart/pull/195)
1. Fixed kinematics and dynamics when a skeleton has multiple parent-less bodies
    * [Pull request #196](https://github.com/dartsim/dart/pull/196)
1. Fixed issue on installing DART 4 alongside DART 3 on Linux
    * [Issue #122](https://github.com/dartsim/dart/issues/122)
    * [Pull request #203](https://github.com/dartsim/dart/pull/203)
1. Fixed warnings on gcc
    * [Pull request #206](https://github.com/dartsim/dart/pull/206)
1. Renamed getDof() to getNumDofs()
    * [Pull request #209](https://github.com/dartsim/dart/pull/209)
1. Added cylinder shape for soft body
    * [Pull request #210](https://github.com/dartsim/dart/pull/210)

### Version 4.0.0 (2014-06-02)

1. Added implicit joint spring force and damping force
1. Added planar joint
1. Added soft body dynamics
1. Added computation of velocity and acceleration of COM
1. Added bullet collision detector
  * [Pull request #156](https://github.com/dartsim/dart/pull/156)
1. Improved performance of forward dynamics algorithm
  * [Pull request #188](https://github.com/dartsim/dart/pull/188)
1. Improved dynamics API for Skeleton and Joint
  * [Pull request #161](https://github.com/dartsim/dart/pull/161)
  * [Pull request #192](https://github.com/dartsim/dart/pull/192)
  * [Pull request #193](https://github.com/dartsim/dart/pull/193)
1. Improved constraint dynamics solver
  * [Pull request #184](https://github.com/dartsim/dart/pull/184)
1. Improved calculation of equations of motion using Featherstone algorithm
  * [Issue #85](https://github.com/dartsim/dart/issues/87)
1. Improved optimizer interface and added nlopt solver
  * [Pull request #152](https://github.com/dartsim/dart/pull/152)
1. Fixed self collision bug
  * [Issue #125](https://github.com/dartsim/dart/issues/125)
1. Fixed incorrect integration of BallJoint and FreeJoint
  * [Issue #122](https://github.com/dartsim/dart/issues/122)
  * [Pull request #168](https://github.com/dartsim/dart/pull/168)

### Version 3.0 (2013-11-04)

1. Removed Transformation classes. Their functionality is now included in joint classes.
1. Added Featherstone algorithm. Can currently only be used without collision handling. The old algortihm is still present and used for that case.
1. Removed kinematics namespace. Functionality is moved to dynamics classes.
1. Added dart root namespace
1. A lot of function and variable renames
1. Added constraint namespace
1. Added "common" namespace

### Version 2.6 (2013-09-07)

1. Clean-up of build system:
  * Renamed DART_INCLUDEDIR to the standard-compliant DART_INCLUDE_DIRS in CMake files. Users need to adapt their CMake files for this change.
  * Users no longer need to call find_package(DARTExt) in the CMake files. A call to find_package(DART) also finds its dependencies now.
  * Allow user to overwrite installation prefix
  * Add possibility to include DART header files as '#include \<dart/dynamics/Skeleton.h\>' in addition to '#include \<dynamics/Skeleton.h\>'
  * Allow out-of-source builds
1. URDF loader:
  * Major clean-up
  * Consider mesh scaling factor

### Version 2.5 (2013-07-16)

1. Replaced robotics::World with simulation::World
1. Removed robotics::Robot
1. Added simulation::SimWindow
1. Some speed-up of Eigen calculations
1. Added abstract trajectory interface
1. ConstraintDynamics handles contact, joint limit and other constraint forces simultaneously
1. Improved Lemke algorithm for solving LCP
1. Renamed skeletonDynamics::getQDotVector() to getPoseVelocity()
1. Added abstract CollisionDetector interface allowing for multiple different collision detector implementations.
1. Created math namespace
1. Added System class as base class to Skeleton and Joint
1. URDF loader: Removed ability to load nonstandard URDF files with an object tag
1. Added support for multiple shapes per BodyNode
1. Made urdfdom a dependency instead of including it in the DART source
1. Added function to CollisionDetector to let user check a specific pair of BodyNodes for collision

### Version 2.4 (2013-03-05)

1. Mass and inertia are no longer stored in Shape but in BodyNode.
1. Different shapes for collision and visualization (not just different meshes)
1. Shapes are no longer centered at the COM but can be transformed independently relative to the link frame.
1. Improved URDF support
  * Support for non-mesh shapes
  * Does not create dummy root nodes anymore
  * Support for continuous joints
  * Support for arbitrary joint axes for revolute joints (but not for prismatic joints) instead of only axis-aligned joint axes
  * Support for relative mesh paths even if the robot and world URDF files are in different directories
  * All supported joint types can be root joints
1. Clean-up of the Robot class
1. Removed Object class
1. More robust build and installation process on Linux

