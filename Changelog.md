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

