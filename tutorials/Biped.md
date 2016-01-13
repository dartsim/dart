Biped
-------------------
1. Controller
	__PUBLIC:__
    + Controller
    	- arbitararily define the stiffness and samping coefficients to 1000 and 50 except the first Dof of root. Then set the target position of PD controller
    		`int nDofs = mBiped->getNumDofs();`
            `mForces = Eigen::VectorXd::Zero(nDofs);`
            `mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);`
            `mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);`
            `for (size_t i = 0; i<6;++i)`
            `{mKp(i,i) = 0.0;`
            `mKd(i,i) = 0.0}`
            `for (size_t i =6; i<6;++i`
            `{mKp(i,i) = 1000;`
            `mKd(i,i) = 50};`
            `setTargetPositions(mBiped->getPositions()));`
    + setTargetPositions
    	_Reset the desired dof position position to the current position_
    + clearForces
    	`mForces.setZero();`
    + addPDForces
    	- __lesson 2__
    	compute the forces by PD controller and add them to the internal forces of biped via _setForces_
        `Eigen::VectorXd q = mBiped->getPositions();`
        `Eigen::VectorXd dq = mBiped->getVelocities();`
        `Eigen::VectorXd p = -mKp*(q - mTargetPosition)`
        `Eigen::VectorXd d = -mKd* dq`
        `mForces += p+d`
        __? overriding or cumulative addition__
    + addSPDForces
    	- __lesson 3__
    	> the basic idea of SPD is to compute control force using the predicted state at the next time step, instead of the current state. The implementation of SPD involves accessing the current dynamic quantities in Lagrange's equations of motion.
    	
    	`Eigen::VectorXd q = mBiped->getPositions()`
        `Eigen::VectorXd dq = mBiped->getVelocities()`
        `Eigen::MatrixXd invM = (mBiped0>getMassMatrix() + mKd * mBiped->getTimeStep()).inverse()`
        `Eigen::VectorXd p = -mKp * (q - mTargetPositions + dq*mBiped->getTimeStep())`
        `Eigen::VectorXd qd = -mKd * dq`
        `Eigen::VectorXd qddot = invM*(-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getContraintForces())`
        `mForces += p + d -mKd * qddot *mBiped->getTimeStep()`
        __?accumulative addition rather than overriding__
    + addAnkleStrategyForces
    	- __lesson 4__
    	> Ankle (or hip) strategy is an effective way to maintain standing balance.  The idea is to adjust the target position of ankles according to the deviation between the center of mass and the center of pressure projected on the ground.
    	> __ θa = -kp (x - p) - kd (dx/dt - dp/dt)__
    	
        compute the deviation between the center of mass and an approximated center of pressure __? in the anterior-posterior axis__
    `Eigen::Vector3d COM = mBiped->getCOM();`
    `Eigen::Vector3d offset(0.05, 0, 0);`
    `Eigen::Vector3d COP = mBiped->getBodyNode("h_heel_left")->getTransform() * offset;`
    `double diff = COM[0] - COP[0];`
    `Eigen::Vector3d dCOM = mBiped->getCOMLinearVelocity();`
    `Eigen::Vector3d dCOP =  mBiped->getBodyNode("h_heel_left")->getLinearVelocity(offset);`
    `double dDiff = dCOM[0] - dCOP[0];`
    __?The remaining of the ankle strategy implementation is just the matter of parameters tuning__
    Different feedback rule for falling forward and backward
    + setWheelCommands
    	- __lesson 6__
    + changeWheelSpeed

  __PROTECTED:__
	+ arguments: _mBiped, mForces, mKp, mKd, mTargetPosition (Target positions for the PD ocntroller), mPreOffset, mSpeed_
2. MyWindow
	__PUBLIC:__
    + Mywindow
    	- setWorld
    	- setController
    		`mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("biped")));`
    + keyboard
    + timeStepping
    	- clearForces
    	- addSPDForces
    	- addAnkleStrategyForces
    	- setWheelCommands
    	- apply body forces based on user input and color the body shape red
    		`if (mForceCountDown > 0)`
            `{`
            `BodyNode* bn = mWorld->getSkeleton("biped")->getBodyNode("h_abdomen");`
            `const ShapePtr& shape = bn->getVisualizationShape(0)`
            `shape->setColor(dart::Color::Red());`
            `if (mPositiveSign)`
            `bn->addExtForce(default_force * Eigen::Vector3d::UnitX(), bn->getCOM(), false, false);`
            `else`
            `bn->addExtForce(-default_force * Eigen::Vector3d::UnitX(), bn->getCOM(), false, false);`
            `--mForceCountDown`
            `}`
		- self-iteration
    
  __PROTECTED:__
    + arugments: _mController, mForceCountdown (number of iterations before clearing a force entry),  mPositiveSign_
3. loadBiped
	+ __lesson 1__
	+ load from file
		`WorldPtr world = skelParse::readWorld(DART_DATA_PATH"skel/biped.skel");`
        `SkeletonPtr biped = world->getSkeleton("biped");`
    + activate joint limits on the biped --> there are default joint limits in the XML file, like j_shin_right is in [-pi, 0]
    	`for (size_t i = 0; i<biped->getNumJoints(); ++i`
        `{biped->getJoint(i)->setPositionLimitEnforced(true);}`
        `biped->enableSelfCollision()`
4. setInitialPose
	+ __lesson 2__
	> To actively control its own motion, the biped must exert internal forces using actuators. Here we will use PD controller to produce internal forces to make the biped hold a target pose
	
    set initial position of some degree of freedom
    	`biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(),0.15`
        _Here the degree of freedom named "j_thigh_left_z" is set to 0.15 radian._
    __?But this initial pose decide the pose in simulation?__ no change  if comment this function
5. modifyBipedWithSkateboard
	+ __lesson 5__
	+ load skateboard skeleton from file
    + settle joint properties. have a slight offset to put skateboard under the foot
    + merge the skeleton
6. setVelocityAccuators
	+ __lesson 6__
	+ set actuator type of each wheel from FORCE to VELOCITY
	+ set the stiffness and damping coefficients for the wheels to zero.
	+ `int index1 = mBiped->getDof("joint_front_left_2")->getIndexInSkeleton();`
	+ `mBiped->setCommand(index1, mSpeed);`
7. solveIK
	+ __lesson 7__
8. createFloor
	+ create a skeleton called "floor"
	+ give the floor a body
		`BodyNodePtr body = floor->createJointAndBodyNodePair<weldJoint>(nullptr)`
    + set shape as box, color as black
    + addVisualization and addCollisionShape
    + put the body into position
    	`Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());`
        `tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);`
        `body->getParentJoint()->setTransformFromParentBodyNode(tf);`
9. int main
	+ create floor
	+ load biped
	+ modifyVolocityAccuators
	+ setVelocityAcuators
	+ solve inverse kinematics
		- set Positions accordding to balanced pose
		- set world gravity
	+ macro define `bullet collision`
	+ add floor to world
	+ add biped to world