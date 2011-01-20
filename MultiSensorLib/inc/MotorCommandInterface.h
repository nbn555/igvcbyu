/**
 * @file: MotorCommandInterface.h
 * @date: Jan 12, 2011
 * @author: Thomas Eldon Allred
 * @brief: an interface for the a device that sends commands to the motor controller.
 */

#ifndef MOTORCOMMANDINTERFACE_H_
#define MOTORCOMMANDINTERFACE_H_

#include "MotorController.h"

class MotorCommandInterface {
public:
	/**
	 * MotorCommandInterface - abstract class to send commands to the motor controller
	 * @param motorPtr - pointer to the MotorController
	 */
	MotorCommandInterface( MotorController * motorPtr );

	/**
	 * setVelocity - sets the speed for the robot
	 * @param linearVelocity - the linear speed of the robot
	 * @param angularVelocity - the angular speed of the robot follows
	 * directions on the unit circle eg. left is positive right is negative
	 * @return true upon success
	 */
	bool setVelocity( double linearVelocity, double angularVelocity );

	/**
	 * doProcess - abstract function implements the functionality fo the command
	 */
	virtual void doProcess() = 0;

	virtual ~MotorCommandInterface();
protected:
	MotorController * motorPtr;	//!pointer to the motor controller
	double linearVelocity;		//!The most recent linearVelocity set
	double angularVelocity;		//!The most recent angularVelocity set
};

#endif /* MOTORCOMMANDINTERFACE_H_ */
