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
	MotorCommandInterface();

	/**
	 * @brief sets the speed for the robot
	 * @param[in] linearVelocity the linear speed of the robot
	 * @param[in] angularVelocity the angular speed of the robot follows
	 * directions on the unit circle eg. left is positive right is negative
	 * @return bool true upon success
	 */
	bool setVelocity( double linearVelocity, double angularVelocity );

	/**
	 * @brief abstract function implements the functionality of the command
	 */
	virtual void doProcess() = 0;

	/**
	 * @brief tells us if the command was successful
	 * @return bool true if command was successful
	 */
	bool getSuccess();

	virtual ~MotorCommandInterface();
protected:
	double linearVelocity;		//!<The most recent linearVelocity set
	double angularVelocity;		//!<The most recent angularVelocity set
	bool success;				//!<Whether the last command was successful
};

/**
 * @brief interfaces with the roboteq motor controller when it is in mixed mode
 * @todo change this class so the linear and angular gain constants are in the config file
 */
class MotorCommand: public MotorCommandInterface {
public:
	MotorCommand( double lC=100, double aC=60 );
	virtual ~MotorCommand();

	void doProcess();

private:
	double linearConst, angConst;
};

/**
 * @brief interfaces with the roboteq motor controller when it is in tank or separate mode
 */
class DualMotorCommand: public MotorCommandInterface {
public:
	DualMotorCommand();
	virtual ~DualMotorCommand();
	void doProcess();
};

/**
 * @brief Dummy class separates the roboteq motor controller from the software
 */
class DummyMotorCommand: public MotorCommandInterface {
public:
	DummyMotorCommand();
	virtual ~DummyMotorCommand();
	void doProcess();
};

#endif /* MOTORCOMMANDINTERFACE_H_ */
