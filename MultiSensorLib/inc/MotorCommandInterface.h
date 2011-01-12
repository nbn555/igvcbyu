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
	MotorCommandInterface();
	virtual ~MotorCommandInterface();
protected:
	//MotorController motor;
};

#endif /* MOTORCOMMANDINTERFACE_H_ */
