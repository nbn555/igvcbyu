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
	MotorCommandInterface( MotorController * motorPtr );

	bool setVelocity( double linearVelocity, double angularVelocity );
	virtual void doProcess() = 0;

	virtual ~MotorCommandInterface();
protected:
	MotorController * motorPtr;
	double linearVelocity;
	double angularVelocity;
};

#endif /* MOTORCOMMANDINTERFACE_H_ */
