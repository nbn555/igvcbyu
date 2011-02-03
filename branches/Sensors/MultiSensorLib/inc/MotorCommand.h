/*
 * MotorCommands.h
 *
 *  Created on: Jan 11, 2011
 *      Author: igvcbyu
 */

#ifndef MOTORCOMMANDS_H_
#define MOTORCOMMANDS_H_

#include "MotorCommandInterface.h"
#include "MotorController.h"

class MotorCommand: public MotorCommandInterface {
public:
	MotorCommand( double lC=400, double aC=200 );
	virtual ~MotorCommand();

	void doProcess();

private:
	double linearConst, angConst;
};

#endif /* MOTORCOMMANDS_H_ */
