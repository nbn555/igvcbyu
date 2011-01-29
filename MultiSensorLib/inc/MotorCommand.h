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
	MotorCommand( MotorController * mC, double lC=700, double aC=5 );
	virtual ~MotorCommand();

	void doProcess();

	bool Go(float linear, float angular);

private:
	double linearConst, angConst;
};

#endif /* MOTORCOMMANDS_H_ */
