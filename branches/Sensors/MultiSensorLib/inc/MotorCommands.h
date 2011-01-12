/*
 * MotorCommands.h
 *
 *  Created on: Jan 11, 2011
 *      Author: igvcbyu
 */

#ifndef MOTORCOMMANDS_H_
#define MOTORCOMMANDS_H_

#include "MotorController.h"

class MotorCommands {
public:
	MotorCommands();
	virtual ~MotorCommands();



	bool Go(float linear, float angular);

private:
	float linearConst, angConst;
	MotorController motor;
};

#endif /* MOTORCOMMANDS_H_ */
