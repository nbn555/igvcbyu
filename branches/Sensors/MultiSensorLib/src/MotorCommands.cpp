/*
 * MotorCommands.cpp
 *
 *  Created on: Jan 11, 2011
 *      Author: igvcbyu
 */

#include "MotorCommands.h"
#include "MotorController.h"

MotorCommands::MotorCommands() {
	linearConst=1;
	angConst=.5;
}

bool Go(float linear, float angular){
	int rmp1= linearConst*linear + angConst*angular;
	int rmp2= linearConst*linear - angConst*angular;
	if(!(motor.setSpeed(1, rmp1)&&motor.setSpeed(2, rmp2)))
		return false;
	motor.doProcess();
	return true;
}

MotorCommands::~MotorCommands() {
	// TODO Auto-generated destructor stub
}
