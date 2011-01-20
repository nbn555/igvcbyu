/*
 * MotorCommands.cpp
 *
 *  Created on: Jan 11, 2011
 *      Author: igvcbyu
 */

#include "MotorCommands.h"

MotorCommands::MotorCommands( MotorController * mP ) : MotorCommandInterface(mP) {
	linearConst=1;
	angConst=.5;
}

bool MotorCommands::Go(float linear, float angular){
	int rmp1= linearConst*linear + angConst*angular;
	int rmp2= linearConst*linear - angConst*angular;
	if(!(motor.setSpeed(MotorController::Channel1, rmp1)&&motor.setSpeed(MotorController::Channel2, rmp2)))
		return false;
	motor.doProcess();
	return true;
}

MotorCommands::~MotorCommands() {
	// TODO Auto-generated destructor stub
}
