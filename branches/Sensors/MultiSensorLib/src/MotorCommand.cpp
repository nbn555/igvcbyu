/*
 * MotorCommands.cpp
 *
 *  Created on: Jan 11, 2011
 *      Author: igvcbyu
 */

#include "MotorCommand.h"

MotorCommand::MotorCommand(MotorController * mC, double lC, double aC): MotorCommandInterface(mC), linearConst(lC), angConst(aC) {
}

void MotorCommand::doProcess() {
	this->Go(this->linearVelocity,this->angularVelocity);
}

bool MotorCommand::Go(float linear, float angular){
	int rmp1= linearConst*linear + angConst*angular;
	int rmp2= linearConst*linear - angConst*angular;
	if(!(this->motorPtr->setSpeed(MotorController::Channel1, rmp1)&&this->motorPtr->setSpeed(MotorController::Channel2, rmp2)))
		return false;
	motorPtr->doProcess();
	return true;
}

MotorCommand::~MotorCommand() {
}
