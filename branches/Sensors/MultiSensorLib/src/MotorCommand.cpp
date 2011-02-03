/*
 * MotorCommands.cpp
 *
 *  Created on: Jan 11, 2011
 *      Author: igvcbyu
 */

#include "MotorCommand.h"

MotorCommand::MotorCommand( double lC, double aC): linearConst(lC), angConst(aC) {
}

void MotorCommand::doProcess() {
	int rmp1= linearConst * this->linearVelocity + angConst*this->angularVelocity;
	int rmp2= linearConst * this->linearVelocity - angConst*this->angularVelocity;
	if(!(MotorController::instance()->setSpeed(MotorController::Channel1, rmp1)&&MotorController::instance()->setSpeed(MotorController::Channel2, rmp2)))
		this->success = false;
	MotorController::instance()->doProcess();
	this->success =  true;

}

MotorCommand::~MotorCommand() {
}
