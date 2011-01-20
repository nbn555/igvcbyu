/*
 * JoystickCommand.cpp
 *
 *  Created on: Jan 19, 2011
 *      Author: tallred3
 */

#include "JoystickCommand.h"

JoystickCommand::JoystickCommand( MotorController * mP ): MotorCommandInterface( mP ) {

}

JoystickCommand::~JoystickCommand() { }

void JoystickCommand::doProcess() {
		this->joystick.getJoystickPosition(0, this->x, this->y, this->z, this->buttons );

		int m1 = this->x * 1000;
		int m2 = this->z * 1000;

		this->motorPtr->setSpeed(MotorController::Channel1, m1);
		this->motorPtr->setSpeed(MotorController::Channel2, m2);

}
