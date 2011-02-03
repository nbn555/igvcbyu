/*
 * JoystickCommand.cpp
 *
 *  Created on: Jan 19, 2011
 *      Author: tallred3
 */

#include "JoystickCommand.h"

#include <stdlib.h>
#include <iostream>

using namespace std;

#define DEBUG

JoystickCommand::JoystickCommand( int joystickIndex ): x(-1), y(-1), z(-1), joystickIndex(joystickIndex) {

	system("wminput -d &");

	//spin waiting for the joystick to get initialized


	while(!this->joystick.getJoysticksCount()){
#ifdef DEBUG
		cout << "Waiting for joystick connection Count:" << this->joystick.getJoysticksCount() << endl;
#endif
	}
	cout << "Joystick found" << endl;

	this->joystick.getJoystickPosition( this->joystickIndex, this->x, this->y, this->z, this->buttons );

	//once the joystick is initialized size the function pointers and their data pointers
	cout << this->buttons.size() << " " << endl;
	this->functPtrs = vector<void(*)(void*)>(this->buttons.size(), NULL);
	this->functPtrsData = vector<void*>(this->buttons.size(), NULL);

}

JoystickCommand::~JoystickCommand() { }

void JoystickCommand::doProcess() {
		this->joystick.getJoystickPosition(this->joystickIndex, this->x, this->y, this->z, this->buttons );

		int m1 = this->x * 1000;
		int m2 = this->z * 1000;

//		MotorController::instance()->setSpeed(MotorController::Channel1, m1);
//		MotorController::instance()->setSpeed(MotorController::Channel2, m2);

		for( unsigned int i = 0; i < this->buttons.size(); i++ ) {
			if(this->buttons[i]) {
				if( NULL != this->functPtrs[i])
					this->functPtrs[i](this->functPtrsData[i]);
			}
		}

#ifdef DEBUG
		cout << x << " " << y << " " << z << " ";
		for(unsigned int i = 0; i < this->buttons.size(); i++ )
			cout << this->buttons[i];
		cout << endl;
#endif

}

void JoystickCommand::registerButton( unsigned int buttonIndex, void (*funct)(void*), void* data ) {

	if(buttonIndex >= 0 && buttonIndex < this->functPtrs.size() ) {
		this->functPtrs[buttonIndex] = funct;
		this->functPtrsData[buttonIndex] = data;
	}
}

