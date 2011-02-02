/*
 * JoystickCommand.h
 *
 *  Created on: Jan 19, 2011
 *      Author: tallred3
 */

#ifndef JOYSTICKCOMMAND_H_
#define JOYSTICKCOMMAND_H_

#include <mrpt/hwdrivers/CJoystick.h>
#include <vector>
#include "MotorCommandInterface.h"

class JoystickCommand: public MotorCommandInterface {
public:
	JoystickCommand( MotorController * mP, int joystickIndex = 0 );
	virtual ~JoystickCommand();

	void doProcess();
	void registerButton( unsigned int buttonIndex, void (*funct)(void*), void* data );
private:
	float x, y, z;
	std::vector<bool> buttons;
	std::vector<void(*)(void*)> functPtrs;
	std::vector<void*> functPtrsData;
	mrpt::hwdrivers::CJoystick joystick;
	int joystickIndex;
};

#endif /* JOYSTICKCOMMAND_H_ */
