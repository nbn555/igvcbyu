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
	JoystickCommand( MotorController * mP );
	virtual ~JoystickCommand();

	void doProcess();
private:
	float x, y, z;
	std::vector<bool> buttons;
	mrpt::hwdrivers::CJoystick joystick;
};

#endif /* JOYSTICKCOMMAND_H_ */
