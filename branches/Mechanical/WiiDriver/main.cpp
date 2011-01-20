/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <stdlib.h>

#include <mrpt/utils/CConfigFile.h>

#include "MotorController.h"
#include "JoystickCommand.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

/*
 * Quick How to
 * make sure the /dev/uinput device exsists type
 * ls -lah /dev/uinput
 * if it doesn't exsist type
 * sudo modprobe uinput
 *
 * For ubuntu
 * this will only last until the next reboot
 * or add the /etc/modules file by adding the following
 * su -c 'echo "uinput">>/etc/modules'
 * reboot or run the modprobe command above
 * end ubuntu
 *
 * Make sure permissions for /dev/uinput are 666 if not run
 * sudo chmod 666 /dev/uinput
 * install wminput stuff with for ubuntu
 * sudo apt-get install wminput
 * for fedora
 * sudo yum install wminput
 * install joystick drivers for ubuntu
 * sudo apt-get install joystick
 * for fedora
 * sudo yum install joystick
 * check the default config file is the gamepad file by typing
 * ls -lah /etc/cwiid/wminput
 * you should see something like "default -> gamepad"
 * if not delete the default link and create a new one with the following
 * sudo rm default
 * sudo ln -s gamepad default
 *
 * make sure the config file has the following
 * #Classic.Dpad.X = ABS_X
 * #Classic.Dpad.Y = ABS_Y
 * Classic.LStick.X = ABS_X
 * Classic.LStick.Y = ABS_Y
 * Classic.RStick.X = ABS_HAT1X
 * Classic.RStick.Y = ABS_HAT1Y
 *
 * then run the wminput command
 * wminput
 * then press buttons 1 and 2 on the wiimote
 * once the bluetooth link has been established run where the astrisk is the index of the joystick (should be zero)
 * jscal -c /dev/input/js*
 * follow the prompts
 * run the wiiDriver program
 */
int main( int argc, char** argv ) {

	MotorController motor("/dev/ttyUSB0",true);
	JoystickCommand js(&motor);

	while(1) {

		js.doProcess();
		motor.doProcess();

	}

	return 0;
}
