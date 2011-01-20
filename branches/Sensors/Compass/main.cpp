/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <stdlib.h>

#include <mrpt/utils/CConfigFile.h>

#include "Compass.h"
#include "MotorController.h"
#include "JoystickCommand.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

int main( int argc, char** argv ) {

	Compass comp;
	CConfigFile config( "Compass.ini" );

	comp.loadConfig(config, "COMPASS" );
	comp.initialize();

	while(1) {
		comp.doProcess();
		if( comp.isYawValid() ) cout << "Yaw: " << comp.getYaw() << endl;
		if( comp.isPitchValid() ) cout << "Pitch: " << comp.getPitch() << endl;
		if( comp.isRollValid() ) cout << "Roll: " << comp.getRoll() << endl;
		cout << endl;
	}

	return 0;
}
