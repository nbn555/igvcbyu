/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <mrpt/utils/CConfigFile.h>

//#include "SensorData.h"
#include "Compass.h"
#include "MotorController.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

/*double CompensateCompass(double deg) {
	double piToD = 3.14159265/180;
	const double OFFSET = 7.4;
	if (deg == 0) return deg;
	if (deg < 200)
		deg = deg + 34*sin(.93*piToD*deg - 0.15) - OFFSET;
	else
		deg = deg + 18 * sin(deg*piToD - .3);

	return deg;
}
*/
int main( int argc, char** argv ) {

	ofstream fout("Compass Accuracy Testing.txt"); // to take GPS readings for testing
	fout.precision(7);


	Compass comp;
	CConfigFile config( "Compass.ini" );

	comp.loadConfiguration(config, "COMPASS" );
	comp.init();

	while(1) {
		comp.sensorProcess();
		if( comp.isYawValid() ) cout << "Yaw: " << comp.getYaw() * 180 / 3.14159265 << endl;
		//if( comp.isYawValid() ) cout << "Correct Yaw: " << CompensateCompass(comp.getYaw() * 180 / 3.14159265) << endl;
		if( comp.isPitchValid() ) cout << "Pitch: " << comp.getPitch() << endl;
		if( comp.isRollValid() ) cout << "Roll: " << comp.getRoll() << endl;


		cout << endl;

		mrpt::system::sleep(200);
	}

	fout.close();

	return 0;
}
