/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>

#include <mrpt/utils/CConfigFile.h>

#include "Compass.h"

using namespace mrpt::utils;
using namespace std;

int main( int argc, char** argv ) {

	Compass comp;
	CConfigFile config( "Compass.ini" );

	comp.loadConfig(config, "COMPASS" );
	comp.initialize();

	//cout << Compass::computeChecksum("#000*fd\n\r") << endl;

	while(1) {
		comp.doProcess();
		//TODO make the compass class support the TObservation type.  IE we need to extend CObservation with a ObservationCompass.
		if( comp.isYawValid() ) cout << "Yaw: " << comp.getYaw() << endl;
		if( comp.isPitchValid() ) cout << "Pitch: " << comp.getPitch() << endl;
		if( comp.isRollValid() ) cout << "Roll: " << comp.getRoll() << endl;
		cout << endl;
	}

	return 0;
}