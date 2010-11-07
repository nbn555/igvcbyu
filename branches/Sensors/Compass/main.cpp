/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>

#include "Compass.h"

using namespace std;

int main( int argc, char** argv ) {

	Compass comp( "Compass.ini" );

	while(1) {
		comp.doProcess();
		if( comp.isYawValid() ) cout << "Yaw: " << comp.getYaw() << endl;
		if( comp.isPitchValid() ) cout << "Pitch: " << comp.getPitch() << endl;
		if( comp.isRollValid() ) cout << "Roll: " << comp.getRoll() << endl;
		cout << endl;
	}

	return 0;
}
