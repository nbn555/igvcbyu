/*
 * main.cpp
 *
 *  Created on: Jan 12, 2011
 *      Author: tallred3
 */

#include <iostream>
#include <vector>

#include <mrpt/hwdrivers/CJoystick.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;

int main(int argc, char** argv ) {
	CJoystick joy;

	float x, y, z;
	vector<bool> buttons;

	while (1){
		joy.getJoystickPosition(0, x, y, z, buttons );

		cout << "X=" << x << " Y=" << y << " Z=" << z << "Button cnt " << buttons.size() << endl;
	}

	return 0;

}
