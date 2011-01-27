/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include "NavigationChallenge.h"

using namespace std;

int main( int argc, char** argv ) {
	NavigationChallenge nav = NavigationChallenge();
	string file = "points.ini";
	nav.AutonomousMode(file, true);
	return 0;
}
