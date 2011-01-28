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
	//this makes a navigation challenge go, by loading the ini files and using points.txt + startPoint as waypoints.
	NavigationChallenge nav = NavigationChallenge();
	string file = "points.txt";
	//true means it is a navigation challenge, needs some work for the false to work
	nav.AutonomousMode(file, true);
	return 0;
}
