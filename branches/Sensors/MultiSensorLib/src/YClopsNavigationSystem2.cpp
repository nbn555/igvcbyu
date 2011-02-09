/**
 *  \file: YClopsNavigationSystem2.cpp
 *  \date: Feb 6, 2011
 *  \author: tallred3
 */

#include "YClopsNavigationSystem2.h"
#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

bool isGpsDataShown;
bool isCompassDataShown;
bool isLidarDataShown;
YClopsNavigationSystem2::YClopsNavigationSystem2(CConfigFile & config)
: isGpsDataShown(false), isCompassDataShown(false), isLidarDataShown(false), isCameraDataShown(false)
{
	this->motor = new DualMotorCommand();
	this->compass = new Compass( config );
}

YClopsNavigationSystem2::~YClopsNavigationSystem2() {
	delete this->motor;
	delete this->compass;
}

void YClopsNavigationSystem2::doProcess() {
	cout << "." << endl;

	this->motor->doProcess();
	this->compass->doProcess();

	if( this->isCompassDataShown ) {
		this->compass->dumpData(cout);
	}

}

void YClopsNavigationSystem2::setAutonomusMode() {
	this->useYclopsMotorCommand();
}

void YClopsNavigationSystem2::setNavigationMode() {
	this->useYclopsMotorCommand();
}

void YClopsNavigationSystem2::setIdle() {

}

bool YClopsNavigationSystem2::toggleGpsDump() {
	this->isGpsDataShown = !this->isGpsDataShown;
	return this->isGpsDataShown;
}

bool YClopsNavigationSystem2::toggleCompassDump() {
	this->isCompassDataShown = !this->isCompassDataShown;
	return this->isCompassDataShown;
}

bool YClopsNavigationSystem2::toggleLidarDump() {
	this->isLidarDataShown = !this->isLidarDataShown;
	return this->isLidarDataShown;
}

bool YClopsNavigationSystem2::toggleCameraDump() {
	this->isCameraDataShown = !this->isCameraDataShown;
	return this->isCameraDataShown;
}

void YClopsNavigationSystem2::useYclopsMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new MotorCommand();
	delete tmp;
}

void YClopsNavigationSystem2::useWiiMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new DualMotorCommand();
	delete tmp;
}

void YClopsNavigationSystem2::useNullMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new DummyMotorCommand();
	delete tmp;
}
