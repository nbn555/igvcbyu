/*
 * GpsDevice.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#include <iostream>

#include "GpsDevice.h"

using namespace std;
using namespace mrpt::hwdrivers;

GpsDevice::GpsDevice(const std::string &fileName, const int Buffer_Length):CGPSInterface(Buffer_Length),configFile(fileName) { //fixes potential default constructor problem
	this->loadConfig_sensorSpecific(this->configFile, "iniGPS");
	this->initialize();
	cout << this->getSerialPortName() << endl;
	cout << this->getProcessRate() << endl;
	cout << this->getSensorLabel() << endl;
	cout << this->getState() << endl;


}

GpsDevice::~GpsDevice() {
	// TODO Auto-generated destructor stub
}
