/*
 * GpsDevice.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#include <iostream>

#include "GPS.h"

using namespace std;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;

GPS::GPS(const int Buffer_Length):CGPSInterface(Buffer_Length), isGpggaUsed(false), isGprmcUsed(false) { //fixes potential default constructor problem
}

GPS::~GPS() {

}

//This function ought to be called after loadConfig but before initialize
void GPS::initConfig( mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	this->vendor = config.read_string(sectionName, "GPS_TYPE", "Novatel" );
	this->isGpggaUsed = config.read_bool(sectionName,"use_gga", true);
	this->isGprmcUsed = config.read_bool(sectionName,"use_rmc", false);
	this->baudRate = config.read_int(sectionName,"baudRate", 57600);
	this->portName = config.read_string(sectionName,"COM_port_LIN","/dev/ttyS0");
	this->processRate = config.read_int(sectionName,"process_rate", 1);

}

void GPS::initialize() {

	cout << "here" << endl;
	CSerialPort myCom(this->portName);
	myCom.setConfig(this->baudRate,0,8,1,false);
	cout << "Post open" << endl;

	stringstream inputStream;
	if( this->isGpggaUsed ) {
		cout << "Using gpgga" << endl;
		if( "PocketMAX" == this->vendor )
			inputStream << "$jasc,gpgga," << this->processRate << "\n\r";
		else if( "Novatel" == this->vendor )
			inputStream << "log gpgga ontime " << this->processRate << "\n\r";
		else
			cerr << "Unsupported vendor" << endl;
		myCom.Write(inputStream.str().c_str(),inputStream.str().length());//this will tell the gps to get a satellite reading once a second
	}

	if( this->isGprmcUsed ) {
		cout << "Using gprmc" << endl;
		if( "PocketMAX" == this->vendor) {
			inputStream << "$jasc,gprmc," << this->processRate << "\n\r";
		}
		else if( "Novatel" == this->vendor ) {
			inputStream << "log gprmc ontime " << this->processRate << "\n\r";
		}
		else
			cerr << "Unsupported vendor" << endl;
		myCom.Write(inputStream.str().c_str(),inputStream.str().length());//this will tell the gps to run the nmea rmc command once a second
	}

	myCom.close();

}
