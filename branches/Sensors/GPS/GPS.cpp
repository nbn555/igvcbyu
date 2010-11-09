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

void GPS::loadConfig_sensorSpecific(CConfigFileBase & config, const string & sectionName) {

	string value = config.read_string(sectionName, "COM_port_LIN", "/dev/ttyS0" );
	this->m_COM.open(value);

	int baudRate = config.read_int(sectionName, "baudRate", 9600 );

	this->m_COM.setConfig( baudRate, 0, 8, 1);

	this->isGpggaUsed = config.read_bool(sectionName,"use_gga", true);
	this->isGprmcUsed = config.read_bool(sectionName,"use_rmc", false);

}

void GPS::initialize() {
	string writeValue;
	if( this->isGpggaUsed ) {
		writeValue = "$jasc,gpgga,1\n\r";
		this->m_COM.Write(writeValue.c_str(),writeValue.length());//this will tell the old gps to get a satellite reading once a second
	}

	if( this->isGprmcUsed ) {
		writeValue = "$jasc,gprmc,1\n\r";
		this->m_COM.Write(writeValue.c_str(),writeValue.length());//this will tell the old gps to run the nmea rmc command once a second
	}
}
