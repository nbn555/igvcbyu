/*
 * GpsDevice.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <mrpt/slam/CObservationGPS.h>

#include "GPS.h"
#include "logging.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;

GPS::GPS(const int Buffer_Length):CGPSInterface(Buffer_Length), isGpggaUsed(false), isGprmcUsed(false) { //fixes potential default constructor problem
}

GPS::~GPS() {

}

//This function ought to be called after loadConfig but before initialize
void GPS::initConfig( mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	this->vendor = config.read_string(sectionName, "GPS_TYPE", "Novatel" );

	LOG(DEBUG4) << "Using " << this->vendor << " GPS" << endl;

	this->isGpggaUsed = config.read_bool(sectionName,"use_gga", true);

	LOG(DEBUG4) << "Using gpgga: " << (this->isGpggaUsed?"TRUE":"FALSE") << endl;

	this->isGprmcUsed = config.read_bool(sectionName,"use_rmc", false);

	LOG(DEBUG4) << "Using gprmc: " << (this->isGprmcUsed?"TRUE":"FALSE") << endl;

	this->baudRate = config.read_int(sectionName,"baudRate", 57600);

	LOG(DEBUG4) << "Using " << this->baudRate << " baud" << endl;

	this->portName = config.read_string(sectionName,"COM_port_LIN","/dev/ttyS0");

	LOG(DEBUG4) << "Connecting to port " << this->portName << endl;

	this->processRate = config.read_int(sectionName,"process_rate", 1);

	LOG(DEBUG4) << "Using a process rate of " << this->processRate << endl;

}

void GPS::initialize() {
	// check if connection is already established, if not, reconnect
	if (this->isGPS_connected()) {return; }
	LOG(DEBUG4) << "Connecting to GPS ... (./YClopsLib/GPS)" << endl;
	CSerialPort myCom(this->portName);
	myCom.setConfig(this->baudRate,0,8,1,false);
	LOG(DEBUG4) << "Opened connection to GPS in initialize function" << endl;

	stringstream inputStream;
	if( this->isGpggaUsed ) {
		if( "PocketMAX" == this->vendor )
			inputStream << "$jasc,gpgga," << this->processRate << "\n\r";
		else if( "Novatel" == this->vendor )
			inputStream << "log gpgga ontime " << this->processRate << "\n\r";
		else
			LOG(ERROR) << "Unsupported GPS vendor:" << this->vendor << endl;
		myCom.Write(inputStream.str().c_str(),inputStream.str().length());//this will tell the gps to get a satellite reading once a second
	}

	if( this->isGprmcUsed ) {
		if( "PocketMAX" == this->vendor) {
			inputStream << "$jasc,gprmc," << this->processRate << "\n\r";
		}
		else if( "Novatel" == this->vendor ) {
			inputStream << "log gprmc ontime " << this->processRate << "\n\r";
		}
		else
			LOG(ERROR) << "Unsupported GPS vendor:" << this->vendor << endl;
		myCom.Write(inputStream.str().c_str(),inputStream.str().length());//this will tell the gps to run the nmea rmc command once a second
	}

	myCom.close();

	LOG(DEBUG4) << "GPS initialized" << endl;
}

void GPS::dumpData(std::ostream & out ) {
	out << "************" << endl;
	out << "GPS" << endl;
	out << "Connection: " << this->isGPS_connected() << " " << "Signal: " << this->isGPS_signalAcquired();

	static bool isValid = false;
	static CObservationGPSPtr gpsData;
	CGenericSensor::TListObservations				lstObs;
	//prime the pump with the first gps observation

	this->getObservations(lstObs);
	out << " Size: " << lstObs.size() << endl;
	if(lstObs.size()) {
		isValid = true;
		gpsData = CObservationGPSPtr(lstObs.begin()->second);
		//this->appendObservation(gpsData);//This probably wont be used
	}
	if(isValid) {
		if( gpsData->has_GGA_datum ) {
			out << "GGA: " << gpsData->GGA_datum.latitude_degrees << ", " << gpsData->GGA_datum.longitude_degrees << endl;
		}

		if( gpsData->has_RMC_datum ) {
			out << "RMC: " << gpsData->RMC_datum.latitude_degrees << ", " << gpsData->RMC_datum.longitude_degrees << endl;
		}
	}
	out << "************" << endl;

}
