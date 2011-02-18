/*
 * GpsDevice.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#include "GPS2.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;


GPS2::GPS2(const int Buffer_Length):CGPSInterface(Buffer_Length), isGpggaUsed(false), isGprmcUsed(false) { //fixes potential default constructor problem
}

GPS2::~GPS2() {

}

void GPS2::initialize(CConfigFile * config) { // we might change to CConfigFileBase
	loadConfig(*config, "GPS");
	initConfig(*config, "GPS");
	initializeCom();

	while(!lstObs.size()) {
		doProcess();
		cout << "Getting GPS observations" << endl;
		mrpt::system::sleep(200);
		getObservations(lstObs);
	}

	gpsData = CObservationGPSPtr(lstObs.begin()->second);

	if(!gpsData.pointer()->has_GGA_datum)
		cout << "gpsData.pointer()->has_GGA_datum fails" << endl;
	if(!gpsData.pointer()->has_RMC_datum)
		cout << "gpsData.pointer()->has_RMC_datum fails" << endl;

}

void GPS2::doGPSProcess() {
	doProcess();
	getObservations( lstObs );

	if (lstObs.empty())
		printf("[Test_GPS] Waiting for data...\n");

	for(itObs = lstObs.begin(); itObs != lstObs.end(); itObs++){

			gpsData=CObservationGPSPtr(itObs->second);

			if(!gpsData.pointer()->has_GGA_datum)
				cout << "gpsData.pointer()->has_GGA_datum fails" << endl;
			if(!gpsData.pointer()->has_RMC_datum)
				cout << "gpsData.pointer()->has_RMC_datum fails" << endl;

		}

}

void GPS2::dumpData(std::ostream & out ) {
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


double GPS2::GetDistanceToWaypoint (double lon, double lat) {
	return AbstractNavigationInterface::haversineDistance(lon, lat, gpsData->GGA_datum.longitude_degrees, gpsData->GGA_datum.latitude_degrees);
}

double GPS2::GetDistanceToWaypoint (double lat1, double lon1, double lat2, double lon2) {
	return AbstractNavigationInterface::haversineDistance(lat1, lon1, lat2, lon2);
}

double GPS2::GetGpsSpeed() {
	const double knotToMph = 0.868976242;
	if (gpsData->has_RMC_datum)
		return (gpsData->RMC_datum.speed_knots * knotToMph);
	else return 0.0;
}

double GPS2::GetGpsDirection() {

	if (gpsData->has_RMC_datum)
		return gpsData->RMC_datum.direction_degrees;

	else
		cout << "No GPS_RMC_datum to return" << endl;
	return 0.0;
}

double GPS2::GetGpsLatitude() {
	if (gpsData->has_GGA_datum)
		return gpsData->GGA_datum.latitude_degrees;

	return 0.0;
}

double GPS2::GetGpsLongitude() {
	if (gpsData->has_GGA_datum)
		return gpsData->GGA_datum.longitude_degrees;

	return 0.0;
}

CPoint2D GPS2::GetCurrentGpsLocation() {
	CPoint2D curPos;
	curPos.m_coords[0] = GetGpsLatitude();
	curPos.m_coords[1] = GetGpsLongitude();

	return curPos;
}



//**************  PRIVATE  *****************************

void GPS2::initializeCom() {
	// check if connection is already established, if not, reconnect
	if (this->isGPS_connected()) {return; }
	cout << "Connecting to GPS ... (./YClopsLib/GPS)" << endl;
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

//This function ought to be called after loadConfig but before initialize
void GPS2::initConfig( mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	this->vendor = config.read_string(sectionName, "GPS_TYPE", "Novatel" );
	this->isGpggaUsed = config.read_bool(sectionName,"use_gga", true);
	this->isGprmcUsed = config.read_bool(sectionName,"use_rmc", false);
	this->baudRate = config.read_int(sectionName,"baudRate", 57600);
	this->portName = config.read_string(sectionName,"COM_port_LIN","/dev/ttyS0");
	this->processRate = config.read_int(sectionName,"process_rate", 1);

}



