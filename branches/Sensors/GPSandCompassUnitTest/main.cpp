/*
 * 	main.cpp for GPS and Compass Unit Test
 *
 *  Created on: Feb. 10, 2011
 *  Author: 	IGVC.BYU
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <cstdio>   /* Standard input/output definitions */
#include <cstring>  /* String function definitions */
#include <vector>

#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include "GPS.h"
#include "WaypointPlanner.h"
#include "Compass.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace std;


CObservationGPSPtr gpsData;
CGenericSensor::TListObservations				lstObs;
CGenericSensor::TListObservations::iterator 	itObs;
bool outside = false;

const double metersToFeet = 3.2808399;

/******************GPS Functions************************/

/*
 * List of GPS functions used to extract information from
 * the GPS. Can be used to double check.
 */

/******************Compass Functions************************/

/*
 * List of Compass functions used to extract information from
 * the Compass
 */



/***************************Main****************************/

bool testGPSInitialize(GPS * gps, CConfigFile * config) {
	gps->loadConfig(*config, "GPS");
	gps->initConfig(*config, "GPS");
	gps->initialize();

	if (outside) {
		int i = 20;
		for(; !lstObs.size() && i > 0; i--) // try 10 seconds
		{
			gps->doProcess();
			gps->getObservations(lstObs);
			mrpt::system::sleep(500);
			cout << "Getting Observation from GPS ..." << endl;
		}


		gpsData = CObservationGPSPtr(lstObs.begin()->second);
		if (!(i > 0))
				cout << "no info in lstObs, testGPSInitialize Class" << endl;
		return (i > 0); // this means it successfully got data in lstObs
	}
	else {
		return true;
	}
}

bool testGPSConnection(GPS * gps)
{
	return gps->isGPS_connected();
}

bool testGPSShowData(GPS * gps, unsigned numOfRecord){

	gps->doProcess();
	gps->getObservations( lstObs );
	for(itObs = lstObs.begin(); itObs != lstObs.end(); itObs++){

		CObservationGPSPtr gpsData=CObservationGPSPtr(itObs->second);
		//cout << "Lat: " << gpsData.pointer()->GGA_datum.latitude_degrees << endl;
		//cout << "Lon: " << gpsData.pointer()->GGA_datum.longitude_degrees << endl;

		if(!gpsData.pointer()->has_GGA_datum)
			cout << "gpsData.pointer()->has_GGA_datum fails" << endl;
		if(!gpsData.pointer()->has_RMC_datum)
			cout << "gpsData.pointer()->has_RMC_datum fails" << endl;
		gps->dumpData(cout);
		return true;
	}
	cout << "No gps data to show" << endl;
	return false;
}

bool testCompassInitialize(Compass * compass, CConfigFile * config) {
	compass = new Compass(*config);
	return true;
}

bool testCompassConnection(Compass * compass) {
	//mrpt::hwdrivers::TSensorState state = compass->getState();
	//return (state == mrpt::hwdrivers::ssWorking);
	return true;
}

// TODO
bool testCompassShowData(Compass * compass, int numOfRecord){
	for (int i = 0; i < numOfRecord; i++) {
		compass->doProcess();
		compass->dumpData(cout);
		mrpt::system::sleep(1000);
	}
		return(compass->getPitch() != 0);
	}

// TODO
bool testCompassAndGPSConnections(GPS * gps, Compass * compass){
	return false;
}

// TODO
bool testCompassAndGPSShowData(GPS * gps, Compass * compass){
	return false;
}

int main( int argc, char** argv ) {


	////////////////Configure GPS
	CConfigFile * config = new CConfigFile("GPS.ini");
	GPS * gps = new GPS();


	//////////////////Configure COMPASS
	Compass comp;
	CConfigFile * compassConfig = new CConfigFile( "Compass.ini" );

	Compass * compass = &comp;

	//Compass * compass = new Compass(string("Compass.ini"));
	int showRecord = 20;
	cout << "Start GPS and Compass unit testing" << endl;
	//assert(testGPSInitialize(gps, config) == true);
	//assert(testGPSConnection(gps) == true);
	//assert(testGPSShowData(gps, showRecord) == true);
	assert(testCompassInitialize(compass, compassConfig) == true);
	assert(testCompassConnection(compass) == true);
	assert(testCompassShowData(compass, showRecord) == true);

	assert(testCompassAndGPSConnections(gps, compass) == true);
	assert(testCompassAndGPSShowData(gps, compass) == true);
	cout << "Pass all the unit tests" << endl;

	return 0;
}
