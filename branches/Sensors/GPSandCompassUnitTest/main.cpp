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

bool TESTING = true;

double PrevPosLat;
double PrevPosLon;
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

bool testGPSConnection(GPS * gps)
{
	return (gps->isGPS_connected());
}


bool testGPSShowData(GPS * gps, int numOfRecord){
	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	while(lstObs.size() < numOfRecord)
	{
		gps->doProcess();
		gps->getObservations(lstObs);
		//cout << "Getting Observation from GPS ..." << endl;
	}

	for(itObs = lstObs.begin(); itObs != lstObs.end(); itObs++){
		CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);
		//cout << "Lat: " << gpsData.pointer()->GGA_datum.latitude_degrees << endl;
		//cout << "Lon: " << gpsData.pointer()->GGA_datum.longitude_degrees << endl;

		if(!gpsData.pointer()->has_GGA_datum)
			cout << "gpsData.pointer()->has_GGA_datum fails" << endl;
		if(!gpsData.pointer()->has_RMC_datum)
			cout << "gpsData.pointer()->has_RMC_datum fails" << endl;
	}
	return true;
}

bool testCompassConnection(Compass * compass) {
	//mrpt::hwdrivers::TSensorState state = compass->getState();
	//return (state == mrpt::hwdrivers::ssWorking);
}

// TODO
bool testCompassShowData(Compass * compass, int numOfRecord){
	return false;
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

	CConfigFile config("GPS.ini");
	GPS * gps = new GPS();
	Compass * compass = new Compass(string("Compass.ini"));
	int showRecord = 100;
	cout << "Start GPS and Compass unit testing" << endl;

	assert(testGPSConnection(gps) == true);
	assert(testGPSShowData(gps, showRecord) == true);
	assert(testCompassConnection(compass) == true);
	assert(testCompassShowData(compass, showRecord) == true);

	assert(testCompassAndGPSConnections(gps, compass) == true);
	assert(testCompassAndGPSShowData(gps, compass) == true);
	cout << "Pass all the unit tests" << endl;

	return 0;
}
