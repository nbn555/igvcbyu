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


bool testGPS(GPS * gps, CConfigFile & config )
{



	gps->setSerialPortName ( config.read_string("GPS", "COM_port_LIN", "/dev/ttyUSB1" ) );
	gps->loadConfig(config, "GPS");
	gps->initConfig(config, "GPS");
	gps->initialize();


	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;

	bool done = false;
	while(!done)
	{
		while(lstObs.size() == 0)
		{

			gps->doProcess();
			gps->getObservations(lstObs);
			cout << "Getting Observation from GPS ..." << endl;
		}
		itObs = lstObs.begin();
		CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);
		if(gpsData.pointer()->has_GGA_datum || gpsData.pointer()->has_RMC_datum)
		{
			if(gpsData.pointer()->has_GGA_datum)
				cout << "Got initial point at " << gpsData.pointer()->GGA_datum.latitude_degrees << ":" << gpsData.pointer()->GGA_datum.longitude_degrees << endl;
			if(gpsData.pointer()->has_RMC_datum)
				cout << "Got initial point at " << gpsData.pointer()->RMC_datum.latitude_degrees << ":" << gpsData.pointer()->RMC_datum.longitude_degrees << endl;
			done = true;
		}
		else
		{
			cout << "ERROR: INVALID DATA" << endl;
			gpsData.pointer()->dumpToConsole();
		}
	}


}

bool testCompass() {
	Compass * compass = new Compass(string("Compass.ini"));
	compass -> doProcess();
	compass->dumpData(cout);

}
int main( int argc, char** argv ) {

	CConfigFile config("GPS.ini");
	GPS * gps = new GPS();

	assert(testGPSConnection(gps) == false);
	assert(testGPS(gps, config) == false);
	//if(assert(testGPS() == false))
		//cout << "Error: testGPS" << endl;

	//this->camera = new Camera();
	return 0;
}
