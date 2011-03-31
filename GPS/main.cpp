/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *  Modified on: Oct 26, 2010 for GPS application
 *      Author: igvcbyu
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>

#include <cstdio>   /* Standard input/output definitions */
#include <cstring>  /* String function definitions */
#include <vector>

#include <mrpt/hwdrivers/CGPSInterface.h>

#include "GPS.h"
#include "WaypointPlanner.h"

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

//CObservationGPSPtr gpsData;

#define reps 1800

/*
 * List of functions.  Should be helpful in finding information from GPS
 */

void initializationTest(GPS * gps) {
	//gps->dumpData(cout);
	while (1) {
		gps->sensorProcess();
		double lat = gps->GetGpsLatitude();
		double lon = gps->GetGpsLongitude();

		if (lat == 0.0 || lon == 0.0)
			cout << "Invalid readings" << endl;
		else {
			cout << "good" << endl;
			cout << "distance to TestPoint = " << gps->GetDistanceToWaypoint(40.24757, -111.6481) << endl;
		}
		mrpt::system::sleep(1000);
	}
}

void accuracyTest(GPS * gps) {
	ofstream fout("Accuracy Testing.txt"); // to take GPS readings for testing
	fout.precision(12);

	fout << " Simulate for " << reps << " cycles " << endl << endl;

	mrpt::system::sleep(1000);
	double totalSum = 0;
	double stability = 0;
	double distance;

	double gpsAccuracyArray[reps];
	double preLat = gps->GetGpsLatitude();
	double preLon = gps->GetGpsLongitude();
	for (int i = 0; i < reps; i++) {
		mrpt::system::sleep(1000);
		gps->sensorProcess();
		fout.precision(14);
		distance = gps->GetDistanceToWaypoint(preLat, preLon);

		// tests if no GPS data was returned or if returned too large error
		if (distance == 0.0 || distance > 1) {
			++stability;
			fout << " BAD READING " << endl;
		}
		else {
			totalSum += gpsAccuracyArray[i] = distance;
			fout << "Distance = " << gpsAccuracyArray[i] << endl;

		}




		preLat = gps->GetGpsLatitude();
		preLon = gps->GetGpsLongitude();
	}
	fout << endl << "Accuracy for " << (reps) << " points taken:       " << totalSum/(reps-stability) << " meters." << endl;

	fout << "And " << stability << " readings lost" << endl;

	fout.close();
}

void saveGPSpointsToFile(GPS * gps) {
	ofstream fout("readings.txt"); // to take GPS readings for testing
	fout.precision(12);
	fout <<  setw(20) << "Latitude" << setw(20) << "Longitude" << endl;

	double prevLat = 0.0;
	double prevLon = 0.0;

	while(1)//while (! mrpt::system::os::kbhit())

	{
		cout << "press enter to get GPS reading" << endl;
		getchar(); // waits for a command before proceeding

		gps->sensorProcess();
		fout << setw(20) << gps->GetGpsLatitude() << setw(20) << gps->GetGpsLongitude << endl;

		cout << "Please ensure the gps point is good" << endl;
		getchar(); // waits for a command before proceeding

	}
	fout.close();
}


int main() {

	cout << "here" << endl;
	GPS * gps = new GPS();
	CConfigFile config("GPS.ini");
	gps->loadConfiguration(config, "GPS");
	gps->init();

	//initializationTest(gps);
	//accuracyTest(gps);
	saveGPSpointsToFile(gps);



	return 0;
}
