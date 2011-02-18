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

#include "GPS2.h"
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

void accuracyTest(GPS2 * gps) {
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
		gps->doGPSProcess();
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

int main() {

	GPS2 gps;
	CConfigFile * config = new CConfigFile("GPS.ini");
	gps.initialize(config);

	accuracyTest(&gps);

	return 0;
}
