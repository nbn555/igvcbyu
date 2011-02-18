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

#define length 7

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

//CObservationGPSPtr gpsData;

#define reps 1000

/*
 * List of functions.  Should be helpful in finding information from GPS
 */

void accuracyTest(GPS2 * gps) {

	double totalSum = 0;
	double stability = 0;

	double gpsAccuracyArray[reps];
	double preLat = gps->GetGpsLatitude();
	double preLon = gps->GetGpsLongitude();
	for (int i = 0; i < reps; i++) {
		mrpt::system::sleep(1000);
		gps->doGPSProcess();
		cout.precision(14);
		totalSum += gpsAccuracyArray[i] = gps->GetDistanceToWaypoint(preLat, preLon);
		cout << "Distance = " << gpsAccuracyArray[i] << endl;

		// tests if no GPS data was returned or if returned too large error
		if (gpsAccuracyArray[i] == 0.0 || gpsAccuracyArray[i] > 5) ++stability;

		preLat = gps->GetGpsLatitude();
		preLon = gps->GetGpsLongitude();
	}
	cout << endl << "Accuracy for " << reps << " points taken:       " << totalSum/(reps - stability) << " meters." << endl;

	cout << "And " << stability << " readings lost" << endl;

}

int main() {

	GPS2 gps;
	CConfigFile * config = new CConfigFile("GPS.ini");
	gps.initialize(config);

	accuracyTest(&gps);

	return 0;
}
