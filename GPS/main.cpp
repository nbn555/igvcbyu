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

bool TESTING = false;

double PrevPosLat;
double PrevPosLon;
const double metersToFeet = 3.2808399;
/*
 * List of functions.  Should be helpful in finding information from GPS
 */

int main() {

	//GPS2 gps;
	return 0;
}
