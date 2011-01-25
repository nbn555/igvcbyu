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

#define length 7

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

CObservationGPSPtr gpsData;

bool TESTING = true;

double PrevPosLat;
double PrevPosLon;
const double metersToFeet = 3.2808399;
/*
 * List of functions.  Should be helpful in finding information from GPS
 */

double GetGpsSpeed() {
	const double knotToMph = 0.868976242;
	return (gpsData->RMC_datum.speed_knots * knotToMph);
}

double GetGpsDirection() {
	return gpsData->RMC_datum.direction_degrees;

	return 0.0;
}

double GetGpsLat() {
	if (gpsData->has_GGA_datum)
		return gpsData->GGA_datum.latitude_degrees;

	return 0.0;
}

double GetGpsLon() {
	if (gpsData->has_GGA_datum)
		return gpsData->GGA_datum.longitude_degrees;

	return 0.0;
}

double GetDistanceToWaypoint (double lon, double lat) {
	return AbstractNavigationInterface::haversineDistance(lon, lat, gpsData->GGA_datum.longitude_degrees, gpsData->GGA_datum.latitude_degrees);
}

double GetDistanceToWaypoint (double lat1, double lon1, double lat2, double lon2) {
	return AbstractNavigationInterface::haversineDistance(lat1, lon1, lat2, lon2);
}


int main() {

	cout.precision(14);
	GPS gps;

	gps.setSerialPortName ( "ttyUSB0" ); // for use of usb to serial on laptop

	CConfigFile config("GPS.ini");
	gps.loadConfig(config, "GPS");
	gps.initConfig(config, "GPS");

	gps.initialize();

	cout << gps.isGPS_connected() << endl;

	CGenericSensor::TListObservations				lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;

	//prime the pump with the first gps observation

	while(!lstObs.size()) {
		gps.doProcess();
		mrpt::system::sleep(500);
		gps.getObservations(lstObs);
		PrevPosLat = 0.0;
		PrevPosLon = 0.0;
	}

	gpsData = CObservationGPSPtr(lstObs.begin()->second);

	cout << (int)gpsData->GGA_datum.fix_quality << endl;
	cout << gpsData->RMC_datum.validity_char << endl;

	CPoint2D curPos;

	if(gps.usesGpgga() && 2 == gpsData->GGA_datum.fix_quality ) {
		curPos.m_coords[0] = gpsData->GGA_datum.latitude_degrees;
		curPos.m_coords[1] = gpsData->GGA_datum.longitude_degrees;
	} else if(gps.usesGprmc() && 'V' == gpsData->RMC_datum.validity_char) {
		curPos.m_coords[0] = gpsData->RMC_datum.latitude_degrees;
		curPos.m_coords[1] = gpsData->RMC_datum.longitude_degrees;
	} else {
		cerr << "Invalid GPS data" << endl;
	}

	// Commented this code out because TSPNavigation was throwing an eigen error of some sorts
////////////////////////////////////////////////////
	/*
	gps.doProcess();

	//double lat = curPos.m_coords[0];
	//double lon = curPos.m_coords[1];

	cout << "Solving tsp " << endl;
	TSPNavigation solver( curPos.m_coords[0], curPos.m_coords[1] );
	//TSPNavigation solver( const curPos );
	solver.loadPoints("gpsWeighpoints_raw.txt");
	vector<CPoint2D> visitOrder = solver.solve();
	cout.precision(10);
	for(vector<CPoint2D>::iterator point = visitOrder.begin(); point != visitOrder.end(); ++point )
		cout << point->m_coords[TSPNavigation::LAT] << " " << point->m_coords[TSPNavigation::LON] << endl;

	unsigned visitOrderIndex = 0;
	*/
////////////////////////////////////////////////////

	//  Writes GPS latitude, longitude, and distance between both into a file.

	ofstream fout("readings.txt"); // to take GPS readings for testing
	fout.precision(12);

	fout << "Latitude" << setw(20) << "Longitude" << setw(20) << "   Distance from last(ft)"<< endl;


	while(TESTING)//while (! mrpt::system::os::kbhit())

	{
		cout << "press enter to get GPS reading" << endl;
		getchar(); // waits for a command before proceeding

		gps.doProcess();
		mrpt::system::sleep( 1000 );

		gps.getObservations( lstObs );

		if (lstObs.empty())
		{
			printf("[Test_GPS] Waiting for data...\n");
			cout << "Please press enter again" << endl;
		}
		else
		{
			for (itObs=lstObs.begin();itObs!=lstObs.end();itObs++)
			{
				ASSERT_(itObs->second->GetRuntimeClass()==CLASS_ID(CObservationGPS));

				CObservationGPSPtr gpsData=CObservationGPSPtr(itObs->second);

				if(gps.usesGpgga()) {
					curPos.m_coords[0] = gpsData->GGA_datum.latitude_degrees;
					curPos.m_coords[1] = gpsData->GGA_datum.longitude_degrees;
				} else if(gps.usesGprmc()) {
					curPos.m_coords[0] = gpsData->RMC_datum.latitude_degrees;
					curPos.m_coords[1] = gpsData->RMC_datum.longitude_degrees;
				} else {
					cerr << "Invalid GPS data" << endl;
					exit(EXIT_FAILURE);
				}

				gpsData->dumpToConsole(); // displays MRPT form data of all GPA measurements and calculations on console

				//sends data to a file for current gps location and distance from last location visited
				fout << curPos.m_coords[0] << setw(20) << curPos.m_coords[1] << setw(20) << (metersToFeet * GetDistanceToWaypoint(curPos.m_coords[0] ,curPos.m_coords[1] ,PrevPosLat, PrevPosLon))<< endl << endl;

				PrevPosLat = curPos.m_coords[0];
				PrevPosLon = curPos.m_coords[1];



				//commented out this code because it was throwing an eigen errors of some sorts
////////////////////////////////////////////////////
				/*
				assert(curPos.m_coords[0] != 0 );
				assert(curPos.m_coords[1] != 0 );


				cout.precision(10);
				 if( visitOrderIndex < visitOrder.size() ) {
					cout << "At position " << curPos.m_coords[0] << "," << curPos.m_coords[1] << endl;
					cout << "going to " << visitOrder[visitOrderIndex].m_coords[0] <<
					"," << visitOrder[visitOrderIndex].m_coords[1] << endl;
					double dist = TSPNavigation::haversineDistance(
							visitOrder[visitOrderIndex].m_coords[0],
							visitOrder[visitOrderIndex].m_coords[1],
							curPos.m_coords[0],curPos.m_coords[1]);
					cout << "at a dist of " << dist << endl;
					if( 1 > dist ) {
						visitOrderIndex++;
						cout << "going to next weighpoint" << endl;
					}
				}
			*/

////////////////////////////////////////////////////
			}
			lstObs.clear();
		}
	fout.close();
	}

	return 0;
}


/*  // code for determine distances if needed
int main() {

	double lat[length] = {
			40.24711,
			40.2470983,
			40.2471167,
			40.2471133,
			40.2471167,
			40.2471067,
			40.24712
	};

	double lon[length] = {
			-111.648653,
			-111.648753,
			-111.648652,
			-111.648745,
			-111.648647,
			-111.648757,
			-111.648655
	};


	double metersToFeet = 3.2808399;
	//gpsData->dumpToConsole();
	int n = 1;
	while (n < length) {
	//cout << GetDistanceToWaypoint(lat1, lon1, lat2, lon2);
	cout << "Feet Conversion = " << (GetDistanceToWaypoint(lat[n], lon[n], lat[n-1], lon[n-1])*metersToFeet)<< endl;
	++n;
	}
	return 0;
}
*/
