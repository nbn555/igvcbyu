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

CObservationGPSPtr gpsData;

/*
 * List of unusable functions.  Should be helpful in finding information from GPS
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

double GetDistanceToWaypoint (double lon1, double lat1, double lon2, double lat2) {
	return AbstractNavigationInterface::haversineDistance(lon1, lat1, lon2, lat2);
}

int main() {

	GPS gps;
	//gps.setSerialPortName ( "ttyS0" );
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
	}
		//CObservationGPS(((CObservationGPS)(lstObs.begin()))).dumpToConsole();
	gpsData = CObservationGPSPtr(lstObs.begin()->second);
	gpsData->dumpToConsole();
	//TSPNavigation solver(bla);


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

	//double lon1 = 40.247589;
	//double lat1 = -111.648093;
	//double lon2 = 40.247347;
	//double lat2 = -111.648407;
	//cout << GetDistanceToWaypoint(lon1, lat1, lon2, lat2);
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
	ofstream fout("readings.txt"); // to take GPS readings for testing
	fout << "Longitude" << setw(15) << "Latitude" << setw(15) << "Distance from last"<<  setw(15) << endl;
	while (! mrpt::system::os::kbhit())
	{
		cout << "press enter to get gps reading" << endl;
		getchar();

		gps.doProcess();
		mrpt::system::sleep( 500 );

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
				}*/

				//gpsData->dumpToConsole();
				fout << curPos.m_coords[0] << setw(15) << curPos.m_coords[1] << setw(15) << "GPGGA " << endl;
				fout << curPos.m_coords[0] << setw(15) << curPos.m_coords[1] << setw(15) << "GPRMC " << endl;
			}


			lstObs.clear();
		}
		fout.close();
	}
	return 0;
}
