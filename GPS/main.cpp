/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *  Modified on: Oct 26, 2010 for GPS application
 *      Author: igvcbyu
 */

#include <iostream>
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

int main() {

	GPS gps;
	gps.setSerialPortName ( "ttyS0" );
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

	CObservationGPSPtr gpsData=CObservationGPSPtr(lstObs.begin()->second);

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

	cout << "Solving tsp " << endl;
	TSPNavigation solver( curPos.m_coords[0], curPos.m_coords[1] );
	solver.loadPoints("gpsWeighpoints_raw.txt");
	vector<CPoint2D> visitOrder = solver.solve();
	cout.precision(10);
	for(vector<CPoint2D>::iterator point = visitOrder.begin(); point != visitOrder.end(); ++point )
		cout << point->m_coords[TSPNavigation::LAT] << " " << point->m_coords[TSPNavigation::LON] << endl;

	unsigned visitOrderIndex = 0;

	while (! mrpt::system::os::kbhit())
	{
		gps.doProcess();
		mrpt::system::sleep( 500 );

		gps.getObservations( lstObs );

		if (lstObs.empty())
		{
			printf("[Test_GPS] Waiting for data...\n");
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

				gpsData->dumpToConsole();
			}
			lstObs.clear();
		}
	}

	return 0;
}
