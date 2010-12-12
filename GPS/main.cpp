/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *  Modified on: Oct 26, 2010 for GPS application
 *      Author: igvcbyu
 */

#include <iostream>

#include <cstdio>   /* Standard input/output definitions */
#include <cstring>  /* String function definitions */
#include <vector>

#include <mrpt/hwdrivers/CGPSInterface.h>

#include "GPS.h"
#include "TSPSolver.h"
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

	CPoint2D curPos;
	if(gps.usesGpgga()) {
		curPos.m_coords[0] = gpsData->GGA_datum.latitude_degrees;
		curPos.m_coords[1] = gpsData->GGA_datum.longitude_degrees;
	} else if(gps.usesGprmc()) {
		curPos.m_coords[0] = gpsData->RMC_datum.latitude_degrees;
		curPos.m_coords[1] = gpsData->RMC_datum.longitude_degrees;
	} else {
		cerr << "Invalid GPS data" << endl;
	}

	TSPSolver solver( curPos.m_coords[0], curPos.m_coords[1] );
	solver.loadPoints("gpsWeighpoints_raw.txt");
	vector<CPoint2D> visitOrder = solver.solve();
	int visitOrderIndex = 0;

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
				}

				if( visitOrderIndex < visitOrder.size() ) {
					cout << "At position " << visitOrder[visitOrderIndex].m_coords[0] <<
							"," << visitOrder[visitOrderIndex].m_coords[1] << endl;
					if( 1 < TSPSolver::haversineDistance(
							visitOrder[visitOrderIndex].m_coords[0],
							visitOrder[visitOrderIndex].m_coords[1],
							curPos.m_coords[0],curPos.m_coords[1]) ) {
						visitOrderIndex++;
					}
				}

				gpsData->dumpToConsole();
			}
			lstObs.clear();
		}
	}

	return 0;
}
