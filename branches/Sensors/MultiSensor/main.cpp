/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <vector>

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/utils/CConfigFile.h>

#include "Compass.h"
#include "GPS.h"
#include "WaypointPlanner.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

void initCompass( Compass & compass );
void initGPS( GPS & gps );

int main( int argc, char** argv ) {

	Compass compass(false);
	initCompass(compass);

	GPS gps;
	initGPS(gps);

	cout << gps.isGPS_connected() << endl;

	AbstractNavigationInterface * solver = new SequentialNavigation(40.247605, -111.648192);
	solver->loadPoints("gpsWeighpoints_raw.txt");
	vector<CPoint2D> visited = solver->solve();

	while(!mrpt::system::os::kbhit()) {
		mrpt::system::sleep( 500 );
		compass.doProcess();
		gps.doProcess();

		CGenericSensor::TListObservations				lstObs;
		CGenericSensor::TListObservations::iterator 	itObs;

		gps.getObservations( lstObs );

		if( compass.isYawValid() ) cout << "Yaw: " << compass.getYaw() << endl;
		if( compass.isPitchValid() ) cout << "Pitch: " << compass.getPitch() << endl;
		if( compass.isRollValid() ) cout << "Roll: " << compass.getRoll() << endl;
		cout << endl;

		for (itObs=lstObs.begin();itObs!=lstObs.end();itObs++)
		{
			ASSERT_(itObs->second->GetRuntimeClass()==CLASS_ID(CObservationGPS));

			CObservationGPSPtr gpsData=CObservationGPSPtr(itObs->second);

			gpsData->dumpToConsole();

		}

		lstObs.clear();

	}

	cout.precision(10);
	for(vector<CPoint2D>::iterator point = visited.begin(); point != visited.end(); ++point )
		cout << point->m_coords[TSPNavigation::LAT] << " " << point->m_coords[TSPNavigation::LON] << endl;

	delete solver;
	return 0;
}

void initCompass( Compass & compass ) {
	CConfigFile config( "MS.ini" );

	compass.loadConfig(config, "COMPASS" );
	compass.initialize();
}

void initGPS( GPS & gps ) {
	gps.setSerialPortName ( "ttyUSB0" );
	CConfigFile config("MS.ini");
	gps.loadConfig(config, "GPS");
	gps.initConfig(config, "GPS");

	gps.initialize();
}
