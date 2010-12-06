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

#include <mrpt/hwdrivers/CGPSInterface.h>
#include "GPS.h"
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

	CGenericSensor::TListObservations			lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;

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
				gpsData->dumpToConsole();
			}
			lstObs.clear();
		}
	}

	return 0;
}
