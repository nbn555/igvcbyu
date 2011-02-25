/*
 * GpsDevice.h.  Regard
 * http://www.gpsinformation.org/dale/nmea.htm for further information
 * on GPGGA and GPRMC qualities.
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#ifndef GPSDEVICE_H_
#define GPSDEVICE_H_

#include <iostream>
#include <mrpt/slam/CObservationGPS.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <string>
#include <mrpt/utils/CConfigFile.h>
#include "WaypointPlanner.h"

#include "YClopsSensor.h"

class GPS: protected mrpt::hwdrivers::CGPSInterface, public YClopsSensor {

public:
	GPS(const int Buffer_Length = 500);
	void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);
	void init();
	void sensorProcess();
	SensorData * getData();

	/*
	 * Dumps current GPS Latitude and Longitude points GGA and RMC
	 * @param out the ostream to which to write the data
	 */
	void dumpData(std::ostream & out ) const;

	virtual ~GPS();

	/*
	 * Returns the distance to a waypoint with respect to current
	 * location in meters.
	 * Returns 0.0 if no data to return.
	 */
	double GetDistanceToWaypoint (double lat, double lon);

	/*
	 * Returns the distance between any two locations in meters.
	 * Returns 0.0 if no data to return.
	 */
	double GetDistanceToWaypoint (double lat1, double lon1,
										double lat2, double lon2);

	/*
	 * Returns Speed if there is GPRMC data available. Returns 0
	 * if not.
	 */
	double GetGpsSpeed();

	/*
	 * Returns directions if there is GPRMC data available. Returns 0.0
	 * if not.
	 */
	double GetGpsDirection();

	/*
	 * Returns current latitude.  0.0 if information not currently
	 * available.
	 */
	double GetGpsLatitude();

	/*
	 * Returns current longitude. 0.0 if information not currently
	 * available.
	 */

	double GetGpsLongitude();

	/*
	 * Returns both current latitude and longitude.
	 */
	mrpt::poses::CPoint2D GetCurrentGpsLocation();

protected:
private:
	std::string vendor;
	bool isGpggaUsed;
	bool isGprmcUsed;
	std::string portName;//These are needed for sending initial commands to the old GPS
	int baudRate;
	int processRate;
	// Needed for the doProcess function call of the mrpt GPS
	CGenericSensor::TListObservations				lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	mrpt::slam::CObservationGPSPtr 					gpsData;

	// Initializes the serial port with nmea commands to turn on continous GPS data flow
	void initializeCom();

	// Initializes the Config File passed to it from initialize()
	void initConfig(mrpt::utils::CConfigFileBase & configSource,
					const std::string & iniSection = "GPS");

	// GPGGA gives static information for GPS communication
	bool usesGpgga() const { return this->isGpggaUsed; };

	// GPRMC give robust information for GPS communication
	bool usesGprmc() const { return this->isGprmcUsed; };
};

#endif /* GPSDEVICE_H_ */
