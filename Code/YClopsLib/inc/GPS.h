/**
 * @file GpsDevice.h.
 * @link http://www.gpsinformation.org/dale/nmea.htm for further information on GPGGA and GPRMC qualities.
 * @date Nov 3, 2010
 * @author igvcbyu
 * @brief GPS Sensor Header
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

/**
 * @brief GPS interface class
 */
class GPS: protected mrpt::hwdrivers::CGPSInterface, public YClopsSensor {

public:
	GPS(const int Buffer_Length = 500);
	void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);
	void init();
	void sensorProcess();
	SensorData * getData();

	/**
	 * @brief Dumps current GPS Latitude and Longitude points GGA and RMC
	 * @param out the ostream to which to write the data
	 */
	void dumpData(std::ostream & out ) const;

	virtual ~GPS();

	/**
	 * Returns the distance to a waypoint with respect to current
	 * location in meters.
	 * Returns 0.0 if no data to return.
	 */
	double GetDistanceToWaypoint (double lat, double lon)const;

	/**
	 * Returns the distance between any two locations in meters.
	 * Returns 0.0 if no data to return.
	 */
	double GetDistanceToWaypoint (double lat1, double lon1,
										double lat2, double lon2)const;

	/**
	 * Returns Speed if there is GPRMC data available. Returns 0
	 * if not.
	 */
	double GetGpsSpeed()const;

	/**
	 * Returns directions if there is GPRMC data available. Returns 0.0
	 * if not.
	 */
	double GetGpsDirection()const;

	/**
	 * Returns current latitude.  0.0 if information not currently
	 * available.
	 */
	double GetGpsLatitude()const;

	/**
	 * Returns current longitude. 0.0 if information not currently
	 * available.
	 */
	double GetGpsLongitude() const;

	/**
	 * @brief Returns both current latitude and longitude.
	 * @return mrpt::poses::CPoint2D the latitude and longitude of the gps
	 */
	mrpt::poses::CPoint2D GetCurrentGpsLocation() const;

	/*
	 * IsDataValid functions tests to ensure the returned gps point is within a certain distance
	 * of the testLat and testLon given in the .ini file.
	 */
	bool IsGGADataValid() const;
	bool IsRMCDataValid() const;

protected:
	/**
	 * @brief A class to abstract the specific GPS command strings from the rest of the code
	 */
	class GPSStringer {
	public:
		static const std::string POCKETMAX;
		static const std::string NOVATEL;
		std::string portName;					//!< The port the gps is connected to
		const std::string vendor;				//!< The type of gps
		const std::string clearCommand;			//!< The command to stop the gps from transmitting data
		const std::string clearCommandResponse;	//!< The acknoledge string after stopping data
		const std::string ggaCommand;			//!< The command to get gga data
		const std::string rmcCommand;			//!< The command to get rmc data
		virtual ~GPSStringer() {};
	protected:
		GPSStringer( const std::string port, const std::string vendor, const std::string clearCommand,
				const std::string clearCommandResponse, const std::string ggaCommand,
				const std::string rmcCommand ): portName(port), vendor(vendor), clearCommand(clearCommand),
				clearCommandResponse(clearCommandResponse), ggaCommand(ggaCommand), rmcCommand(rmcCommand) { };
	};

	/**
	 * @brief A class initialized with the Novatel command strings
	 */
	class NovatelGPSStringer: public GPSStringer {
	public:
		NovatelGPSStringer( std::string port ): GPSStringer(port, GPSStringer::NOVATEL, "unlogall", "", "log gpgga ontime ", "log gprmc ontime " ) {};
		virtual ~NovatelGPSStringer() { };
	};

	/**
	 * @brief A class initialized with the PocketMax command strings
	 */
	class PocketMaxGPSStringer: public GPSStringer {
	public:
		PocketMaxGPSStringer( std::string port ): GPSStringer(port, GPSStringer::POCKETMAX, "$joff", "$>", "$jasc,gpgga,", "$jasc,gprmc,") {};
		virtual ~PocketMaxGPSStringer() { };
	};
private:
	bool isGpggaUsed;	//!<If we use data from the gga datum
	bool isGprmcUsed;	//!<If we use data from the rmc datum
	int baudRate;		//!<The baud rate the gps uses
	int processRate;
	double testLat;		//!<For testing waypoints to make sure they are valid (must be within 10000 meters of point)
	double testLon;		//!<For testing waypoints for validity
	GPSStringer * gpsStrings; //!<The string set the gps interface uses
	// Needed for the doProcess function call of the mrpt GPS
	CGenericSensor::TListObservations				lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	mrpt::slam::CObservationGPSPtr 					gpsData;

	/**
	 *  @brief Initializes the serial port with nmea commands to turn on continuous GPS data flow
	 */
	void initializeCom();

	/**
	 * @brief tells us if we are using GGA datum
	 * @note GPGGA gives static information for GPS communication
	 * @return bool true if GGA datum is used
	 */
	bool usesGpgga() const { return this->isGpggaUsed; };

	/**
	 * @brief tells us if we are using RMC datum
	 * @note  GPRMC give robust information for GPS communication
	 * @return bool true if RMC datum is used
	 */
	bool usesGprmc() const { return this->isGprmcUsed; };
};

#endif /* GPSDEVICE_H_ */
