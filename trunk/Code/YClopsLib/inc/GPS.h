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

namespace mrpt {
	namespace hwdrivers {
		/**
		 * @brief GPS interface class
		 */
		class GPS: public CGPSInterface {
			DEFINE_GENERIC_SENSOR(GPS);
		public:
			GPS(const int Buffer_Length = 500);
			void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);
			void initialize();
			void sensorProcess();
			SensorData * getData();

			/**
			 * @brief Dumps current GPS Latitude and Longitude points GGA and RMC
			 * @param out the ostream to which to write the data
			 */
			void dumpData(std::ostream & out ) const;

			virtual ~GPS();

			/**
			 * @brief computes the distance to a waypoint with respect to current location in meters.
			 * @return double distance measured in meters or 0.0 if no data to return.
			 */
			double GetDistanceToWaypoint (double lat, double lon)const;

			/**
			 * @brief computes the distance between two locations in meters.
			 * @return double distance measured in meters returns 0.0 if no data to return.
			 */
			double GetDistanceToWaypoint (double lat1, double lon1,
												double lat2, double lon2)const;

			/**
			 * @brief gets speed if there is GPRMC data available. Returns 0 if not.
			 * @return double speed in miles per hour
			 */
			double GetGpsSpeed()const;

			/**
			 * @brief gets current pose from the GPRMC data if available available. Returns 0.0 if not.
			 * @return double direction as measured in radians
			 */
			double GetGpsDirection()const;

			/**
			 * @brief gets the current latitude.  0.0 if information not currently available.
			 * @return double current latitude measured in degrees
			 */
			double GetGpsLatitude()const;

			/**
			 * @brief gets the current longitude. 0.0 if information not currently available.
			 * @return double current longitude measured in degrees
			 */
			double GetGpsLongitude() const;

			/**
			 * @brief Returns both current latitude and longitude.
			 * @return mrpt::poses::CPoint2D the latitude and longitude of the gps
			 */
			mrpt::poses::CPoint2D GetCurrentGpsLocation() const;

			/**
			 * @brief IsGGADataValid functions tests to ensure the returned gps point is within a certain distance
			 * of the testLat and testLon given in the .ini file.
			 * @return bool true if data recieved is valid
			 */
			bool IsGGADataValid() const;

			/**
			 * @brief IsGGADataValid functions tests to ensure the returned gps point is within a certain distance
			 * of the testLat and testLon given in the .ini file.
			 * @return bool true if data recieved is valid
			 */
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
				const std::string setComSpeed;			//!< The command string to set the baud rate
				virtual ~GPSStringer() {};
			protected:
				GPSStringer( const std::string port, const std::string vendor, const std::string clearCommand,
						const std::string clearCommandResponse, const std::string ggaCommand,
						const std::string rmcCommand, const std::string comSpeed ): portName(port), vendor(vendor), clearCommand(clearCommand),
						clearCommandResponse(clearCommandResponse), ggaCommand(ggaCommand), rmcCommand(rmcCommand),
						setComSpeed(comSpeed) { };


		};

			/**
			 * @brief A class initialized with the Novatel command strings
			 */
			class NovatelGPSStringer: public GPSStringer {
			public:
				NovatelGPSStringer( std::string port ): GPSStringer(port, GPSStringer::NOVATEL, "unlogall", "", "log gpgga ontime ", "log gprmc ontime ", "COM " ) {};
				virtual ~NovatelGPSStringer() { };
			};

			/**
			 * @brief A class initialized with the PocketMax command strings
			 */
			class PocketMaxGPSStringer: public GPSStringer {
			public:
				PocketMaxGPSStringer( std::string port ): GPSStringer(port, GPSStringer::POCKETMAX, "$joff", "$>", "$jasc,gpgga,", "$jasc,gprmc,", "$jbaud,") {};
				virtual ~PocketMaxGPSStringer() { };
			};
		private:
			bool isGpggaUsed;	//!<If we use data from the gga datum
			bool isGprmcUsed;	//!<If we use data from the rmc datum
			int baudRate;		//!<The baud rate the gps uses
			int processRate;
			double testLat;		//!<For testing waypoints to make sure they are valid (must be within 10000 meters of point)
			double testLon;		//!<For testing waypoints for validity
			std::string vendor; //!<To give proper commands for intialize com
			GPSStringer * gpsStrings; //!<The string set the gps interface uses
			// Needed for the doProcess function call of the mrpt GPS
			CGenericSensor::TListObservations				lstObs;
			CGenericSensor::TListObservations::iterator 	itObs;
			mrpt::slam::CObservationGPSPtr 					gpsData;

			/**
			 * @brief pre communication to serial port at baud 9600.  Sets to desired baudrate found in .ini file
			 */
			void preComInitialize();

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
	}
}
#endif /* GPSDEVICE_H_ */
