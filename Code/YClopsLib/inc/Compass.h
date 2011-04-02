/**
 * @file Compass.h
 * @date Nov 4, 2010
 * @author igvcbyu
 * @brief Compass sensor header file
 */

#ifndef COMPASS_H_
#define COMPASS_H_

#include "YClopsSensor.h"

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/utils/CConfigFile.h>
#include <string>
#include <iostream>

namespace mrpt {
	namespace hwdrivers{

		/**
		 * @brief Interface to the Honeywell HMR3000 digital compass
		 */
		class Compass: public CGenericSensor {
			DEFINE_GENERIC_SENSOR(Compass);
		public:
			static std::string setParameterResponse;
			Compass();

			void loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName );
			void initialize();
			void doProcess();

			/**
			 * @brief SensorData needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
			 */
			SensorData * getData();
			void dumpData( std::ostream & out ) const;

			virtual ~Compass();

			/**
			 * @brief gets the yaw value from the compass
			 * @return double The last received Yaw value
			 */
			double getYaw() const { return (this->degrees ? this->yaw : mrpt::utils::DEG2RAD(this->yaw)); };

			/**
			 * @brief gets the pitch value from the compass
			 * @return double The last received Pitch value
			 */
			double getPitch() const { return (this->degrees?this->pitch:mrpt::utils::DEG2RAD(this->pitch)); };

			/**
			 * @brief gets the roll value from the compass
			 * @return double the last received Roll value in degrees
			 */
			double getRoll() const { return (this->degrees?this->roll:mrpt::utils::DEG2RAD(this->roll)); };

			/**
			 * @brief gets the validity of the yaw observation
			 * @returns true if the compass returned a valid yaw value
			 * and if that value was less than 999 -> just an impossible degree number
			 * if not reset.
			 */
			bool isYawValid() const { return "N" == this->yawStatus.substr(0,1); };

			/**
			 * @brief gets the validity of the pitch observation
			 * @returns true if the compass returned a valid pitch value
			 */
			bool isPitchValid() const { return "N" == this->pitchStatus.substr(0,1); };

			/**
			 * @brief gets the validity of the roll observation
			 * @returns true if the compass returned a valid roll value
			 */
			bool isRollValid() const { return "N" == this->rollStatus.substr(0,1); };

			/**
			 * @brief gets the state of the output of the compass
			 * @returns bool true if the compass was set to output degrees
			 */
			bool isDegrees() const { return this->degrees; };

			static std::string computeChecksum( const std::string & sentence );

			/**
			 * @brief sets the deviation of the compass
			 * @note deviation is the error introduced to the compass by local magnetic fields
			 * @param deviation measured in degrees
			 * @returns bool true if the command was successfully sent
			 */
			bool setDeviation( double deviation );

			/**
			 * @brief sets the variation of the compass
			 * @note variation is the angular distance between true north and magnetic north for the given location
			 * @param variation aka declination measured in degrees
			 * @returns bool true if the command was successfully sent
			 */
			bool setVariation( double variation );

			/**
			 * @brief turns the compass on to calibration mode
			 * In calibration mode we need to rotate the compass around in a circle slowly until its
			 * internal count reaches 275.
			 * @returns bool true if the compass was successfully put into calibration mode
			 */
			bool setCalibrationMode();

			/**
			 * @brief writes the compass calibration results to the EEPROM
			 * @returns bool true if the command to write the data to EEPROM was successful.
			 */
			bool saveCalibrationResults();

			/**
			 * @brief gets the number of calibration observations from the compass
			 * @returns double a normalized scalar from 0-1 representing the total number of calibration observations
			 */
			double getCalibrationProgress();

		private:

			int offset; 			//!< int offset is for compass if we decide to offset it to true north instead of magnetic north. Configurable in .ini file
			bool degrees;
			double prevYawDeg;		//!<prevYawDeg is for case where compass returns an invalid yaw, it will return the prevYaw value instead in degrees (does not effect final output in radians or degrees)
			double yaw;				//!<the current yaw reading of the compass
			double pitch;			//!<the current pitch reading of the compass
			double roll;			//!<the current roll reading of the compass

			double deviation;		//!<accounts for the local magnetic field errors
			double variation;		//!<accounts for the relative angle between magnetic north and true north

			std::string yawStatus;	//!<the current status of the yaw of compass 'N'
			std::string pitchStatus;//!<the current status of the pitch of the compass 'N'
			std::string rollStatus;	//!<the current status of the roll of the compass 'N'

			mrpt::hwdrivers::CSerialPort serialPort;//!<hardware interface to the compass serial port

			/**
			 * @brief parses a response string from the HMR3000 Compass Module and updates the current state
			 */
			void parseResponse( const std::string & data);

			/*
			 * @brief offers compensation for yaw.  within about 10 degrees at worst.
			 * @param deg - the raw degrees measure
			 * @return double - the compensated degrees value
			 */
			double CompensateYaw(double deg);

			bool sendCommand( std::string command, std::string expectedResponse );

		};
	}
}
#endif /* COMPASS_H_ */
