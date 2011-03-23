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

/**
 * @brief Interface to the Honeywell HMR3000 digital compass
 */
class Compass: public YClopsSensor {

public:

	Compass();

	void loadConfiguration(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName );
	void init();
	void sensorProcess();

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

protected:
	void loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase&, const std::string& = "COMPASS" );

private:



	int offset; //!< int offset is for compass if we decide to offset it to true north instead of magnetic north. Configurable in .ini file
	bool degrees;
	double prevYawDeg;//!< prevYawDeg is for case where compass returns an invalid yaw, it will return the prevYaw value instead in degrees (does not effect final output in radians or degrees)
	double yaw;
	double pitch;
	double roll;

	std::string yawStatus;	//!<the current status of the yaw of compass 'N'
	std::string pitchStatus;//!<the current status of the pitch of the compass 'N'
	std::string rollStatus;	//!<the current status of the roll of the compass 'N'

	mrpt::hwdrivers::CSerialPort serialPort;//!<hardware interface to the compass serial port

	/**
	 * parses a response from the HMR3000 Compass Module
	 */
	void parseResponse( const std::string & data);

	/**
	 * clears out the data for parsing
	 */
	void reset();

	/*
	 * offers compensation for yaw.  within about 10 degrees at worst.
	 */
	double CompensateYaw(double deg);
};

#endif /* COMPASS_H_ */
