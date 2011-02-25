/*
 * Compass.h
 *
 *  Created on: Nov 4, 2010
 *      Author: igvcbyu
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

class Compass: public YClopsSensor {

public:

	Compass();

	void loadConfiguration(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName );
	void init();
	void sensorProcess();

	/**
	 * SensorData needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
	 */
	SensorData * getData();
	void dumpData( std::ostream & out ) const;

	virtual ~Compass();

	/**
	 * @return The last received Yaw value in degrees
	 */
	double getYaw() const { return (this->degrees?this->yaw:mrpt::utils::DEG2RAD(this->yaw)); };

	/**
	 * @return The last received Pitch value in degrees
	 */
	double getPitch() const { return (this->degrees?this->pitch:mrpt::utils::DEG2RAD(this->pitch)); };

	/**
	 * @return the last received Roll value in degrees
	 */
	double getRoll() const { return (this->degrees?this->roll:mrpt::utils::DEG2RAD(this->roll)); };

	/**
	 * returns true if the compass returned a valid yaw value
	 */
	bool isYawValid() const { return "N" == this->yawStatus.substr(0,1); };

	/**
	 * returns true if the compass returned a valid pitch value
	 */
	bool isPitchValid() const { return "N" == this->pitchStatus.substr(0,1); };

	/**
	 * returns true if the compass returned a valid roll value
	 */
	bool isRollValid() const { return "N" == this->rollStatus.substr(0,1); };

	/**
	 * returns true if the compass was set to output degrees
	 */
	bool isDegrees() const { return this->degrees; };

	static std::string computeChecksum( const std::string & sentence );

protected:
	void loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase&, const std::string& = "COMPASS" );

private:
	bool degrees;
	double yaw;
	double pitch;
	double roll;

	std::string yawStatus;
	std::string pitchStatus;
	std::string rollStatus;

	mrpt::hwdrivers::CSerialPort serialPort;

	/**
	 * parses a response from the HMR3000 Compass Module
	 */
	void parseResponse( const std::string & data);

	/**
	 * clears out the data for parsing
	 */
	void reset();
};

#endif /* COMPASS_H_ */
