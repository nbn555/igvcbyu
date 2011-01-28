/*
 * Compass.h
 *
 *  Created on: Nov 4, 2010
 *      Author: igvcbyu
 */

#ifndef COMPASS_H_
#define COMPASS_H_

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <string>

class Compass: public mrpt::hwdrivers::CGenericSensor, public mrpt::utils::CDebugOutputCapable {

public:
	Compass( bool degrees = false, const int bufferLength = 500 );//Currently buferLength isn't used std::string is.  But an optimization could be to use a buffer instead of a string.
	virtual ~Compass();
	void doProcess();
	const mrpt::hwdrivers::TSensorClassId* GetRuntimeClass() const; //DONT USE THIS. DONT REGISTER THIS SENSOR. RETURNS NULL.

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
