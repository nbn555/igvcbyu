/*
 * GpsDevice.h
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#ifndef GPSDEVICE_H_
#define GPSDEVICE_H_
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <string>
#include <mrpt/utils/CConfigFile.h>

class GPS: public mrpt::hwdrivers::CGPSInterface {
public:
	GPS(const int Buffer_Length = 500);
	virtual ~GPS();
	void initialize();
	void initConfig( mrpt::utils::CConfigFileBase & configSource, const std::string & iniSection = "GPS");//mrpt doesn't make the loadConfig_sensorSpecific virtual so we have to load our specific config settings ourselves
	bool usesGpgga() const { return this->isGpggaUsed; };
	bool usesGprmc() const { return this->isGprmcUsed; };
	void dumpData(std::ostream & out );
protected:
private:
	std::string vendor;
	bool isGpggaUsed;
	bool isGprmcUsed;
	std::string portName;//These are needed for sending initial commands to the old GPS
	int baudRate;
	int processRate;
};

#endif /* GPSDEVICE_H_ */
