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
protected:
	void loadConfig_sensorSpecific(mrpt::utils::CConfigFileBase & configSource, const std::string & iniSection = "GPS");
private:
	bool isGpggaUsed;
	bool isGprmcUsed;
};

#endif /* GPSDEVICE_H_ */
