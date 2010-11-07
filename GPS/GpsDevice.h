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

class GpsDevice:public mrpt::hwdrivers::CGPSInterface {
public:
	GpsDevice(const std::string& fileName, const int Buffer_Length = 500);
	virtual ~GpsDevice();
private:
	mrpt::utils::CConfigFile configFile;
};

#endif /* GPSDEVICE_H_ */
