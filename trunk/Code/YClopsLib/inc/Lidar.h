/*
 * Lidar.h
 *
 *  Created on: Feb 25, 2011
 *      Author: tallred3
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include "YClopsSensor.h"

class Lidar: protected mrpt::hwdrivers::CSickLaserSerial, public YClopsSensor {
public:
	Lidar();

	void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);

	void init();

	void sensorProcess();

	SensorData * getData();

	void dumpData( std::ostream & out );

	virtual ~Lidar();
};

#endif /* LIDAR_H_ */
