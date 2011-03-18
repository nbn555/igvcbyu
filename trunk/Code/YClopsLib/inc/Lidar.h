/**
 * @file Lidar.h
 * @date Feb 25, 2011
 * @author tallred3
 * @brief Lidar Sensor Code
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include "YClopsSensor.h"

/**
 * @brief Interfaces with the SICK 290 laser range finder
 */
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
