/*
 * YClopsSensor.h
 *
 *  Created on: Feb 24, 2011
 *      Author: tallred3
 */

#ifndef YCLOPSSENSOR_H_
#define YCLOPSSENSOR_H_

#include <mrpt/utils/CConfigFile.h>
#include "SensorData.h"

class YClopsSensor {
public:
	YClopsSensor();

	/**
	 * load sensor specific values from the configuration file
	 */
	virtual void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) = 0;

	/**
	 * initialize the sensor
	 */
	virtual void init() = 0;

	/**
	 * collect data from the sensor and cache it
	 */
	virtual void sensorProcess() = 0;

	/**
	 * SensorData needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
	 */
	virtual SensorData * getData() = 0;

	/**
	 * dumps data out to the parameter out
	 * @param out - the stream to which to write the data
	 */
	void dumpData( std::ostream & out ) const;

	/**
	 * class destructor
	 */
	virtual ~YClopsSensor();
};

#endif /* YCLOPSSENSOR_H_ */
