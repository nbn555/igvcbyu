/*
 * WheelEncoder.h
 *
 *  Created on: Feb 24, 2011
 *      Author: tallred3
 */

#ifndef WHEELENCODER_H_
#define WHEELENCODER_H_

#include "YClopsSensor.h"

class WheelEncoder: public YClopsSensor {
public:
	WheelEncoder();

	/**
	 * load sensor specific values from the configuration file
	 */
	void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);

	/**
	 * initialize the sensor
	 */
	void init();

	/**
	 * collect data from the sensor and cache it
	 */
	void sensorProcess();

	/**
	 * SensorData needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
	 */
	SensorData * getData();

	/**
	 * dumps data out to the parameter out
	 * @param out - the stream to which to write the data
	 */
	void dumpData( std::ostream & out ) const;

	virtual ~WheelEncoder();
protected:
	unsigned int leftCount;
	unsigned int rightCount;
	unsigned int leftCountAbsolute;
	unsigned int rightCountAbsolute;

};

#endif /* WHEELENCODER_H_ */
