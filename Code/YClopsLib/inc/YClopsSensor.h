/**
 * @file YClopsSensor.h
 * @date Feb 24, 2011
 * @author tallred3
 * @brief YClops Sensor Base Class
 */

#ifndef YCLOPSSENSOR_H_
#define YCLOPSSENSOR_H_

#include <mrpt/utils/CConfigFile.h>
#include "SensorData.h"

class YClopsSensor {
public:
	YClopsSensor();

	/**
	 * @brief load sensor specific values from the configuration file
	 * @param[in] config the input configuration
	 * @param[in] sectionName The configuration section to parse
	 */
	virtual void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) = 0;

	/**
	 * @brief initialize the sensor
	 */
	virtual void init() = 0;

	/**
	 * @brief collect data from the sensor and cache it
	 */
	virtual void sensorProcess() = 0;

	/**
	 * @brief SensorData needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
	 * @return SensorData * allocated on the heap
	 */
	virtual SensorData * getData() = 0;

	/**
	 * @brief dumps data out to the parameter out
	 * @param[out] out The stream to which to write the data
	 */
	void dumpData( std::ostream & out ) const;

	/**
	 * @brief class destructor
	 */
	virtual ~YClopsSensor();
};

#endif /* YCLOPSSENSOR_H_ */
