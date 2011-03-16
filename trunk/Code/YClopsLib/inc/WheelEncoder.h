/**
 * @file WheelEncoder.h
 * @date Feb 24, 2011
 * @author tallred3
 * @brief Wheel Encoder Class Header
 */

#ifndef WHEELENCODER_H_
#define WHEELENCODER_H_

#include "YClopsSensor.h"

class WheelEncoder: public YClopsSensor {
public:
	WheelEncoder();

	/**
	 * @brief load sensor specific values from the configuration file
	 */
	void loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);

	/**
	 * @brief initialize the sensor
	 */
	void init();

	/**
	 * @brief collect data from the sensor and cache it
	 */
	void sensorProcess();

	/**
	 * @return SensorData* needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
	 */
	SensorData * getData();

	/**
	 * @brief dumps data out to the parameter out
	 * @param[out] the stream to which to write the data
	 */
	void dumpData( std::ostream & out ) const;

	virtual ~WheelEncoder();
protected:
	int leftCount;			//!<The current count of the left encoder
	int rightCount;			//!<The current count of the right encoder
	int leftCountAbsolute;	//!<The total count of the left encoder
	int rightCountAbsolute;	//!<The total count of the right encoder
	int leftEncoderSpeed;	//!<The current rate of change in the count of the left encoder
	int rightEncoderSpeed;	//!<The current rate of change in the count of the right encoder
	int encoderPPR;			//!<The configured pulses per revolution in the encoder

};

#endif /* WHEELENCODER_H_ */
