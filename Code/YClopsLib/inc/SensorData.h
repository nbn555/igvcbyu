/**
 * @file SensorData.h
 * @date Feb 24, 2011
 * @author tallred3
 * @brief Sensor Data Wrapper class definitions
 */

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include <ctime>
#include <mrpt/slam.h>

/**
 * @brief Sensor Data parent class
 */
class SensorData {
public:
	SensorData(): t(time(NULL)) { };
	virtual ~SensorData() {};
	time_t t;		//!Time stamp for when the sensor data was created
};

/**
 * @brief GPS sensor data object
 */
class GPSData: public SensorData {
public:
	GPSData( double latitude, double longitude, bool valid ): latitude(latitude), longitude(longitude), valid(valid) {};
	virtual ~GPSData() {};

	double latitude;	//!the read latitude of the gps observation
	double longitude;	//!the read longitude of the gps observation
	bool valid;			//!If the gps observation is valid or not

};

/**
 * @brief Compass Observation class
 */
class CompassData: public SensorData {
public:
	CompassData( double yaw, double pitch, double roll, bool yawValid, bool pitchValid, bool rollValid, bool degrees ):
		yaw(yaw), pitch(pitch), roll(roll), yawValid(yawValid), pitchValid(pitchValid), rollValid(rollValid), degrees(degrees) {}
	virtual ~CompassData() {};

	double yaw;		//!The left right angle of the compass as measured from magnetic north
	double pitch;	//!The up down angle of the compass	good for +/-45 degrees
	double roll;	//!The cork screw angle of the compass good for +/- 45 degrees

	bool yawValid;	//!true if the yaw reading is valid
	bool pitchValid;//!true if the pitch reading is valid
	bool rollValid;	//!true if the roll reading is valid

	bool degrees;	//!true if the compass observation is measured in degrees

};

/**
 * @brief Encoder sensor observation
 */
class EncoderData: public SensorData {
public:
	EncoderData( int leftCount, int rightCount, int leftCountAbsolute, int rightCountAbsolute, int leftSpeed, int rightSpeed ): leftCount(leftCount), rightCount(rightCount), leftCountAbsolute(leftCountAbsolute), rightCountAbsolute(rightCountAbsolute), leftSpeed(leftSpeed), rightSpeed(rightSpeed) {};
	virtual ~EncoderData() {};

	int leftCount;			//!The current count of the left encoder
	int rightCount;			//!The current count of the right encoder
	int leftCountAbsolute;	//!The total count of the left encoder
	int rightCountAbsolute;	//!The total count of the right encoder
	int leftSpeed;			//!The current rate of change in the count of the left encoder
	int rightSpeed;			//!The current rate of change in the count of the right encoder
};

/**
 * @brief Camera observation
 */
class CameraData: public SensorData {
public:
	CameraData(mrpt::slam::CSimplePointsMap map) : map(map) {};
	virtual ~CameraData() {};
	mrpt::slam::CSimplePointsMap map; //!map of obstacles as observed by the camera
};

/**
 * @brief Lidar observation
 */
class LidarData: public SensorData {
public:
	LidarData(mrpt::slam::CObservation2DRangeScan map) : map(map) {};
	virtual ~LidarData() {};
	mrpt::slam::CObservation2DRangeScan map; //!map of obstacles as observed by the lidar
};

#endif /* SENSORDATA_H_ */
